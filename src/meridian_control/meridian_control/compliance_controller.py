import time
import threading
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty

from meridian_control.msg import ComplianceState, ExecutionOutcome


class State(str, Enum):
    APPROACH = 'APPROACH'
    PRE_CONTACT = 'PRE_CONTACT'
    CONTACT_ACTIVE = 'CONTACT_ACTIVE'
    SEATED = 'SEATED'
    FAILURE = 'FAILURE'


class ComplianceController(Node):
    def __init__(self):
        super().__init__('compliance_controller')
        self._declare_params()
        self._read_params()
        self._init_state()
        self._setup_ros()
        self.get_logger().info('ComplianceController ready — waiting for sensor data')

    # ------------------------------------------------------------------ params

    def _declare_params(self):
        self.declare_parameter('target_position', [0.4, 0.0, 0.01])
        self.declare_parameter('approach_height', 0.05)
        self.declare_parameter('contact_threshold_n', 0.5)
        self.declare_parameter('seating_force_min_n', 2.0)
        self.declare_parameter('seating_force_max_n', 8.0)
        self.declare_parameter('seating_displacement_m', 0.008)
        self.declare_parameter('abort_force_ceiling_n', 20.0)
        self.declare_parameter('abort_torque_ceiling_nm', 2.0)
        self.declare_parameter('contact_timeout_sec', 10.0)
        self.declare_parameter('kp_xy', 500.0)
        self.declare_parameter('kp_z', 0.0)
        self.declare_parameter('target_insertion_force_n', 5.0)
        self.declare_parameter('force_gain', 0.0001)
        self.declare_parameter('kd_joints', 0.1)
        self.declare_parameter('max_episodes', 10)
        self.declare_parameter('reset_between_episodes', True)

    def _read_params(self):
        self._target_pos = np.array(
            self.get_parameter('target_position').value, dtype=float
        )
        self._approach_height = float(self.get_parameter('approach_height').value)
        self._contact_thresh = float(self.get_parameter('contact_threshold_n').value)
        self._seat_fz_min = float(self.get_parameter('seating_force_min_n').value)
        self._seat_fz_max = float(self.get_parameter('seating_force_max_n').value)
        self._seat_disp = float(self.get_parameter('seating_displacement_m').value)
        self._abort_force = float(self.get_parameter('abort_force_ceiling_n').value)
        self._abort_torque = float(self.get_parameter('abort_torque_ceiling_nm').value)
        self._contact_timeout = float(self.get_parameter('contact_timeout_sec').value)
        self._kp_xy = float(self.get_parameter('kp_xy').value)
        self._kp_z = float(self.get_parameter('kp_z').value)  # reserved; Z is force-controlled
        self._target_fz = float(self.get_parameter('target_insertion_force_n').value)
        self._force_gain = float(self.get_parameter('force_gain').value)
        self._kd = float(self.get_parameter('kd_joints').value)
        self._max_episodes = int(self.get_parameter('max_episodes').value)
        self._reset_between = bool(self.get_parameter('reset_between_episodes').value)

    # --------------------------------------------------------------- init state

    def _init_state(self):
        self._lock = threading.Lock()

        # Latest sensor data (written by subscription callbacks)
        self._q_current = np.zeros(6)
        self._q_dot = np.zeros(6)
        self._jacobian = np.eye(6)
        self._site_pos = np.zeros(3)
        self._have_joints = False
        self._have_jacobian = False
        self._have_pose = False

        # State machine
        self._state = State.APPROACH
        self._pre_contact_z_cmd = None
        self._last_pre_contact_tick = None
        self._contact_z = None
        self._contact_time = None
        self._peak_force = 0.0
        self._episode_start_time = None

        # Episode management
        self._episode_count = 0
        self._episodes = []
        self._episode_ending = False
        self._episode_end_time = None
        self._all_done = False

    # ----------------------------------------------------------------- ROS I/O

    def _setup_ros(self):
        self._pub_cmd = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )
        self._pub_state = self.create_publisher(
            ComplianceState, '/compliance_controller/state', 10
        )
        self._pub_outcome = self.create_publisher(
            ExecutionOutcome, '/execution_outcome', 10
        )

        self.create_subscription(
            WrenchStamped, '/ft_sensor/filtered', self._ft_callback, 10
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10
        )
        self.create_subscription(
            Float64MultiArray, '/jacobian', self._jacobian_callback, 10
        )
        self.create_subscription(
            PoseStamped, '/ft_sensor_site_pose', self._pose_callback, 10
        )

        self._reset_client = self.create_client(Empty, '/mujoco_sim_node/reset')

    # ------------------------------------------------------- sensor callbacks

    def _joint_callback(self, msg: JointState) -> None:
        with self._lock:
            if len(msg.position) >= 6 and len(msg.velocity) >= 6:
                self._q_current[:] = list(msg.position)[:6]
                self._q_dot[:] = list(msg.velocity)[:6]
            self._have_joints = True

    def _jacobian_callback(self, msg: Float64MultiArray) -> None:
        with self._lock:
            self._jacobian = np.array(msg.data, dtype=float).reshape(6, 6)
            self._have_jacobian = True

    def _pose_callback(self, msg: PoseStamped) -> None:
        with self._lock:
            self._site_pos[0] = msg.pose.position.x
            self._site_pos[1] = msg.pose.position.y
            self._site_pos[2] = msg.pose.position.z
            self._have_pose = True

    # ------------------------------------------------------------------ math

    def _pinv_dls(self, J: np.ndarray) -> np.ndarray:
        """Damped least-squares pseudoinverse — avoids singularity blow-up."""
        lam = 0.01
        return J.T @ np.linalg.inv(J @ J.T + lam * np.eye(J.shape[0]))

    @staticmethod
    def _scale_dq(dq: np.ndarray, max_dq: float = 0.02) -> np.ndarray:
        """Scale joint delta so the largest component is ≤ max_dq, preserving direction."""
        peak = np.max(np.abs(dq))
        if peak > max_dq:
            return dq * (max_dq / peak)
        return dq

    # -------------------------------------------------------------- main loop

    def _ft_callback(self, msg: WrenchStamped) -> None:
        fz = msg.wrench.force.z
        tx = msg.wrench.torque.x
        ty = msg.wrench.torque.y
        tz = msg.wrench.torque.z

        with self._lock:
            if not (self._have_joints and self._have_jacobian and self._have_pose):
                return
            if self._all_done:
                return
            # Atomic snapshot of all shared state
            q = self._q_current.copy()
            q_dot = self._q_dot.copy()
            J = self._jacobian.copy()
            site_pos = self._site_pos.copy()
            state = self._state
            episode_ending = self._episode_ending
            episode_end_time = self._episode_end_time
            contact_z = self._contact_z

        J_pinv = self._pinv_dls(J)

        # ── post-episode cooldown ──────────────────────────────────────────
        if episode_ending:
            if state == State.FAILURE:
                self._do_retract(q, q_dot, J_pinv, site_pos)
            if episode_end_time is not None and time.time() - episode_end_time >= 1.0:
                with self._lock:
                    self._episode_ending = False
                    self._episode_count += 1
                    episode_count = self._episode_count
                if episode_count >= self._max_episodes:
                    with self._lock:
                        self._all_done = True
                    self._print_summary()
                    return
                self._start_episode()
            self._publish_state(state, fz, contact_z, site_pos)
            return

        # ── abort checks (CONTACT_ACTIVE only) ──────────────────────────────
        if state == State.CONTACT_ACTIVE:
            total_torque = float(np.sqrt(tx**2 + ty**2 + tz**2))
            with self._lock:
                self._peak_force = max(self._peak_force, abs(fz))
            if abs(fz) > self._abort_force:
                self._end_episode(State.FAILURE, 'over_force', site_pos)
                return
            if total_torque > self._abort_torque:
                self._end_episode(State.FAILURE, 'over_torque', site_pos)
                return
            with self._lock:
                ct = self._contact_time
            if ct and (time.time() - ct) > self._contact_timeout:
                self._end_episode(State.FAILURE, 'timeout', site_pos)
                return

        # ── state machine ────────────────────────────────────────────────────
        if state == State.APPROACH:
            self._do_approach(q, q_dot, J_pinv, site_pos)
        elif state == State.PRE_CONTACT:
            self._do_pre_contact(q, q_dot, J_pinv, site_pos, fz)
        elif state == State.CONTACT_ACTIVE:
            self._do_contact_active(q, q_dot, J_pinv, site_pos, fz)

        with self._lock:
            contact_z = self._contact_z
        self._publish_state(state, fz, contact_z, site_pos)

    # ─────────────────────────────────────── state handlers

    def _do_approach(self, q, q_dot, J_pinv, site_pos):
        goal = np.array([
            self._target_pos[0],
            self._target_pos[1],
            self._approach_height,
        ])
        delta_x6 = np.zeros(6)
        delta_x6[:3] = goal - site_pos
        delta_q = self._scale_dq(J_pinv @ delta_x6)
        self._publish_cmd(q + delta_q - self._kd * q_dot)

        xy_err = np.linalg.norm(site_pos[:2] - self._target_pos[:2])
        if abs(site_pos[2] - self._approach_height) < 0.003 and xy_err < 0.005:
            with self._lock:
                self._state = State.PRE_CONTACT
                self._pre_contact_z_cmd = float(site_pos[2])
                self._episode_start_time = time.time()
            self.get_logger().info('APPROACH → PRE_CONTACT')

    def _do_pre_contact(self, q, q_dot, J_pinv, site_pos, fz):
        now = time.time()
        with self._lock:
            last = self._last_pre_contact_tick
            self._last_pre_contact_tick = now
        dt = (now - last) if last is not None else 0.005
        z_rate = 0.010  # 10 mm/s descent

        with self._lock:
            if self._pre_contact_z_cmd is None:
                self._pre_contact_z_cmd = float(site_pos[2])
            self._pre_contact_z_cmd -= z_rate * dt
            target_z = self._pre_contact_z_cmd

        delta_x6 = np.zeros(6)
        delta_x6[0] = self._target_pos[0] - site_pos[0]
        delta_x6[1] = self._target_pos[1] - site_pos[1]
        delta_x6[2] = target_z - site_pos[2]
        delta_q = self._scale_dq(J_pinv @ delta_x6)
        self._publish_cmd(q + delta_q - self._kd * q_dot)

        # Skip contact detection during the first 0.5 s — let arm settle
        with self._lock:
            ep_start = self._episode_start_time
        if ep_start is None or (time.time() - ep_start) < 0.5:
            return

        if abs(fz) > self._contact_thresh:
            with self._lock:
                self._state = State.CONTACT_ACTIVE
                self._contact_z = float(site_pos[2])
                self._contact_time = time.time()
                self._peak_force = abs(fz)
            self.get_logger().info('PRE_CONTACT → CONTACT_ACTIVE')

    def _do_contact_active(self, q, q_dot, J_pinv, site_pos, fz):
        # XY: position-controlled (kp_xy * dt scales the error to a per-step delta)
        delta_xy = np.zeros(6)
        delta_xy[0] = self._kp_xy * 0.005 * (self._target_pos[0] - site_pos[0])
        delta_xy[1] = self._kp_xy * 0.005 * (self._target_pos[1] - site_pos[1])
        delta_q_xy = J_pinv @ delta_xy

        # Z: force-controlled — cap per-step delta to ±1 mm to prevent slamming
        force_err = self._target_fz - fz
        delta_z_m = float(np.clip(self._force_gain * force_err, -0.001, 0.001))
        delta_z6 = np.zeros(6)
        delta_z6[2] = delta_z_m
        delta_q_z = J_pinv @ delta_z6

        delta_q = self._scale_dq(delta_q_xy + delta_q_z)
        self._publish_cmd(q + delta_q - self._kd * q_dot)

        with self._lock:
            contact_z = self._contact_z
            self._peak_force = max(self._peak_force, abs(fz))

        insertion_dist = float(contact_z - site_pos[2]) if contact_z else 0.0

        if (self._seat_fz_min <= fz <= self._seat_fz_max
                and insertion_dist >= self._seat_disp):
            self._end_episode(State.SEATED, '', site_pos)

    def _do_retract(self, q, q_dot, J_pinv, site_pos):
        goal = np.array([
            self._target_pos[0],
            self._target_pos[1],
            self._approach_height,
        ])
        delta_x6 = np.zeros(6)
        delta_x6[:3] = goal - site_pos
        delta_q = self._scale_dq(J_pinv @ delta_x6)
        self._publish_cmd(q + delta_q - self._kd * q_dot)

    # ─────────────────────────────────────── episode management

    def _end_episode(self, outcome: State, failure_mode: str, site_pos):
        with self._lock:
            if self._episode_ending:
                return
            ep_start = self._episode_start_time
            contact_z = self._contact_z
            peak = self._peak_force
            self._state = outcome
            self._episode_ending = True
            self._episode_end_time = time.time()
            end_time = self._episode_end_time

        duration = (end_time - ep_start) if ep_start else 0.0
        insertion_dist = max(0.0, float(contact_z - site_pos[2])) if contact_z else 0.0

        self._episodes.append({
            'outcome': outcome,
            'failure_mode': failure_mode,
            'peak_force': peak,
            'insertion_distance': insertion_dist,
            'duration': duration,
        })

        out_msg = ExecutionOutcome()
        out_msg.success = (outcome == State.SEATED)
        out_msg.failure_mode = failure_mode
        out_msg.peak_force = peak
        out_msg.insertion_distance = insertion_dist
        out_msg.duration_sec = duration
        out_msg.profile_id = ''
        self._pub_outcome.publish(out_msg)

        self.get_logger().info(
            f'Episode {self._episode_count + 1} → {outcome.value} '
            f'peak={peak:.2f}N dist={insertion_dist:.4f}m dur={duration:.2f}s'
            + (f' reason={failure_mode}' if failure_mode else '')
        )

        if self._reset_between and self._reset_client.service_is_ready():
            self._reset_client.call_async(Empty.Request())

    def _start_episode(self):
        with self._lock:
            self._state = State.APPROACH
            self._contact_z = None
            self._contact_time = None
            self._peak_force = 0.0
            self._episode_start_time = None
            self._pre_contact_z_cmd = None
            self._last_pre_contact_tick = None
        self.get_logger().info(f'Starting episode {self._episode_count + 1}')

    # ─────────────────────────────────────── publish helpers

    def _publish_cmd(self, q_cmd: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = q_cmd.tolist()
        self._pub_cmd.publish(msg)

    def _publish_state(self, state: State, fz: float, contact_z, site_pos) -> None:
        insertion_dist = max(0.0, float(contact_z - site_pos[2])) if contact_z else 0.0
        msg = ComplianceState()
        msg.state = state.value
        msg.active_profile_id = ''
        msg.fz_filtered = fz
        msg.insertion_distance = insertion_dist
        self._pub_state.publish(msg)

    # ─────────────────────────────────────── summary

    def _print_summary(self) -> None:
        successes = sum(1 for e in self._episodes if e['outcome'] == State.SEATED)
        sep = '-' * 75
        hdr = (
            f"{'Episode':>7} | {'Outcome':>8} | "
            f"{'Peak Force (N)':>14} | "
            f"{'Insertion Distance (m)':>22} | "
            f"{'Duration (s)':>12}"
        )
        self.get_logger().info('\n' + sep)
        self.get_logger().info(hdr)
        self.get_logger().info(sep)
        for i, ep in enumerate(self._episodes, 1):
            outcome_str = 'SUCCESS' if ep['outcome'] == State.SEATED else 'FAILURE'
            row = (
                f"{i:>7} | {outcome_str:>8} | "
                f"{ep['peak_force']:>14.2f} | "
                f"{ep['insertion_distance']:>22.4f} | "
                f"{ep['duration']:>12.2f}"
            )
            self.get_logger().info(row)
        self.get_logger().info(sep)
        self.get_logger().info(f'Total: {successes}/{len(self._episodes)} success')
        self.get_logger().info(sep)


def main(args=None):
    rclpy.init(args=args)
    node = ComplianceController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
