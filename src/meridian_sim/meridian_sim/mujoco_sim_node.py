import threading
import time

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_srvs.srv import Empty


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Nominal config: IK solution placing ft_sensor_site at (0.4, 0.0, 0.15) —
# 15 cm directly above the USB-C target so APPROACH converges immediately.
_NOMINAL_QPOS = np.array([-0.2828, -0.7106, 1.5283, -0.3307, -1.6902, 0.0], dtype=np.float64)

# Approach height used by both the reset callback and the compliance controller.
_APPROACH_HEIGHT = 0.15


class MujocoSimNode(Node):
    def __init__(self):
        super().__init__('mujoco_sim_node')

        self.declare_parameter('mjcf_path', '')
        mjcf_path = self.get_parameter('mjcf_path').value
        if not mjcf_path:
            self.get_logger().fatal('mjcf_path param is required')
            raise RuntimeError('mjcf_path param is required')

        self._mjmodel = mujoco.MjModel.from_xml_path(mjcf_path)
        self._mjdata = mujoco.MjData(self._mjmodel)

        # Initialise to nominal pose
        self._mjdata.qpos[:6] = _NOMINAL_QPOS
        self._mjdata.ctrl[:6] = _NOMINAL_QPOS
        mujoco.mj_forward(self._mjmodel, self._mjdata)

        self._ft_site_id = mujoco.mj_name2id(
            self._mjmodel, mujoco.mjtObj.mjOBJ_SITE, 'ft_sensor_site'
        )
        if self._ft_site_id < 0:
            self.get_logger().fatal('ft_sensor_site not found in MJCF')
            raise RuntimeError('ft_sensor_site not found')

        self._lock = threading.Lock()

        # Shared state copied from sim thread → publish thread
        self._joint_pos = np.zeros(6)
        self._joint_vel = np.zeros(6)
        self._joint_eff = np.zeros(6)
        self._ft_force = np.zeros(3)
        self._ft_torque = np.zeros(3)
        self._jacobian = np.zeros((6, 6))
        self._site_pos = np.zeros(3)
        self._site_quat = np.zeros(4)

        self._pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_ft = self.create_publisher(WrenchStamped, '/ft_sensor/raw_sim', 10)
        self._pub_jac = self.create_publisher(
            Float64MultiArray, '/jacobian', 10
        )
        self._pub_pose = self.create_publisher(
            PoseStamped, '/ft_sensor_site_pose', 10
        )

        self._sub_cmd = self.create_subscription(
            Float64MultiArray, '/joint_commands', self._cmd_callback, 10
        )

        self._reset_srv = self.create_service(
            Empty, '~/reset', self._reset_callback
        )

        # Publish at 200 Hz from the main thread; sim runs at 1 kHz in a thread.
        self._pub_timer = self.create_timer(0.005, self._publish_callback)

        self._running = True
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()
        self.get_logger().info(f'MuJoCo sim started: {mjcf_path}')

    def _sim_loop(self) -> None:
        dt = self._mjmodel.opt.timestep  # honours whatever XML sets
        next_t = time.perf_counter()
        while self._running:
            with self._lock:
                mujoco.mj_step(self._mjmodel, self._mjdata)
                self._joint_pos[:] = self._mjdata.qpos[:6]
                self._joint_vel[:] = self._mjdata.qvel[:6]
                self._joint_eff[:] = self._mjdata.qfrc_actuator[:6]
                # sensordata layout: ft_force[0:3], ft_torque[3:6]
                self._ft_force[:] = self._mjdata.sensordata[0:3]
                self._ft_torque[:] = self._mjdata.sensordata[3:6]

                jacp = np.zeros((3, self._mjmodel.nv))
                jacr = np.zeros((3, self._mjmodel.nv))
                mujoco.mj_jacSite(
                    self._mjmodel, self._mjdata, jacp, jacr, self._ft_site_id
                )
                self._jacobian[:3, :] = jacp[:, :6]
                self._jacobian[3:, :] = jacr[:, :6]

                self._site_pos[:] = self._mjdata.site_xpos[self._ft_site_id]
                mujoco.mju_mat2Quat(self._site_quat, self._mjdata.site_xmat[self._ft_site_id])

            next_t += dt
            sleep_t = next_t - time.perf_counter()
            if sleep_t > 0:
                time.sleep(sleep_t)

    def _publish_callback(self) -> None:
        now = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = now
        js.name = JOINT_NAMES
        with self._lock:
            js.position = self._joint_pos.tolist()
            js.velocity = self._joint_vel.tolist()
            js.effort = self._joint_eff.tolist()
            force = self._ft_force.copy()
            torque = self._ft_torque.copy()
            jac_data = self._jacobian.flatten().tolist()
            spos = self._site_pos.copy()
            squat = self._site_quat.copy()
        self._pub_joints.publish(js)

        ws = WrenchStamped()
        ws.header.stamp = now
        ws.header.frame_id = 'ft_sensor_site'
        ws.wrench.force.x = force[0]
        ws.wrench.force.y = force[1]
        ws.wrench.force.z = force[2]
        ws.wrench.torque.x = torque[0]
        ws.wrench.torque.y = torque[1]
        ws.wrench.torque.z = torque[2]
        self._pub_ft.publish(ws)

        jac_msg = Float64MultiArray()
        dim0 = MultiArrayDimension()
        dim0.label = 'cartesian'
        dim0.size = 6
        dim0.stride = 36
        dim1 = MultiArrayDimension()
        dim1.label = 'joints'
        dim1.size = 6
        dim1.stride = 6
        jac_msg.layout.dim = [dim0, dim1]
        jac_msg.data = jac_data
        self._pub_jac.publish(jac_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = spos[0]
        pose_msg.pose.position.y = spos[1]
        pose_msg.pose.position.z = spos[2]
        pose_msg.pose.orientation.w = squat[0]
        pose_msg.pose.orientation.x = squat[1]
        pose_msg.pose.orientation.y = squat[2]
        pose_msg.pose.orientation.z = squat[3]
        self._pub_pose.publish(pose_msg)

    def _cmd_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 6:
            self.get_logger().warn(
                f'Expected 6 joint commands, got {len(msg.data)}; ignoring.'
            )
            return
        with self._lock:
            self._mjdata.ctrl[:6] = msg.data

    def _reset_callback(self, _request, response):
        with self._lock:
            self._mjdata.qpos[:6] = _NOMINAL_QPOS.copy()
            self._mjdata.qvel[:] = 0.0
            self._mjdata.ctrl[:6] = _NOMINAL_QPOS.copy()
            mujoco.mj_forward(self._mjmodel, self._mjdata)

            # Jacobian-based placement: fingertip ±2 mm XY from target_site
            target_id = mujoco.mj_name2id(
                self._mjmodel, mujoco.mjtObj.mjOBJ_SITE, 'target_site'
            )
            fingertip_id = mujoco.mj_name2id(
                self._mjmodel, mujoco.mjtObj.mjOBJ_BODY, 'fingertip'
            )
            target_pos = self._mjdata.site_xpos[target_id].copy()
            fingertip_pos = self._mjdata.xpos[fingertip_id].copy()

            xy_offset = np.array([
                np.random.uniform(-0.002, 0.002),
                np.random.uniform(-0.002, 0.002),
                0.0,
            ])
            desired_pos = target_pos + xy_offset
            desired_pos[2] = _APPROACH_HEIGHT  # reset to above-table hover, not to table surface

            jacp = np.zeros((3, self._mjmodel.nv))
            mujoco.mj_jacBody(self._mjmodel, self._mjdata, jacp, None, fingertip_id)
            J = jacp[:, :6]  # first 6 DoFs
            dp = desired_pos - fingertip_pos
            dq, _, _, _ = np.linalg.lstsq(J, dp, rcond=None)
            self._mjdata.qpos[:6] += np.clip(dq, -0.05, 0.05)
            self._mjdata.ctrl[:6] = self._mjdata.qpos[:6].copy()
            mujoco.mj_forward(self._mjmodel, self._mjdata)

        self.get_logger().info('Sim reset to randomised near-target pose')
        return response

    def destroy_node(self) -> None:
        self._running = False
        self._sim_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
