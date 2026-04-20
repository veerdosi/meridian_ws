import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, sosfilt


class ButterworthFilter:
    """Stateful 2nd-order Butterworth low-pass filter for N channels."""

    def __init__(self, cutoff_hz: float, sample_rate: float, n_channels: int = 6):
        nyquist = sample_rate / 2.0
        self._sos = butter(2, cutoff_hz / nyquist, btype='low', output='sos')
        # zi shape: (n_sections, 2, n_channels)
        self._zi = np.zeros((self._sos.shape[0], 2, n_channels))

    def process(self, x: np.ndarray) -> np.ndarray:
        """Filter one sample across all channels; x shape (n_channels,)."""
        y = np.empty_like(x)
        for i in range(len(x)):
            result, self._zi[:, :, i] = sosfilt(
                self._sos, [x[i]], zi=self._zi[:, :, i]
            )
            y[i] = result[0]
        return y


class FtSensorNode(Node):
    def __init__(self):
        super().__init__('ft_sensor_node')

        self.declare_parameter('use_sim', True)
        self.declare_parameter('add_noise', False)
        self.declare_parameter('noise_std_force', 0.1)   # N
        self.declare_parameter('noise_std_torque', 0.01) # Nm
        self.declare_parameter('cutoff_hz', 200.0)

        self._use_sim = self.get_parameter('use_sim').value
        self._add_noise = self.get_parameter('add_noise').value
        self._noise_std_force = self.get_parameter('noise_std_force').value
        self._noise_std_torque = self.get_parameter('noise_std_torque').value
        cutoff_hz = self.get_parameter('cutoff_hz').value

        sample_rate = 1000.0  # Hz — matches the 1kHz timer
        self._filter = ButterworthFilter(cutoff_hz, sample_rate, n_channels=6)

        self._pub_filtered = self.create_publisher(
            WrenchStamped, '/ft_sensor/filtered', 10
        )
        self._pub_raw = self.create_publisher(
            WrenchStamped, '/ft_sensor/raw', 10
        )

        if self._use_sim:
            self._sub_sim = self.create_subscription(
                WrenchStamped,
                '/ft_sensor/raw_sim',
                self._sim_callback,
                10,
            )
        else:
            self.get_logger().warn(
                'use_sim is False — real hardware topic not implemented; node is idle.'
            )

    def _sim_callback(self, msg: WrenchStamped) -> None:
        raw = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ])

        if self._add_noise:
            noise = np.concatenate([
                np.random.normal(0.0, self._noise_std_force, 3),
                np.random.normal(0.0, self._noise_std_torque, 3),
            ])
            raw = raw + noise

        # Publish raw (post-noise, pre-filter)
        raw_msg = WrenchStamped()
        raw_msg.header = msg.header
        raw_msg.wrench.force.x = raw[0]
        raw_msg.wrench.force.y = raw[1]
        raw_msg.wrench.force.z = raw[2]
        raw_msg.wrench.torque.x = raw[3]
        raw_msg.wrench.torque.y = raw[4]
        raw_msg.wrench.torque.z = raw[5]
        self._pub_raw.publish(raw_msg)

        # Filter and publish
        filtered = self._filter.process(raw)
        filt_msg = WrenchStamped()
        filt_msg.header = msg.header
        filt_msg.wrench.force.x = filtered[0]
        filt_msg.wrench.force.y = filtered[1]
        filt_msg.wrench.force.z = filtered[2]
        filt_msg.wrench.torque.x = filtered[3]
        filt_msg.wrench.torque.y = filtered[4]
        filt_msg.wrench.torque.z = filtered[5]
        self._pub_filtered.publish(filt_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FtSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
