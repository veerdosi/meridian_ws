import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve the MJCF path relative to meridian_description's install share
    # Falls back to the source tree if the package hasn't been installed yet.
    try:
        desc_share = get_package_share_directory('meridian_description')
        mjcf_path = os.path.join(desc_share, 'assets', 'ur5e.xml')
    except Exception:
        mjcf_path = os.path.join(
            os.path.dirname(__file__),
            '..', '..', '..', '..', 'src',
            'meridian_description', 'assets', 'ur5e.xml',
        )

    mujoco_sim_node = Node(
        package='meridian_sim',
        executable='mujoco_sim_node',
        name='mujoco_sim_node',
        parameters=[{'mjcf_path': mjcf_path}],
        output='screen',
    )

    ft_sensor_node = Node(
        package='meridian_sensors',
        executable='ft_sensor_node',
        name='ft_sensor_node',
        parameters=[{
            'use_sim': True,
            'add_noise': True,
            'noise_std_force': 0.05,
            'noise_std_torque': 0.005,
            'cutoff_hz': 200.0,
        }],
        output='screen',
    )

    return LaunchDescription([mujoco_sim_node, ft_sensor_node])
