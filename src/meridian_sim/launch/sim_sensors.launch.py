import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer', default_value='false',
        description='Open the MuJoCo passive viewer on DISPLAY :99 (VNC)',
    )
    enable_viewer = LaunchConfiguration('enable_viewer')

    try:
        desc_share = get_package_share_directory('meridian_description')
        mjcf_path = os.path.join(desc_share, 'assets', 'ur5e.xml')
    except Exception:
        mjcf_path = os.path.join(
            os.path.dirname(__file__),
            '..', '..', '..', '..', 'src',
            'meridian_description', 'assets', 'ur5e.xml',
        )

    try:
        sim_share = get_package_share_directory('meridian_sim')
        params_yaml = os.path.join(sim_share, 'config', 'compliance_params.yaml')
    except Exception:
        params_yaml = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'compliance_params.yaml'
        )

    mujoco_sim_node = Node(
        package='meridian_sim',
        executable='mujoco_sim_node',
        name='mujoco_sim_node',
        parameters=[{'mjcf_path': mjcf_path, 'enable_viewer': enable_viewer}],
        output='screen',
        additional_env={'DISPLAY': ':99'},
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

    compliance_controller = Node(
        package='meridian_control',
        executable='compliance_controller',
        name='compliance_controller',
        parameters=[params_yaml],
        output='screen',
    )

    return LaunchDescription([
        enable_viewer_arg,
        mujoco_sim_node,
        ft_sensor_node,
        compliance_controller,
    ])
