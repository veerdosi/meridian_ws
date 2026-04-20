import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_viewer_arg = DeclareLaunchArgument(
        'enable_viewer', default_value='false',
        description='Open the MuJoCo passive viewer on DISPLAY :99 (VNC)',
    )
    enable_viewer = LaunchConfiguration('enable_viewer')

    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode', default_value='false',
        description='Slow motion, scripted failure on ep2, SEATED hold, narration',
    )
    demo_mode = LaunchConfiguration('demo_mode')

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

    bag_dir = os.path.join(
        '/root/meridian_ws', 'bags', datetime.now().strftime('%Y%m%d_%H%M%S')
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
        parameters=[params_yaml, {'demo_mode': demo_mode}],
        output='screen',
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 10000000,
        }],
        output='screen',
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'sqlite3',
            '-o', bag_dir,
            '/ft_sensor/raw',
            '/ft_sensor/filtered',
            '/ft_sensor/raw_sim',
            '/ft_sensor_site_pose',
            '/joint_states',
            '/compliance_controller/state',
            '/execution_outcome',
            '/jacobian',
        ],
        output='screen',
    )

    return LaunchDescription([
        enable_viewer_arg,
        demo_mode_arg,
        mujoco_sim_node,
        ft_sensor_node,
        compliance_controller,
        foxglove_bridge,
        rosbag_record,
    ])
