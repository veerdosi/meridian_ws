from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_arg = DeclareLaunchArgument(
        'bag',
        description='Absolute path to the bag directory to replay',
    )
    bag_path = LaunchConfiguration('bag')

    ros2_bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path, '--clock'],
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

    return LaunchDescription([
        bag_arg,
        ros2_bag_play,
        foxglove_bridge,
    ])
