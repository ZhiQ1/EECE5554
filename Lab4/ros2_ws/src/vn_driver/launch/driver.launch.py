from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB0',
            description='VectorNav VN-100 serial port'
        ),
        Node(
            package='vn_driver',
            executable='driver',
            name='vn_driver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': 115200
            }]
        )
    ])
