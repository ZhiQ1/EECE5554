from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for the RTK GNSS receiver'
    )

    gps_node = Node(
        package='gps_driver',
        executable='rtk_driver',
        name='gps_driver',
        output='screen',
        parameters=[
            {'port': LaunchConfiguration('port')}
        ]
    )

    return LaunchDescription([
        port_arg,
        gps_node
    ])
