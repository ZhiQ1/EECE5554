import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    gps_port = LaunchConfiguration('gps_port')
    imu_port = LaunchConfiguration('imu_port')

    gps_launch = os.path.join(
        get_package_share_directory('gps_driver'),
        'launch',
        'rtk_driver.launch.py'
    )

    imu_launch = os.path.join(
        get_package_share_directory('vn_driver'),
        'launch',
        'driver.launch.py'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'gps_port',
            default_value='/dev/ttyACM0',
            description='Serial port for the RTK GNSS receiver'
        ),

        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the VN-100 IMU'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gps_launch),
            launch_arguments={'port': gps_port}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
            launch_arguments={'port': imu_port}.items()
        )

    ])
