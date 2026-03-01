import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # FarmTRX RTK GPS via standard NMEA0183 serial
    # RTK corrections are handled internally by the FarmTRX unit
    gps_params = os.path.join(
        get_package_share_directory("moxl"), 'config', 'gps.yaml')

    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            output='both',
            parameters=[gps_params],
        ),
    ])
