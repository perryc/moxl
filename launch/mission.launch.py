import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("moxl")

    airstrip_file = os.path.join(package_path, "config", "airstrips", "CDS2.json")

    return LaunchDescription(
        [
            # Toolpath planner — generates mowing strips from runway data
            Node(
                package="toolpath_planner",
                executable="toolpath_node",
                name="toolpath_node",
                output="screen",
                parameters=[
                    {
                        "airstrip_file": airstrip_file,
                        "runway": "11/29",
                        "cutting_width": 1.52,
                        "overlap": 0.15,
                    }
                ],
            ),

            # Engine controller (stub — real GPIO implementation later)
            Node(
                package="toolpath_planner",
                executable="engine_controller_node",
                name="engine_controller",
                output="screen",
            ),

            # Blade controller (stub — real GPIO implementation later)
            Node(
                package="toolpath_planner",
                executable="blade_controller_node",
                name="blade_controller",
                output="screen",
            ),

            # Mission orchestrator — sequences the full mowing mission
            Node(
                package="toolpath_planner",
                executable="mission_node",
                name="mission_node",
                output="screen",
                parameters=[
                    {
                        "park_lat": 50.63860,
                        "park_lon": -105.04030,
                    }
                ],
            ),

            # Safety monitor — GPS watchdog + e-stop (LiDAR stubbed)
            Node(
                package="toolpath_planner",
                executable="safety_monitor_node",
                name="safety_monitor",
                output="screen",
                parameters=[
                    {
                        "gps_timeout_sec": 2.0,
                        "require_rtk": True,
                    }
                ],
            ),
        ]
    )
