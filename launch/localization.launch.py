import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("moxl")

    use_sim_time = LaunchConfiguration("use_sim_time")
    magnetic_declination = LaunchConfiguration("magnetic_declination_radians")

    localization_params_path = os.path.join(
        package_path, "config", "robot_localization.yaml"
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "magnetic_declination_radians",
                default_value="0.1257",
                description="Magnetic declination (0.0 for sim, real value for field)",
            ),

            # GPS heading bridge: QuaternionStamped → Imu
            # Speed-dependent covariance: trusts magnetometer at standstill,
            # backs off at speed so GPS-derived heading dominates.
            # NOT launched in sim — sim_imu_relay provides /gps/imu instead.
            Node(
                package="toolpath_planner",
                executable="heading_to_imu_node",
                name="heading_to_imu",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=UnlessCondition(use_sim_time),
            ),

            # EKF #1: odom frame — wheel odometry + GPS magnetometer heading
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_odom",
                output="screen",
                parameters=[localization_params_path, {"use_sim_time": use_sim_time}],
            ),

            # EKF #2: map frame — wheel odometry + GPS position + heading
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_se_map",
                output="screen",
                parameters=[localization_params_path, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("odometry/filtered", "odometry/filtered/map"),
                ],
            ),

            # NavSat Transform: converts GPS NavSatFix → odometry/gps
            # Uses magnetometer heading from /gps/imu (not odom yaw)
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[
                    localization_params_path,
                    {
                        "use_sim_time": use_sim_time,
                        "magnetic_declination_radians": magnetic_declination,
                    },
                ],
                remappings=[
                    ("odometry/filtered", "odometry/filtered/map"),
                    ("imu", "gps/imu"),
                ],
            ),
        ]
    )
