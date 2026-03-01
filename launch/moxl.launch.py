import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name = 'moxl'

    share_directory = get_package_share_directory(package_name)
    xacro_file = os.path.join(share_directory, 'description/robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'use_ros2_control': '1',
        'use_sim_time': '0'
    }).toxml()

    # Robot state publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': False}]
    )

    # Twist mux — combines joystick and autonomous cmd_vel
    twist_mux_params = os.path.join(share_directory, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
    )

    # ros2_control controller manager
    controller_params_file = os.path.join(share_directory, 'config', 'controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_config},
                    controller_params_file]
    )

    # Load controllers sequentially after controller_manager starts
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        twist_mux,
        controller_manager,

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager,
                on_start=[load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller],
            )
        ),

        # GPS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/gps.launch.py']),
        ),

        # Localization (robot_localization EKF + navsat_transform)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/localization.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
            }.items(),
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/nav2.launch.py']),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),

        # Mission control (toolpath planner + engine/blade stubs + orchestrator)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/mission.launch.py']),
        ),

        # Foxglove bridge for web-based monitoring
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory("foxglove_bridge"), '/launch/foxglove_bridge_launch.xml']),
        ),
    ])
