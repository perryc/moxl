"""Teleop-only launch for manual driving tests.

Brings up just enough to drive the robot manually:
  - URDF + robot_state_publisher
  - ros2_control (diff_drive_controller)
  - twist_mux (joystick priority)
  - Joystick driver + teleop_twist_joy
  - GPS + localization (for position monitoring)
  - Foxglove bridge (for monitoring)

Does NOT launch Nav2 or mission control.
Usage: ros2 launch moxl teleop.launch.py
"""

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
    share_directory = get_package_share_directory('moxl')

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

    # Twist mux
    twist_mux_params = os.path.join(share_directory, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')]
    )

    # ros2_control
    controller_params_file = os.path.join(share_directory, 'config', 'controllers.yaml')
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_config},
                    controller_params_file]
    )

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

        # Joystick
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/joystick.launch.py']),
        ),

        # GPS + Localization (for position monitoring while driving)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/gps.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([share_directory, '/launch/localization.launch.py']),
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # Foxglove bridge for monitoring
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                [get_package_share_directory("foxglove_bridge"),
                 '/launch/foxglove_bridge_launch.xml']),
        ),
    ])
