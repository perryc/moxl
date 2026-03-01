import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name = "moxl"

    xacro_file = os.path.join(get_package_share_directory(package_name), 'description/robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file, mappings={
        'use_ros2_control': '0',
        'use_sim_time': '1'
    }).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_drive_base_controller/cmd_vel')],
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-world', 'map',
                   '-name', 'moxl',
                   '-z', '0.2',
                   '-Y', '120.0'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/moxl/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        output='screen'
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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(package_name), '/launch/localization.launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(package_name), '/launch/nav2.launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(),
    )

    foxglove_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            [get_package_share_directory("foxglove_bridge"),
             '/launch/foxglove_bridge_launch.xml']),
        launch_arguments={'include_hidden': 'true'}.items(),
    )

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.sdf')

    # Mission control nodes (with RTK check disabled for sim)
    airstrip_file = os.path.join(get_package_share_directory(package_name), 'config', 'airstrips', 'CDS2.json')

    mission_nodes = [
        Node(
            package='toolpath_planner',
            executable='toolpath_node',
            name='toolpath_node',
            output='screen',
            parameters=[{
                'airstrip_file': airstrip_file,
                'runway': '11/29',
                'cutting_width': 1.52,
                'overlap': 0.15,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='toolpath_planner',
            executable='engine_controller_node',
            name='engine_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='toolpath_planner',
            executable='blade_controller_node',
            name='blade_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='toolpath_planner',
            executable='mission_node',
            name='mission_node',
            output='screen',
            parameters=[{
                'park_lat': 50.637833,
                'park_lon': -105.038583,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='toolpath_planner',
            executable='safety_monitor_node',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'gps_timeout_sec': 2.0,
                'require_rtk': False,
                'use_sim_time': True,
            }],
        ),
    ]

    return LaunchDescription([
        bridge,
        node_robot_state_publisher,
        twist_mux,
        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments={
                'gz_args': '-r -v 6 {}'.format(world_path),
            }.items()),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_controller],
            )
        ),
        gz_spawn_entity,
        localization,
        nav2,
        foxglove_bridge,
    ] + mission_nodes)
