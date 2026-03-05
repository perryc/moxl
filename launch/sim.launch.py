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

    # Park position: ~30m north of runway 11/29 north edge
    # ENU (0, 211) → GPS (50.63640, -105.03180), facing south toward runway
    park_lat = 50.63640
    park_lon = -105.03180

    # Spawn at park position relative to Gazebo origin (= navsat datum)
    # Park GPS (50.63640, -105.03180) → ENU (2.8, 214.8) from datum
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-world', 'map',
                   '-name', 'moxl',
                   '-x', '2.8', '-y', '214.8', '-z', '0.2',
                   '-Y', '-1.5708'],
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

    # Kickstart diff_drive odom: open_loop controller won't publish odom
    # until it receives at least one cmd_vel. Timestamp (0,0) bypasses
    # the staleness check.
    odom_kickstart = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once',
             '/diff_drive_base_controller/cmd_vel',
             'geometry_msgs/msg/TwistStamped',
             '{header: {stamp: {sec: 0, nanosec: 0}}, twist: {linear: {x: 0.0}}}'],
        output='screen'
    )

    # Relay Gazebo IMU → localization topics.
    # heading_to_imu_node has no NMEA input in sim, so relay the Gazebo
    # IMU to the topics the EKF and navsat_transform expect.
    # heading_offset corrects Gazebo IMU (relative yaw from start=0) to
    # absolute world-frame heading matching the robot's spawn yaw.
    spawn_yaw = -1.5708
    imu_relay = ExecuteProcess(
        cmd=['python3',
             os.path.join(get_package_share_directory(package_name),
                          'scripts', 'sim_imu_relay.py'),
             '--ros-args', '-p', 'use_sim_time:=true',
             '-p', f'heading_offset:={spawn_yaw}'],
        output='screen'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(package_name), '/launch/localization.launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'magnetic_declination_radians': '0.0',  # Gazebo IMU gives true heading
        }.items(),
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
                'overlap': 0.05,
                'start_lat': park_lat,
                'start_lon': park_lon,
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
                'park_lat': park_lat,
                'park_lon': park_lon,
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
        Node(
            package='toolpath_planner',
            executable='sdr_detector_node',
            name='sdr_detector',
            output='screen',
            parameters=[{
                'enabled': False,
                'frequency_mhz': 122.8,
                'quiet_period_sec': 300.0,
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
                'gz_args': '-r -v 4 {}'.format(world_path),
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
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_diff_controller,
                on_exit=[odom_kickstart],
            )
        ),
        gz_spawn_entity,
        imu_relay,
        localization,
        nav2,
        foxglove_bridge,
    ] + mission_nodes)
