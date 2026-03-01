# ROS2 and Gazebo launch file for the mobile robot model

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    robotXacroName ='differential_drive_robot'

    namePackage = 'mobile_robot'

    modelFileRelativePath = 'model/robot.xacro'

    #worldFileRelativePath = 'worlds/empty_world.world'

    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)

    #pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    robotDescription = xacro.process_file(pathModelFile).toxml()

    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'))

    #gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 ', pathWorldFile], 'on_exit_shutdown': 'true'}.items())

    gazeboLaunch = IncludeLaunchDescription(gazebo_rosPackageLaunch, launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items())

    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotXacroName,
            '-topic', 'robot_description'
        ],
        output='screen',
    }

    nodeRobotDescription = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
                        'use_sim_time': True}]
    )

    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'paramters',
        'bridge_paramamters.yaml'
    )
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    launchDesctipionObject = LaunchDescription()

    launchDesctipionObject.add_action(gazeboLaunch)

    launchDesctipionObject.add_action(spawnModelNodeGazebo)
    launchDesctipionObject.add_action(nodeRobotStatePublisher)
    launchDesctipionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDesctipionObject
    