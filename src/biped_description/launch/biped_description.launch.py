#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description() -> LaunchDescription:

    ##################### 
    # Launch configurations
    ##################### 
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration('use_gui')
    

    ##################### 
    # urdf related 
    #####################
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('biped_description'))
    xacro_file = os.path.join(pkg_path,'urdf','biped_with_control.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}


    ##################### 
    # Nodes
    ##################### 
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        emulate_tty=True,
    )
    
    # GUI sliders (only when use_gui=true)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )
    
    

 
    ############################## creating LaunchDescription
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            
            DeclareLaunchArgument(
                'use_gui',
                default_value='true',
                description='Whether to show joint_state_publisher_gui sliders'
            ),
            
            
            robot_state_publisher_node,
            joint_state_publisher_gui,
        ]
    )
