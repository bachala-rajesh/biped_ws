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


def generate_launch_description() -> LaunchDescription:
   
    #####################
    # get packages path 
    #####################
    pkg_biped_bringup = get_package_share_directory("biped_bringup_sim")
    
    
    
    ##################### 
    # launch arguments and configurations related 
    #####################
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = LaunchConfiguration('use_gui', default='false')



    #####################
    # Nodes 
    #####################
    
    # simulation robot node
    sim_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_biped_bringup, "launch", "sim_biped.launch.py")),
        launch_arguments={
            "use_gui": use_gui,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    
    # gazebo world node
    world_path = os.path.join(
        pkg_biped_bringup,
        "world",
        "ignition_worlds",
        "empty_world.sdf",
    )
    world_sdf = LaunchConfiguration("world_sdf", default=world_path)
    gz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            # -r = run immediately, -v4 = verbose, then the world file name
            "gz_args": ["-r ", world_sdf],
        }.items(),
    )

    # Start ros_gz_bridge
    gazebo_bridge_config_path = PathJoinSubstitution([pkg_biped_bringup, "config", "gz_bridge.yaml"])
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"use_sim_time": use_sim_time}, {"config_file": gazebo_bridge_config_path}],
        output="screen",
        emulate_tty=True,
    )
    
    
    # Spawn robot node
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    z_pose = LaunchConfiguration("z_pose", default="0.90")
    yaw_pose = LaunchConfiguration("yaw_pose", default="0.00")

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description",
                    "-x", x_pose,
                    "-y", y_pose,
                    "-z", z_pose,      
                    "-Y", yaw_pose,    
                    ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        emulate_tty=True,
        output="screen",
    )
    
    

    #####################
    # creating LaunchDescription
    #####################
    
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                'use_gui',
                default_value='false',
                description='Whether to show joint_state_publisher_gui sliders'
            ),
            
            sim_robot_node,
            gz_node,
            gz_bridge_node,
            spawn_robot_node,
        ]
    )
