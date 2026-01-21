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
from launch.actions import AppendEnvironmentVariable


def generate_launch_description() -> LaunchDescription:
    #####################
    # get packages path
    #####################
    pkg_biped_bringup = get_package_share_directory("biped_bringup")
    pkg_biped_description = get_package_share_directory("biped_description")
    pkg_biped_control = get_package_share_directory("biped_control")

    #####################
    # launch arguments and configurations related
    #####################
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    sim_mode = LaunchConfiguration("sim_mode", default="real")
    use_gui = LaunchConfiguration("use_gui", default="false")

    #####################
    # Nodes
    #####################

    # simulation robot node
    sim_robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_biped_description, "launch", "biped_description.launch.py")
        ),
        launch_arguments={
            "use_gui": use_gui,
            "use_sim_time": use_sim_time,
            "sim_mode": sim_mode,
        }.items(),
    )
    
    # control node
    control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_biped_control, "launch", "biped_control_position.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "sim_mode": sim_mode,
        }.items(),
    )


    #####################
    # creating LaunchDescription
    #####################

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation  clock if true",
            ),
            DeclareLaunchArgument(
                "use_gui",
                default_value="false",
                description="Whether to show joint_state_publisher_gui sliders",
            ),
            sim_robot_node,
            control_node,
        ]
    )
