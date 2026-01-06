import os
from ament_index_python.packages import get_package_share_directory
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
from launch.actions import TimerAction



def generate_launch_description():
    
    # Path to controllers yaml
    controllers_yaml = os.path.join(
        get_package_share_directory('biped_control'),
        'config',
        'biped_controllers.yaml',
    )
    
    
    # ###############
    # Configurations
    # ###############
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    
    
    
    # ###############
    # nodes
    # ###############

    
    #controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        emulate_tty=True,
    )

    # Spawner nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "trajectory_controller",
            "--inactive",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    joints_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joints_position_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    controller_spawners = [
        joint_state_broadcaster_spawner,
        joints_position_controller_spawner,
    ]
    
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        controller_manager_node,
        TimerAction(
            period=2.0,
            actions=controller_spawners
        ),
    ])
