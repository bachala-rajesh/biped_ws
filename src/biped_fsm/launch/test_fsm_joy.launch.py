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
from launch.substitutions import PythonExpression


def generate_launch_description():

    # ###############
    # Configurations
    # ###############
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    pkg_biped_teleop = get_package_share_directory("biped_teleop")

    # ###############
    # nodes
    # ###############

    # joystick node
    teleop_launch_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_biped_teleop, "launch", "biped_teleop_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )


    
    # mujoco fsm state node 
    mujoco_fsm_state_node = Node(
        package="biped_fsm",
        executable="mujoco_state_manager_node",
        name="mujoco_state_manager_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        emulate_tty=True,
    )
    

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            teleop_launch_node,
            mujoco_fsm_state_node,
        ]
    )
