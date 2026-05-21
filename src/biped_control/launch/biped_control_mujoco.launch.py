import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import TimerAction
from launch.substitutions import PythonExpression


def generate_launch_description():

    # Path to controllers yaml
    controllers_yaml = os.path.join(
        get_package_share_directory("biped_control"),
        "config",
        "biped_ros2_controllers_mujoco.yaml",
    )

    # ###############
    # Configurations
    # ###############
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    sim_mode = LaunchConfiguration("sim_mode", default="mujoco")

    # ###############
    # nodes
    # ###############
    # controller manager node
    controller_manager_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        emulate_tty=True,
        remappings=[("~/robot_description", "/robot_description"), ("/imu_sensor_broadcaster/imu", "/imu/data")],
        
        on_exit=Shutdown(),
        condition=IfCondition(PythonExpression(["'", sim_mode, "' != 'gazebo'"])),
    )

    # Spawner nodes
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )
    
    blind_walk_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "blind_walk_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
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
        blind_walk_controller_spawner,
        imu_sensor_broadcaster_spawner,
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "sim_mode",
                default_value="mujoco",
                description="Simulation mode: gazebo, isaacsim or mujoco",
            ),
            controller_manager_node,
            TimerAction(
                period=2.0,
                actions=controller_spawners,
            ),
        ]
    )
