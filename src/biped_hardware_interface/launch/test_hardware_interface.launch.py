import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue




def generate_launch_description():
    
    # Path to controllers yaml
    controllers_yaml = os.path.join(
        get_package_share_directory('biped_hardware_interface'),
        'config',
        'test_controllers.yaml',
    )
    
    
    # ###############
    # Configurations
    # ###############
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    sim_mode = LaunchConfiguration('sim_mode')
    
   
    ##################### 
    # urdf related 
    #####################
    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('biped_description'))
    xacro_file = os.path.join(pkg_path,'urdf','biped_with_sensors_control.urdf.xacro')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), 
        " ",
        xacro_file,
        " ",
        "sim_mode:=", 
        sim_mode
    ])

    #Wrap it in ParameterValue 
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }


    
    
    
    # ###############
    # nodes
    # ###############
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
            ],
        emulate_tty=True,
    )
    
    #controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("~/robot_description", "/robot_description")
        ],
        output="screen",
        emulate_tty=True,
    )


   
    
    
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'sim_mode',
            default_value='real',
            description='Simulation mode: gazebo or isaacsim or real',
        ),
        
        robot_state_publisher_node,
        controller_manager_node,

    ])
