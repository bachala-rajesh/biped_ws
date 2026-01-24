import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('biped_rl_control'),
        'config',
        'biped_rl_control_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='biped_rl_control',
            executable='policy_runner_node',
            name='biped_rl_control_node',
            output='screen',
            parameters=[config],
        ),
    ])
