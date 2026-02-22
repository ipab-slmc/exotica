import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('exotica_examples')
    actions = []

    actions.append(Node(
        package='exotica_examples',
        executable='example_cpp_core',
        name='example_cpp_core_node',
        output='screen',
    ))

    return LaunchDescription(actions)
