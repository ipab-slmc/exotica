import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pyexotica.launch_helpers import shutdown_on_exit


def generate_launch_description():
    pkg = get_package_share_directory('exotica_examples')
    actions = []

    with open(os.path.join(pkg, 'resources/robots/lwr_simplified.urdf'), 'r') as f:
        robot_description = f.read()
    actions.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]))

    actions.append(Node(
        package='exotica_examples',
        executable='example_ompl_freebase',
        name='example_ompl_freebase_node',
        output='screen',
    ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'resources/rviz.rviz')]))

    return LaunchDescription(actions + shutdown_on_exit(actions))
