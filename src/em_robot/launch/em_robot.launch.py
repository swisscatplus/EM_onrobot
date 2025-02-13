from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'em_robot'

    movement_node = Node(
        package = pkg_name,
        executable = 'em_movement',
        name='em_movement',
        output = 'screen',
    )

    localization_node = Node(
        package=pkg_name,
        executable='em_localization',
        name='em_localization',
        output='screen',
    )

    return LaunchDescription([
        movement_node,
        localization_node
    ])