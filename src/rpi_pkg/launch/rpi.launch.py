import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rpi_node = Node(
            package = 'rpi_pkg',
            executable = 'rpi_com_motors',
            output = 'screen',
        )
    cam_node = Node(
            package = 'rpi_pkg',
            executable = 'rpi_cam',
            output = 'screen',
        )

    return LaunchDescription([
        rpi_node,
        cam_node,
    ])
