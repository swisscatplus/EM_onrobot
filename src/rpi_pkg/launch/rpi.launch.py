import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'rpi_pkg'

    rpi_node = Node(
        package = pkg_name,
        executable = 'rpi_motors',
        output = 'screen',
    )

    return LaunchDescription([
        rpi_node,
    ])
