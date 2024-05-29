import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    imu_config = os.path.join(get_package_share_directory('bno055'),'config','bno055_params.yaml')
    rpi_node = Node(
            package = 'rpi_pkg',
            executable = 'rpi_com_motors',
            output = 'screen',
        )

    imu_node = Node(
            package = 'bno055',
            executable = 'bno055',
            parameters = [imu_config],
    )

    return LaunchDescription([
        rpi_node,
        imu_node,
    ])
