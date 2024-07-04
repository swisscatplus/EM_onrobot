import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'rpi_pkg'  

    namespace = LaunchConfiguration('namespace')
 
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Specifying namespace for individual robot'
    )

    imu_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'bno055.yaml'
        )

    rpi_node = Node(
            package = 'rpi_pkg',
            executable = 'rpi_motors',
            output = 'screen',
        )
    cam_node = Node(
            package = 'rpi_pkg',
            executable = 'rpi_cam',
            output = 'screen',
        ) 
    imu_node=Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [imu_config]
    )

    return LaunchDescription([
        namespace_arg,
        rpi_node,
        cam_node,
        imu_node
    ])
