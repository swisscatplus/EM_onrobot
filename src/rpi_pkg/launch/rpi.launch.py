import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'rpi_pkg'

    namespace = LaunchConfiguration('namespace')
    cam_config = LaunchConfiguration('cam_config')

    cam_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'loca.yaml'
        )
    
    imu_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'bno055.yaml'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Specifying namespace for individual robot'
    )

    config_arg = DeclareLaunchArgument(
            'cam_config',
            default_value=cam_config_path,
            description='Path to the config file'
        )

    rpi_node = Node(
        namespace=namespace,
        package = pkg_name,
        executable = 'rpi_motors',
        output = 'screen',
        )
    
    cam_node = Node(
        namespace=namespace,
        package = pkg_name,
        executable = 'rpi_cam',
        output = 'screen',
        parameters=[cam_config_path],
        ) 
    
    imu_node=Node(
        namespace=namespace,
        package = 'bno055',
        executable = 'bno055',
        parameters = [imu_config_path]
    )

    return LaunchDescription([
        namespace_arg,
        config_arg,
        rpi_node,
        cam_node,
        imu_node
    ])
