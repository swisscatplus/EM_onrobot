import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg_name = 'em_robot'
config_file = os.path.join(
    get_package_share_directory(pkg_name),
    'config',
    'ekf.yaml'
)

def generate_launch_description():
    movement_node = Node(
        package=pkg_name,
        executable='em_movement',
        output='screen',
    )

    odom_node = Node(
        package=pkg_name,
        executable='em_odom',
        output='screen',
    )

    localization_node = Node(
        package=pkg_name,
        executable='em_localization',
        output='screen',
    )

    return LaunchDescription([
        movement_node,
        #localization_node,
        #odom_node,
    ])
