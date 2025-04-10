import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

pkg_name = 'em_robot'
config_ekf = os.path.join(
    get_package_share_directory(pkg_name),
    'config',
    'ekf.yaml'
)

config_imu = os.path.join(
    get_package_share_directory('bno055'),
    'config',
    'bno055_params.yaml'
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

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[config_ekf],
        output='screen'
    )

    imu = Node(
        package='bno055',
        executable='bno055',
        parameters=[config_imu]
    )

    localization_node = Node(
        package=pkg_name,
        executable='em_localization',
        output='screen',
    )

    return LaunchDescription([
        movement_node,
        localization_node,
        odom_node,
        ekf,
        imu,
    ])
