import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory("em_robot"),
        "rviz",
        "desk_localization_test.rviz",
    )

    marker_id = LaunchConfiguration("marker_id")
    marker_x = LaunchConfiguration("marker_x")
    marker_y = LaunchConfiguration("marker_y")
    marker_z = LaunchConfiguration("marker_z")
    marker_yaw = LaunchConfiguration("marker_yaw")
    start_rviz = LaunchConfiguration("start_rviz")
    marker_frame = PythonExpression(["'aruco_' + str(", marker_id, ")"])

    odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="desk_test_odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        output="screen",
    )

    marker_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="desk_test_marker_tf",
        arguments=[
            marker_x,
            marker_y,
            marker_z,
            marker_yaw,
            "0",
            "0",
            "map",
            marker_frame,
        ],
        output="screen",
    )

    localization_node = Node(
        package="em_robot",
        executable="em_localization",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="desk_test_rviz",
        arguments=["-d", rviz_config],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "marker_id",
                default_value="0",
                description="ArUco marker ID to publish as a static TF frame named aruco_<marker_id>.",
            ),
            DeclareLaunchArgument(
                "marker_x",
                default_value="0.0",
                description="Marker X position in the map frame.",
            ),
            DeclareLaunchArgument(
                "marker_y",
                default_value="0.0",
                description="Marker Y position in the map frame.",
            ),
            DeclareLaunchArgument(
                "marker_z",
                default_value="0.0",
                description="Marker Z position in the map frame.",
            ),
            DeclareLaunchArgument(
                "marker_yaw",
                default_value="0.0",
                description="Marker yaw in radians in the map frame.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the desk-localization test configuration.",
            ),
            odom_to_base,
            marker_tf,
            localization_node,
            rviz_node,
        ]
    )
