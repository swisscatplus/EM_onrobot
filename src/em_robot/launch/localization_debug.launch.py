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
        "localization_debug.rviz",
    )

    marker_id = LaunchConfiguration("marker_id")
    marker_x = LaunchConfiguration("marker_x")
    marker_y = LaunchConfiguration("marker_y")
    marker_z = LaunchConfiguration("marker_z")
    marker_yaw = LaunchConfiguration("marker_yaw")
    start_rviz = LaunchConfiguration("start_rviz")
    camera_backend = LaunchConfiguration("camera_backend")
    camera_source = LaunchConfiguration("camera_source")
    marker_frame = PythonExpression(["'aruco_' + str(", marker_id, ")"])

    odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="localization_debug_odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        output="screen",
    )

    marker_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="localization_debug_marker_tf",
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
        executable="localization",
        parameters=[
            {
                "camera_backend": camera_backend,
                "camera_source": camera_source,
            }
        ],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="localization_debug_rviz",
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
                description="Start RViz with the localization debug configuration.",
            ),
            DeclareLaunchArgument(
                "camera_backend",
                default_value="opencv",
                description="Camera backend to use for localization tests.",
            ),
            DeclareLaunchArgument(
                "camera_source",
                default_value="0",
                description="OpenCV camera index or video file path.",
            ),
            odom_to_base,
            marker_tf,
            localization_node,
            rviz_node,
        ]
    )
