import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from em_robot.profile import load_profile


def _static_tf_node(name, parent_frame, child_frame, xyzrpy):
    values = [str(value) for value in xyzrpy]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=values + [parent_frame, child_frame],
        output="screen",
    )


def _build_nodes(context):
    profile_name = LaunchConfiguration("profile").perform(context)
    profile_path = LaunchConfiguration("profile_path").perform(context)
    profile_path = profile_path or None

    profile, resolved_profile_path = load_profile(
        profile_name=profile_name,
        profile_path=profile_path,
    )

    pkg_share = get_package_share_directory("em_robot")
    rviz_config = os.path.join(pkg_share, "rviz", "desk_localization_test.rviz")
    config_dir = os.path.join(pkg_share, "config")
    nodes = []

    movement_cfg = profile.get("movement", {})
    if movement_cfg.get("enabled", True):
        nodes.append(
            Node(
                package="em_robot",
                executable="em_movement",
                parameters=[movement_cfg],
                output="screen",
            )
        )

    imu_cfg = profile.get("imu", {})
    if imu_cfg.get("enabled", True):
        backend = imu_cfg.get("backend", "bno055")
        nodes.append(
            _static_tf_node(
                name="imu_static_tf",
                parent_frame=imu_cfg.get("static_tf_parent", "base_link"),
                child_frame=imu_cfg.get("static_tf_child", "bno055"),
                xyzrpy=imu_cfg.get("static_tf_xyzrpy", [0, 0, 0, 0, 0, 0]),
            )
        )

        if backend == "bno055":
            config_imu = os.path.join(
                get_package_share_directory("bno055"),
                "config",
                "bno055_params_i2c.yaml",
            )
            nodes.append(
                Node(
                    package="bno055",
                    executable="bno055",
                    parameters=[config_imu],
                    output="screen",
                )
            )
        elif backend == "fake":
            nodes.append(
                Node(
                    package="em_robot",
                    executable="fake_imu",
                    parameters=[imu_cfg],
                    output="screen",
                )
            )

    localization_cfg = profile.get("localization", {})
    if localization_cfg.get("enabled", True):
        nodes.append(
            Node(
                package="em_robot",
                executable="em_localization",
                parameters=[
                    {
                        "camera_backend": localization_cfg.get("camera_backend", "picamera2"),
                        "camera_source": str(localization_cfg.get("source", "0")),
                        "config_file": os.path.join(config_dir, "calibration.yaml"),
                    }
                ],
                output="screen",
            )
        )

    ekf_cfg = profile.get("ekf", {})
    if ekf_cfg.get("enabled", True):
        nodes.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[os.path.join(config_dir, ekf_cfg.get("config", "ekf.yaml"))],
                output="screen",
            )
        )

    visualization_cfg = profile.get("visualization", {})
    if visualization_cfg.get("enabled", False) and visualization_cfg.get("backend") == "foxglove":
        foxglove_launch = os.path.join(
            get_package_share_directory("foxglove_bridge"),
            "launch",
            "foxglove_bridge_launch.xml",
        )
        nodes.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(foxglove_launch),
                launch_arguments={
                    "port": str(visualization_cfg.get("port", 8765)),
                    "address": str(visualization_cfg.get("address", "0.0.0.0")),
                }.items(),
            )
        )

    if profile.get("start_rviz", False):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="em_robot_rviz",
                arguments=["-d", rviz_config],
                output="screen",
            )
        )

    print(f"Launching em_robot profile '{profile.get('profile_name', profile_name)}' from {resolved_profile_path}")
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "profile",
                default_value="real_robot",
                description="Name of the em_robot runtime profile.",
            ),
            DeclareLaunchArgument(
                "profile_path",
                default_value="",
                description="Optional explicit path to a runtime profile YAML file.",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
