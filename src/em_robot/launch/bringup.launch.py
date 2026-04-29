import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from em_robot.profile_loader import load_profile


def _static_tf_node(name, parent_frame, child_frame, xyzrpy):
    values = [str(value) for value in xyzrpy]
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=values + [parent_frame, child_frame],
        output="screen",
    )


def _resolve_config_path(config_dir, value):
    if not value:
        return ""
    if os.path.isabs(value):
        return value
    return os.path.join(config_dir, value)


def _resolve_package_config_path(package_name, config_subdir, value):
    if not value:
        return ""
    if os.path.isabs(value):
        return value
    return os.path.join(get_package_share_directory(package_name), config_subdir, value)


def _find_repo_root(start_path):
    current = os.path.abspath(start_path)
    while current and current != os.path.dirname(current):
        if os.path.isdir(os.path.join(current, "CAD")) and os.path.isdir(
            os.path.join(current, "src")
        ):
            return current
        current = os.path.dirname(current)
    return None


def _load_robot_description(profile):
    robot_model_cfg = profile.get("robot_model", {})
    if not robot_model_cfg.get("enabled", True):
        return None

    pkg_share = get_package_share_directory("em_robot")
    repo_root = _find_repo_root(pkg_share)
    if repo_root is None:
        raise FileNotFoundError(
            f"Could not locate the workspace root from package share path {pkg_share}"
        )

    default_urdf = os.path.join(
        repo_root,
        "CAD",
        "EdyMobile_URDF_screencast_ROS",
        "URDF_screencast_ROS",
        "urdf",
        "simple_box_robot.urdf",
    )
    urdf_path = str(robot_model_cfg.get("urdf", default_urdf))
    if not os.path.isabs(urdf_path):
        urdf_path = os.path.join(repo_root, urdf_path)

    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        return urdf_file.read()


def _build_nodes(context):
    profile_name = LaunchConfiguration("profile").perform(context)
    profile_path = LaunchConfiguration("profile_path").perform(context)
    profile_path = profile_path or None

    profile, resolved_profile_path = load_profile(
        profile_name=profile_name,
        profile_path=profile_path,
    )

    pkg_share = get_package_share_directory("em_robot")
    rviz_config = os.path.join(
        pkg_share,
        "rviz",
        profile.get("rviz_config", "localization_debug.rviz"),
    )
    config_dir = os.path.join(pkg_share, "config")
    nodes = []

    robot_description = _load_robot_description(profile)
    if robot_description is not None:
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            )
        )

    movement_cfg = profile.get("movement", {})
    if movement_cfg.get("enabled", True):
        nodes.append(
            Node(
                package="em_robot",
                executable="base_controller",
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
            config_imu = _resolve_package_config_path(
                "bno055",
                "config",
                imu_cfg.get("params_file", "bno055_params_i2c.yaml"),
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
                    executable="imu_mock",
                    parameters=[imu_cfg],
                    output="screen",
                )
            )

    localization_cfg = profile.get("localization", {})
    marker_map_cfg = profile.get("marker_map", {})
    if marker_map_cfg.get("enabled", False):
        nodes.append(
            Node(
                package="em_robot",
                executable="marker_map_publisher",
                parameters=[
                    {
                        "config_file": _resolve_config_path(
                            config_dir,
                            marker_map_cfg.get("config", "marker_map.yaml"),
                        ),
                        "diagnostics_rate_hz": float(
                            marker_map_cfg.get("diagnostics_rate_hz", 1.0)
                        ),
                    }
                ],
                output="screen",
            )
        )

    if localization_cfg.get("enabled", True):
        nodes.append(
            Node(
                package="em_robot",
                executable="localization",
                parameters=[
                    {
                        "camera_backend": localization_cfg.get("camera_backend", "picamera2"),
                        "camera_source": str(localization_cfg.get("source", "0")),
                        "camera_loop": bool(localization_cfg.get("loop", False)),
                        "config_file": _resolve_config_path(
                            config_dir,
                            localization_cfg.get("config", "calibration.yaml"),
                        ),
                        "diagnostics_rate_hz": float(
                            localization_cfg.get("diagnostics_rate_hz", 1.0)
                        ),
                        "map_frame": str(localization_cfg.get("map_frame", "map")),
                        "odom_frame": str(localization_cfg.get("odom_frame", "odom")),
                        "base_frame": str(localization_cfg.get("base_frame", "base_link")),
                        "camera_frame": str(
                            localization_cfg.get("camera_frame", "camera_frame")
                        ),
                        "process_rate_hz": float(
                            localization_cfg.get(
                                "camera_rate_hz",
                                localization_cfg.get("process_rate_hz", 5.0),
                            )
                        ),
                        "map_odom_publish_rate_hz": float(
                            localization_cfg.get("map_odom_publish_rate_hz", 20.0)
                        ),
                        "publish_map_to_base": bool(
                            localization_cfg.get("publish_map_to_base", False)
                        ),
                        "processing_scale": float(
                            localization_cfg.get("processing_scale", 1.0)
                        ),
                        "debug_image_topic": str(
                            localization_cfg.get(
                                "debug_image_topic", "/localization/debug_image"
                            )
                        ),
                        "debug_image_scale": float(
                            localization_cfg.get("debug_image_scale", 1.0)
                        ),
                        "debug_image_rate_hz": float(
                            localization_cfg.get("debug_image_rate_hz", 0.0)
                        ),
                        "vision_base_pose_topic": str(
                            localization_cfg.get(
                                "vision_base_pose_topic", "/localization/vision_base_pose"
                            )
                        ),
                        "vision_camera_pose_topic": str(
                            localization_cfg.get(
                                "vision_camera_pose_topic",
                                "/localization/vision_camera_pose",
                            )
                        ),
                    }
                ],
                output="screen",
            )
        )

    aruco_test_cfg = profile.get("aruco_test", {})
    if aruco_test_cfg.get("enabled", False):
        nodes.append(
            Node(
                package="em_robot",
                executable="aruco_camera_test",
                parameters=[
                    {
                        "config_file": _resolve_config_path(
                            config_dir,
                            aruco_test_cfg.get("config", "calibration_ubuntu_test.yaml"),
                        ),
                        "marker_map_file": _resolve_config_path(
                            config_dir,
                            aruco_test_cfg.get("marker_map_config", ""),
                        ),
                        "camera_backend": aruco_test_cfg.get("camera_backend", "opencv"),
                        "camera_source": str(aruco_test_cfg.get("source", "0")),
                        "camera_loop": bool(aruco_test_cfg.get("loop", False)),
                        "diagnostics_rate_hz": float(
                            aruco_test_cfg.get("diagnostics_rate_hz", 1.0)
                        ),
                        "camera_frame": str(
                            aruco_test_cfg.get("camera_frame", "camera_frame")
                        ),
                        "map_camera_frame": str(
                            aruco_test_cfg.get("map_camera_frame", "camera_test")
                        ),
                        "debug_image_topic": str(
                            aruco_test_cfg.get("debug_image_topic", "/aruco_test/debug_image")
                        ),
                        "marker_pose_topic": str(
                            aruco_test_cfg.get("marker_pose_topic", "/aruco_test/marker_poses")
                        ),
                        "camera_pose_topic": str(
                            aruco_test_cfg.get("camera_pose_topic", "/aruco_test/camera_pose")
                        ),
                        "summary_topic": str(
                            aruco_test_cfg.get("summary_topic", "/aruco_test/summary")
                        ),
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
                parameters=[
                    _resolve_config_path(config_dir, ekf_cfg.get("config", "ekf_real.yaml"))
                ],
                output="screen",
            )
        )

    expected_imu_rate = imu_cfg.get("expected_rate_hz")
    if expected_imu_rate is None:
        if imu_cfg.get("backend", "bno055") == "bno055":
            expected_imu_rate = 100.0
        else:
            expected_imu_rate = imu_cfg.get("rate_hz", 30.0)

    diagnostics_cfg = profile.get("diagnostics", {})
    if diagnostics_cfg.get("enabled", True):
        nodes.append(
            Node(
                package="em_robot",
                executable="robot_diagnostics",
                parameters=[
                    {
                        "publish_rate_hz": float(diagnostics_cfg.get("publish_rate_hz", 1.0)),
                        "cmd_vel_timeout": float(movement_cfg.get("cmd_vel_timeout", 0.25)),
                        "expected_odom_rate": float(movement_cfg.get("odom_rate", 0.0)),
                        "expected_imu_rate": float(expected_imu_rate),
                        "expected_filtered_odom_rate": float(
                            diagnostics_cfg.get("expected_filtered_odom_rate", 30.0)
                        ),
                        "localization_expected": bool(localization_cfg.get("enabled", False)),
                        "filtered_odom_expected": bool(ekf_cfg.get("enabled", False)),
                    }
                ],
                output="screen",
            )
        )

    led_cfg = profile.get("leds", {})
    if led_cfg.get("enabled", False):
        nodes.append(
            Node(
                package="em_robot",
                executable="rgb_led_controller",
                parameters=[
                    {
                        "backend": str(led_cfg.get("backend", "mock")),
                        "active_low": bool(led_cfg.get("active_low", False)),
                        "brightness": float(led_cfg.get("brightness", 1.0)),
                        "diagnostics_rate_hz": float(led_cfg.get("diagnostics_rate_hz", 1.0)),
                        "all_topic": str(led_cfg.get("all_topic", "/leds/all/color")),
                        "front_name": str(led_cfg.get("front_name", "front")),
                        "front_topic": str(led_cfg.get("front_topic", "/leds/front/color")),
                        "front_pins": [int(pin) for pin in led_cfg.get("front_pins", [23, 24, 25])],
                        "front_color_order": str(led_cfg.get("front_color_order", "rgb")),
                        "rear_name": str(led_cfg.get("rear_name", "rear")),
                        "rear_topic": str(led_cfg.get("rear_topic", "/leds/rear/color")),
                        "rear_pins": [int(pin) for pin in led_cfg.get("rear_pins", [4, 17, 27])],
                        "rear_color_order": str(led_cfg.get("rear_color_order", "rgb")),
                    }
                ],
                output="screen",
            )
        )

    state_manager_cfg = profile.get("state_manager", {})
    if state_manager_cfg.get("enabled", False):
        nodes.append(
            Node(
                package="em_robot",
                executable="robot_state_manager",
                parameters=[
                    {
                        "publish_rate_hz": float(state_manager_cfg.get("publish_rate_hz", 5.0)),
                        "blink_period_s": float(state_manager_cfg.get("blink_period_s", 1.0)),
                        "cmd_vel_timeout": float(movement_cfg.get("cmd_vel_timeout", 0.25)),
                        "expected_odom_rate": float(movement_cfg.get("odom_rate", 30.0)),
                        "expected_imu_rate": float(expected_imu_rate),
                        "expected_filtered_odom_rate": float(
                            state_manager_cfg.get(
                                "expected_filtered_odom_rate",
                                diagnostics_cfg.get("expected_filtered_odom_rate", 30.0),
                            )
                        ),
                        "moving_linear_threshold": float(
                            state_manager_cfg.get("moving_linear_threshold", 0.02)
                        ),
                        "moving_angular_threshold": float(
                            state_manager_cfg.get("moving_angular_threshold", 0.1)
                        ),
                        "filtered_odom_expected": bool(ekf_cfg.get("enabled", False)),
                        "cmd_vel_topic": str(state_manager_cfg.get("cmd_vel_topic", "/cmd_vel")),
                        "odom_topic": str(state_manager_cfg.get("odom_topic", "/odomWheel")),
                        "imu_topic": str(state_manager_cfg.get("imu_topic", "/bno055/imu")),
                        "filtered_odom_topic": str(
                            state_manager_cfg.get("filtered_odom_topic", "/odometry/filtered")
                        ),
                        "diagnostics_topic": str(
                            state_manager_cfg.get("diagnostics_topic", "/diagnostics")
                        ),
                        "front_led_topic": str(
                            state_manager_cfg.get("front_led_topic", "/leds/front/color")
                        ),
                        "rear_led_topic": str(
                            state_manager_cfg.get("rear_led_topic", "/leds/rear/color")
                        ),
                        "mobility_state_topic": str(
                            state_manager_cfg.get("mobility_state_topic", "/robot_state/mobility")
                        ),
                        "health_state_topic": str(
                            state_manager_cfg.get("health_state_topic", "/robot_state/health")
                        ),
                        "overall_state_topic": str(
                            state_manager_cfg.get("overall_state_topic", "/robot_state/overall")
                        ),
                    }
                ],
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
