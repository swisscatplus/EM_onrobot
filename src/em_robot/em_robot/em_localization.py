#!/usr/bin/env python3
import math
import os

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from em_robot_srv.srv import SetInitialPose
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
detector_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, detector_params)


def detect_aruco_markers(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
    corners_list, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return []

    return [
        {"id": int(marker_id), "corners": corners[0].astype(np.float32)}
        for corners, marker_id in zip(corners_list, ids.flatten())
    ]


def transform_to_matrix(transform):
    return concatenate_matrices(
        translation_matrix(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        ),
        quaternion_matrix(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        ),
    )


def build_transform(parent_frame, child_frame, transform_matrix, stamp):
    translation = translation_from_matrix(transform_matrix)
    quaternion = quaternion_from_matrix(transform_matrix)

    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.x = float(quaternion[0])
    transform.transform.rotation.y = float(quaternion[1])
    transform.transform.rotation.z = float(quaternion[2])
    transform.transform.rotation.w = float(quaternion[3])
    return transform


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_matrix(transform_matrix):
    quaternion = quaternion_from_matrix(transform_matrix)
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw


def blend_angles(previous_yaw, new_yaw, alpha):
    return math.atan2(
        (1.0 - alpha) * math.sin(previous_yaw) + alpha * math.sin(new_yaw),
        (1.0 - alpha) * math.cos(previous_yaw) + alpha * math.cos(new_yaw),
    )


class PicameraSource:
    def __init__(self, image_size, lens_position):
        from libcamera import controls
        from picamera2 import Picamera2

        self._camera = Picamera2()
        preview_config = self._camera.create_preview_configuration(
            main={"format": "XRGB8888", "size": image_size}
        )
        self._camera.configure(preview_config)
        self._camera.start()
        self._camera.set_controls(
            {"AfMode": controls.AfModeEnum.Manual, "LensPosition": lens_position}
        )

    def capture(self):
        return self._camera.capture_array()

    def close(self):
        self._camera.stop()


class OpenCVCameraSource:
    def __init__(self, source, image_size):
        source_value = int(source) if str(source).isdigit() else source
        self._capture = cv.VideoCapture(source_value)
        if image_size:
            self._capture.set(cv.CAP_PROP_FRAME_WIDTH, image_size[0])
            self._capture.set(cv.CAP_PROP_FRAME_HEIGHT, image_size[1])

        if not self._capture.isOpened():
            raise RuntimeError(f"Failed to open OpenCV camera source: {source}")

    def capture(self):
        ok, frame = self._capture.read()
        if not ok:
            raise RuntimeError("Failed to read frame from OpenCV camera source")
        return frame

    def close(self):
        self._capture.release()


def create_camera_source(camera_backend, source, image_size, lens_position):
    if camera_backend == "picamera2":
        return PicameraSource(image_size=image_size, lens_position=lens_position)
    if camera_backend == "opencv":
        return OpenCVCameraSource(source=source, image_size=image_size)
    raise ValueError(f"Unsupported camera backend: {camera_backend}")


class MarkerLocalizationNode(Node):
    def __init__(self):
        super().__init__("marker_localization_node")
        self.get_logger().info("Starting MarkerLocalizationNode...")

        self.declare_parameter("config_file", "")
        self.declare_parameter("camera_backend", "picamera2")
        self.declare_parameter("camera_source", "0")

        config_path = self.get_parameter("config_file").value or os.path.join(
            get_package_share_directory("em_robot"),
            "config",
            "calibration.yaml",
        )
        self.camera_backend = self.get_parameter("camera_backend").value
        self.camera_source = self.get_parameter("camera_source").value

        self.get_logger().info(f"Loading configuration: {config_path}")
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        self.camera_matrix = np.array(config.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get("dist_coeff", []), dtype=np.float32)
        self.marker_size = float(config.get("marker_size", 0.038))
        self.image_size = tuple(config.get("image_size", [1536, 864]))
        self.process_rate_hz = float(config.get("process_rate_hz", 5.0))
        self.lens_position = float(config.get("lens_position", 8.0))
        self.max_position_jump = float(config.get("max_position_jump", 0.25))
        self.max_yaw_jump = math.radians(float(config.get("max_yaw_jump_deg", 20.0)))
        self.max_reprojection_error = float(config.get("max_reprojection_error_px", 8.0))
        self.position_smoothing_alpha = float(config.get("position_smoothing_alpha", 0.2))
        self.yaw_smoothing_alpha = float(config.get("yaw_smoothing_alpha", 0.2))
        self.min_update_translation = float(config.get("min_update_translation", 0.01))
        self.min_update_yaw = math.radians(float(config.get("min_update_yaw_deg", 1.0)))

        camera_offset = config.get("camera_to_base", {})
        self.t_base_camera = concatenate_matrices(
            translation_matrix(
                [
                    float(camera_offset.get("x", 0.176)),
                    float(camera_offset.get("y", 0.0)),
                    float(camera_offset.get("z", 0.0)),
                ]
            ),
            quaternion_matrix(
                quaternion_from_euler(
                    float(camera_offset.get("roll", 0.0)),
                    float(camera_offset.get("pitch", 0.0)),
                    float(camera_offset.get("yaw", math.pi)),
                )
            ),
        )
        self.t_camera_base = inverse_matrix(self.t_base_camera)

        half_marker = self.marker_size / 2.0
        self.marker_object_points = np.array(
            [
                [-half_marker, half_marker, 0.0],
                [half_marker, half_marker, 0.0],
                [half_marker, -half_marker, 0.0],
                [-half_marker, -half_marker, 0.0],
            ],
            dtype=np.float32,
        )

        self.camera = create_camera_source(
            camera_backend=self.camera_backend,
            source=self.camera_source,
            image_size=self.image_size,
            lens_position=self.lens_position,
        )
        self.get_logger().info(
            f"Camera backend '{self.camera_backend}' initialized with source '{self.camera_source}'"
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.set_pose_srv = self.create_service(
            SetInitialPose, "set_initial_pose", self.handle_set_initial_pose
        )

        self.last_map_to_odom = np.identity(4, dtype=np.float64)
        self.last_marker_time = self.get_clock().now()
        self.last_debug_log_time = self.get_clock().now()
        self.last_tf_fallback_log_time = self.get_clock().now()
        self.last_update_log_time = self.get_clock().now()
        self.has_map_lock = False

        self.publish_static_transform()
        self.publish_initial_map_to_odom()

        timer_period = 1.0 / self.process_rate_hz if self.process_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.process_frame)
        self.map_odom_timer = self.create_timer(0.2, self.broadcast_last_map_to_odom)

    def publish_static_transform(self):
        transform = build_transform(
            "base_link",
            "camera_frame",
            self.t_base_camera,
            self.get_clock().now().to_msg(),
        )
        self.static_tf_broadcaster.sendTransform([transform])
        self.get_logger().info("Published static transform: base_link -> camera_frame")

    def publish_initial_map_to_odom(self):
        initial_transform = np.identity(4, dtype=np.float64)
        self.last_map_to_odom = initial_transform
        self.tf_broadcaster.sendTransform(
            build_transform("map", "odom", initial_transform, self.get_clock().now().to_msg())
        )
        self.get_logger().info("Initialized map -> odom to identity.")

    def broadcast_last_map_to_odom(self):
        self.tf_broadcaster.sendTransform(
            build_transform("map", "odom", self.last_map_to_odom, self.get_clock().now().to_msg())
        )

    def lookup_matrix(
        self,
        target_frame,
        source_frame,
        stamp,
        timeout_sec=0.2,
        allow_latest_fallback=False,
    ):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=timeout_sec),
            )
            return transform_to_matrix(transform)
        except Exception as exc:
            if not allow_latest_fallback:
                raise exc

            if "extrapolation into the future" not in str(exc).lower():
                raise exc

            latest_transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )

            now = self.get_clock().now()
            if (now - self.last_tf_fallback_log_time).nanoseconds > int(2e9):
                self.get_logger().warn(
                    f"TF for {target_frame} -> {source_frame} lagged behind the camera stamp; "
                    "falling back to the latest available transform."
                )
                self.last_tf_fallback_log_time = now

            return transform_to_matrix(latest_transform)

    def handle_set_initial_pose(self, request, response):
        try:
            odom_to_base = self.lookup_matrix("odom", "base_link", Time())
        except Exception as exc:
            self.get_logger().warn(f"No odom -> base_link available for initial pose: {exc}")
            response.success = False
            return response

        map_to_base = concatenate_matrices(
            translation_matrix([request.x, request.y, 0.0]),
            quaternion_matrix(quaternion_from_euler(0.0, 0.0, request.yaw)),
        )

        self.last_map_to_odom = map_to_base @ inverse_matrix(odom_to_base)
        self.has_map_lock = True
        self.tf_broadcaster.sendTransform(
            build_transform("map", "odom", self.last_map_to_odom, self.get_clock().now().to_msg())
        )

        self.get_logger().info(
            f"Updated map -> odom from service: x={request.x:.3f}, y={request.y:.3f}, yaw={request.yaw:.3f}"
        )
        response.success = True
        return response

    def estimate_marker_pose(self, corners):
        success, rvec, tvec = cv.solvePnP(
            self.marker_object_points,
            corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            return None, None

        projected_points, _ = cv.projectPoints(
            self.marker_object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.dist_coeffs,
        )
        reprojection_error = float(
            np.mean(np.linalg.norm(projected_points.reshape(-1, 2) - corners, axis=1))
        )
        if reprojection_error > self.max_reprojection_error:
            return None, reprojection_error

        rotation_matrix, _ = cv.Rodrigues(rvec)
        camera_to_marker = np.eye(4, dtype=np.float64)
        camera_to_marker[:3, :3] = rotation_matrix
        camera_to_marker[:3, 3] = tvec.reshape(3)
        return camera_to_marker, reprojection_error

    def process_frame(self):
        capture_stamp = self.get_clock().now()
        frame = self.camera.capture()
        detections = detect_aruco_markers(frame)
        if not detections:
            return

        try:
            odom_to_base = self.lookup_matrix(
                "odom",
                "base_link",
                capture_stamp,
                allow_latest_fallback=True,
            )
        except Exception as exc:
            self.get_logger().warn(f"Skipping vision update, no odom -> base_link transform: {exc}")
            return

        predicted_map_to_base = self.last_map_to_odom @ odom_to_base
        candidates = []
        rejected_for_gating = 0
        rejected_for_reprojection = 0
        missing_marker_tf = 0

        for detection in detections:
            camera_to_marker, reprojection_error = self.estimate_marker_pose(detection["corners"])
            if camera_to_marker is None:
                rejected_for_reprojection += 1
                continue

            try:
                map_to_marker = self.lookup_matrix(
                    "map", f"aruco_{detection['id']}", Time(), timeout_sec=0.5
                )
            except Exception:
                missing_marker_tf += 1
                continue

            map_to_camera = map_to_marker @ inverse_matrix(camera_to_marker)
            map_to_base = map_to_camera @ self.t_camera_base

            dx = map_to_base[0, 3] - predicted_map_to_base[0, 3]
            dy = map_to_base[1, 3] - predicted_map_to_base[1, 3]
            yaw_error = wrap_angle(yaw_from_matrix(map_to_base) - yaw_from_matrix(predicted_map_to_base))
            position_error = math.hypot(dx, dy)

            if self.has_map_lock and (
                position_error > self.max_position_jump or abs(yaw_error) > self.max_yaw_jump
            ):
                rejected_for_gating += 1
                continue

            candidates.append(
                {
                    "marker_id": detection["id"],
                    "map_to_base": map_to_base,
                    "reprojection_error": reprojection_error,
                }
            )

        if not candidates:
            now = self.get_clock().now()
            if (now - self.last_debug_log_time).nanoseconds > int(2e9):
                if not self.has_map_lock:
                    self.get_logger().info(
                        "Markers detected but no initial map lock was accepted. "
                        f"Rejected: reprojection={rejected_for_reprojection}, "
                        f"missing_marker_tf={missing_marker_tf}."
                    )
                else:
                    self.get_logger().info(
                        "Markers detected but no camera pose passed gating. "
                        f"Rejected: gating={rejected_for_gating}, "
                        f"reprojection={rejected_for_reprojection}, "
                        f"missing_marker_tf={missing_marker_tf}."
                    )
                self.last_debug_log_time = now
            return

        weights = np.array(
            [1.0 / max(candidate["reprojection_error"], 1e-3) for candidate in candidates],
            dtype=np.float64,
        )
        weights /= np.sum(weights)

        x = float(
            np.sum([weight * candidate["map_to_base"][0, 3] for weight, candidate in zip(weights, candidates)])
        )
        y = float(
            np.sum([weight * candidate["map_to_base"][1, 3] for weight, candidate in zip(weights, candidates)])
        )
        yaw_values = [yaw_from_matrix(candidate["map_to_base"]) for candidate in candidates]
        mean_yaw = math.atan2(
            float(np.sum([weight * math.sin(yaw) for weight, yaw in zip(weights, yaw_values)])),
            float(np.sum([weight * math.cos(yaw) for weight, yaw in zip(weights, yaw_values)])),
        )

        measured_map_to_base = concatenate_matrices(
            translation_matrix([x, y, 0.0]),
            quaternion_matrix(quaternion_from_euler(0.0, 0.0, mean_yaw)),
        )

        if self.has_map_lock:
            previous_map_to_base = self.last_map_to_odom @ odom_to_base
            smoothed_x = (
                (1.0 - self.position_smoothing_alpha) * previous_map_to_base[0, 3]
                + self.position_smoothing_alpha * x
            )
            smoothed_y = (
                (1.0 - self.position_smoothing_alpha) * previous_map_to_base[1, 3]
                + self.position_smoothing_alpha * y
            )
            smoothed_yaw = blend_angles(
                yaw_from_matrix(previous_map_to_base),
                mean_yaw,
                self.yaw_smoothing_alpha,
            )
        else:
            smoothed_x = x
            smoothed_y = y
            smoothed_yaw = mean_yaw

        fused_map_to_base = concatenate_matrices(
            translation_matrix([smoothed_x, smoothed_y, 0.0]),
            quaternion_matrix(quaternion_from_euler(0.0, 0.0, smoothed_yaw)),
        )

        if self.has_map_lock:
            smoothed_translation_delta = math.hypot(
                fused_map_to_base[0, 3] - previous_map_to_base[0, 3],
                fused_map_to_base[1, 3] - previous_map_to_base[1, 3],
            )
            smoothed_yaw_delta = abs(
                wrap_angle(yaw_from_matrix(fused_map_to_base) - yaw_from_matrix(previous_map_to_base))
            )
            if (
                smoothed_translation_delta < self.min_update_translation
                and smoothed_yaw_delta < self.min_update_yaw
            ):
                return

        self.last_map_to_odom = fused_map_to_base @ inverse_matrix(odom_to_base)
        self.has_map_lock = True
        self.last_marker_time = self.get_clock().now()

        self.tf_broadcaster.sendTransform(
            build_transform("map", "odom", self.last_map_to_odom, self.last_marker_time.to_msg())
        )

        if (self.last_marker_time - self.last_update_log_time).nanoseconds > int(2e9):
            marker_ids = ",".join(str(candidate["marker_id"]) for candidate in candidates)
            measured_yaw = yaw_from_matrix(measured_map_to_base)
            self.get_logger().info(
                f"Updated map -> odom using markers [{marker_ids}] at "
                f"x={smoothed_x:.3f}, y={smoothed_y:.3f}, yaw={smoothed_yaw:.3f} "
                f"(raw x={x:.3f}, y={y:.3f}, yaw={measured_yaw:.3f})"
            )
            self.last_update_log_time = self.last_marker_time

    def destroy_node(self):
        if hasattr(self, "camera") and self.camera is not None:
            self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MarkerLocalizationNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
