#!/usr/bin/env python3
import math
import os

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped
from em_robot.aruco_utils import (
    build_marker_object_points,
    create_aruco_detector,
    detect_aruco_markers,
    estimate_marker_pose,
)
from em_robot_srv.srv import SetInitialPose
from em_robot.camera_sources import create_camera_source
from em_robot.diagnostic_utils import build_diagnostic_array, build_diagnostic_status
from em_robot.transform_utils import (
    blend_angles,
    build_planar_transform,
    build_transform,
    compute_map_to_base_from_marker,
    compute_map_to_odom_from_map_to_base,
    transform_to_matrix,
    wrap_angle,
    yaw_from_matrix,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
from tf_transformations import (
    concatenate_matrices,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization")
        self.get_logger().info("Starting localization node...")

        self.declare_parameter("config_file", "")
        self.declare_parameter("camera_backend", "picamera2")
        self.declare_parameter("camera_source", "0")
        self.declare_parameter("camera_loop", False)
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("debug_image_topic", "/localization/debug_image")
        self.declare_parameter("debug_image_scale", 1.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("vision_base_pose_topic", "/localization/vision_base_pose")
        self.declare_parameter("vision_camera_pose_topic", "/localization/vision_camera_pose")
        self.declare_parameter("turn_detection_threshold_deg", 5.0)

        config_path = self.get_parameter("config_file").value or os.path.join(
            get_package_share_directory("em_robot"),
            "config",
            "calibration.yaml",
        )
        self.camera_backend = self.get_parameter("camera_backend").value
        self.camera_source = self.get_parameter("camera_source").value
        self.camera_loop = bool(self.get_parameter("camera_loop").value)
        self.diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.debug_image_scale = float(self.get_parameter("debug_image_scale").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.vision_base_pose_topic = str(self.get_parameter("vision_base_pose_topic").value)
        self.vision_camera_pose_topic = str(self.get_parameter("vision_camera_pose_topic").value)
        self.turn_detection_threshold = math.radians(float(self.get_parameter("turn_detection_threshold_deg").value))

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
        self.aruco_detector = create_aruco_detector()

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
        self.marker_object_points = build_marker_object_points(self.marker_size)

        self.camera = create_camera_source(
            camera_backend=self.camera_backend,
            source=self.camera_source,
            image_size=self.image_size,
            lens_position=self.lens_position,
            loop=self.camera_loop,
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
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.debug_image_pub = (
            self.create_publisher(Image, self.debug_image_topic, 10)
            if self.debug_image_topic
            else None
        )
        self.vision_base_pose_pub = (
            self.create_publisher(PoseStamped, self.vision_base_pose_topic, 10)
            if self.vision_base_pose_topic
            else None
        )
        self.vision_camera_pose_pub = (
            self.create_publisher(PoseStamped, self.vision_camera_pose_topic, 10)
            if self.vision_camera_pose_topic
            else None
        )

        self.last_map_to_odom = np.identity(4, dtype=np.float64)
        self.last_marker_time = None
        self.last_frame_time = None
        self.last_debug_log_time = self.get_clock().now()
        self.last_tf_fallback_log_time = self.get_clock().now()
        self.last_update_log_time = self.get_clock().now()
        self.last_camera_error_log_time = self.get_clock().now()
        self.has_map_lock = False
        self.last_camera_error = ""
        self.last_detection_count = 0
        self.last_rejected_for_gating = 0
        self.last_rejected_for_reprojection = 0
        self.last_missing_marker_tf = 0
        self.last_candidate_count = 0
        self.last_used_tf_fallback = False
        self.last_tf_fallback_count = 0
        self.last_best_position_error = None
        self.last_best_yaw_error_deg = None
        self.last_best_reprojection_error = None
        self.last_debug_summary = "waiting for frames"

        self.publish_static_transform()
        self.publish_initial_map_to_odom()

        timer_period = 1.0 / self.process_rate_hz if self.process_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.process_frame)
        self.map_odom_timer = self.create_timer(0.2, self.broadcast_last_map_to_odom)
        diagnostics_period = 1.0 / self.diagnostics_rate_hz if self.diagnostics_rate_hz > 0.0 else 1.0
        self.diagnostics_timer = self.create_timer(diagnostics_period, self.publish_diagnostics)

    def publish_static_transform(self):
        transform = build_transform(
            self.base_frame,
            self.camera_frame,
            self.t_base_camera,
            self.get_clock().now().to_msg(),
        )
        self.static_tf_broadcaster.sendTransform([transform])
        self.get_logger().info(
            f"Published static transform: {self.base_frame} -> {self.camera_frame}"
        )

    def publish_initial_map_to_odom(self):
        initial_transform = np.identity(4, dtype=np.float64)
        self.last_map_to_odom = initial_transform
        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.odom_frame,
                initial_transform,
                self.get_clock().now().to_msg(),
            )
        )
        self.get_logger().info(
            f"Initialized {self.map_frame} -> {self.odom_frame} to identity."
        )

    def broadcast_last_map_to_odom(self):
        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.odom_frame,
                self.last_map_to_odom,
                self.get_clock().now().to_msg(),
            )
        )

    def build_pose_stamped(self, frame_id, transform_matrix, stamp):
        translation = transform_matrix[:3, 3]
        quaternion = quaternion_from_matrix(transform_matrix)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(translation[0])
        msg.pose.position.y = float(translation[1])
        msg.pose.position.z = float(translation[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        return msg

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
            self.last_used_tf_fallback = True
            self.last_tf_fallback_count += 1

            return transform_to_matrix(latest_transform)

    def handle_set_initial_pose(self, request, response):
        try:
            odom_to_base = self.lookup_matrix(self.odom_frame, self.base_frame, Time())
        except Exception as exc:
            self.get_logger().warn(
                f"No {self.odom_frame} -> {self.base_frame} available for initial pose: {exc}"
            )
            response.success = False
            return response

        map_to_base = build_planar_transform(request.x, request.y, request.yaw)

        self.last_map_to_odom = compute_map_to_odom_from_map_to_base(map_to_base, odom_to_base)
        self.has_map_lock = True
        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.odom_frame,
                self.last_map_to_odom,
                self.get_clock().now().to_msg(),
            )
        )

        self.get_logger().info(
            f"Updated {self.map_frame} -> {self.odom_frame} from service: "
            f"x={request.x:.3f}, y={request.y:.3f}, yaw={request.yaw:.3f}"
        )
        response.success = True
        return response

    def process_frame(self):
        capture_stamp = self.get_clock().now()
        try:
            frame = self.camera.capture()
        except Exception as exc:
            self.last_camera_error = str(exc)
            if (capture_stamp - self.last_camera_error_log_time).nanoseconds > int(2e9):
                self.get_logger().warn(f"Camera capture failed: {exc}")
                self.last_camera_error_log_time = capture_stamp
            return

        self.last_frame_time = capture_stamp
        self.last_camera_error = ""
        self.last_used_tf_fallback = False
        annotated_frame = frame.copy()
        detections = detect_aruco_markers(frame, detector=self.aruco_detector)
        self.last_detection_count = len(detections)
        self.last_rejected_for_gating = 0
        self.last_rejected_for_reprojection = 0
        self.last_missing_marker_tf = 0
        self.last_candidate_count = 0
        self.last_best_position_error = None
        self.last_best_yaw_error_deg = None
        self.last_best_reprojection_error = None
        if not detections:
            self.last_debug_summary = "no markers detected"
            self.publish_debug_image(annotated_frame, capture_stamp)
            return

        try:
            odom_to_base = self.lookup_matrix(
                self.odom_frame,
                self.base_frame,
                capture_stamp,
                allow_latest_fallback=True,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Skipping vision update, no {self.odom_frame} -> {self.base_frame} transform: {exc}"
            )
            return

        predicted_map_to_base = self.last_map_to_odom @ odom_to_base
        candidates = []
        rejected_for_gating = 0
        rejected_for_reprojection = 0
        missing_marker_tf = 0
        closest_position_error = None
        closest_yaw_error_deg = None
        best_reprojection_error = None

        for detection in detections:
            corners = detection["corners"]
            polyline = corners.reshape((-1, 1, 2)).astype(np.int32)
            cv.polylines(annotated_frame, [polyline], True, (0, 255, 0), 2)

            camera_to_marker, reprojection_error = estimate_marker_pose(
                corners,
                self.marker_object_points,
                self.camera_matrix,
                self.dist_coeffs,
                self.max_reprojection_error,
            )
            if camera_to_marker is None:
                rejected_for_reprojection += 1
                cv.putText(
                    annotated_frame,
                    f"id={detection['id']} rejected",
                    tuple(polyline[0, 0]),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv.LINE_AA,
                )
                continue

            cv.putText(
                annotated_frame,
                f"id={detection['id']} err={reprojection_error:.2f}",
                tuple(polyline[0, 0]),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv.LINE_AA,
            )

            try:
                map_to_marker = self.lookup_matrix(
                    self.map_frame,
                    f"aruco_{detection['id']}",
                    Time(),
                    timeout_sec=0.5,
                )
            except Exception:
                missing_marker_tf += 1
                continue

            map_to_base = compute_map_to_base_from_marker(
                map_to_marker,
                camera_to_marker,
                self.t_camera_base,
            )

            dx = map_to_base[0, 3] - predicted_map_to_base[0, 3]
            dy = map_to_base[1, 3] - predicted_map_to_base[1, 3]
            yaw_error = wrap_angle(yaw_from_matrix(map_to_base) - yaw_from_matrix(predicted_map_to_base))
            position_error = math.hypot(dx, dy)
            yaw_error_deg = math.degrees(yaw_error)

            if closest_position_error is None or position_error < closest_position_error:
                closest_position_error = position_error
                closest_yaw_error_deg = yaw_error_deg
            if best_reprojection_error is None or reprojection_error < best_reprojection_error:
                best_reprojection_error = reprojection_error

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
                    "position_error": position_error,
                    "yaw_error_deg": yaw_error_deg,
                }
            )

        self.last_rejected_for_gating = rejected_for_gating
        self.last_rejected_for_reprojection = rejected_for_reprojection
        self.last_missing_marker_tf = missing_marker_tf
        self.last_best_position_error = closest_position_error
        self.last_best_yaw_error_deg = closest_yaw_error_deg
        self.last_best_reprojection_error = best_reprojection_error

        if not candidates:
            self.last_debug_summary = (
                "markers seen, no accepted pose: "
                f"gate={rejected_for_gating} reproj={rejected_for_reprojection} "
                f"missing_tf={missing_marker_tf}"
            )
            cv.putText(
                annotated_frame,
                "Markers detected, but no localization update accepted",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv.LINE_AA,
            )
            self.publish_debug_image(annotated_frame, capture_stamp)
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

        self.last_candidate_count = len(candidates)
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

        measured_map_to_base = build_planar_transform(x, y, mean_yaw)

        if self.has_map_lock:
            previous_map_to_base = self.last_map_to_odom @ odom_to_base
            measured_yaw = yaw_from_matrix(measured_map_to_base)
            previous_yaw = yaw_from_matrix(previous_map_to_base)
            yaw_change = wrap_angle(measured_yaw - previous_yaw)
            
            # If large yaw change detected, use measurement with minimal smoothing
            # This prevents turns from being dampened by the smoothing filter
            if abs(yaw_change) > self.turn_detection_threshold:  # configurable threshold
                self.get_logger().debug(
                    f"Large turn detected ({math.degrees(yaw_change):.1f}°), "
                    f"reducing smoothing to preserve turn"
                )
                # Use higher vision confidence for turns
                effective_alpha = min(1.0, self.yaw_smoothing_alpha * 1.5)
            else:
                effective_alpha = self.yaw_smoothing_alpha
            
            smoothed_x = (
                (1.0 - self.position_smoothing_alpha) * previous_map_to_base[0, 3]
                + self.position_smoothing_alpha * x
            )
            smoothed_y = (
                (1.0 - self.position_smoothing_alpha) * previous_map_to_base[1, 3]
                + self.position_smoothing_alpha * y
            )
            smoothed_yaw = blend_angles(
                previous_yaw,
                measured_yaw,
                effective_alpha,
            )
        else:
            smoothed_x = x
            smoothed_y = y
            smoothed_yaw = mean_yaw

        fused_map_to_base = build_planar_transform(smoothed_x, smoothed_y, smoothed_yaw)

        self.last_debug_summary = (
            f"accepted {len(candidates)}/{len(detections)} marker poses, "
            f"best reproj={min(candidate['reprojection_error'] for candidate in candidates):.2f}px"
        )

        if self.has_map_lock:
            smoothed_translation_delta = math.hypot(
                fused_map_to_base[0, 3] - previous_map_to_base[0, 3],
                fused_map_to_base[1, 3] - previous_map_to_base[1, 3],
            )
            smoothed_yaw_delta = abs(
                wrap_angle(yaw_from_matrix(fused_map_to_base) - yaw_from_matrix(previous_map_to_base))
            )
            
            # DEBUG: Log yaw fusion details
            self.get_logger().debug(
                f"YAW FUSION: prev={math.degrees(previous_yaw):.1f}° → "
                f"meas={math.degrees(measured_yaw):.1f}° → "
                f"smooth={math.degrees(smoothed_yaw):.1f}° (alpha={effective_alpha:.2f}) "
                f"delta={math.degrees(smoothed_yaw_delta):.1f}° (min_update={math.degrees(self.min_update_yaw):.1f}°)"
            )
            
            if (
                smoothed_translation_delta < self.min_update_translation
                and smoothed_yaw_delta < self.min_update_yaw
            ):
                self.get_logger().debug(
                    f"Update rejected: too small (trans_delta={smoothed_translation_delta:.3f}m, "
                    f"yaw_delta={math.degrees(smoothed_yaw_delta):.1f}°)"
                )
                return

        self.last_map_to_odom = compute_map_to_odom_from_map_to_base(
            fused_map_to_base,
            odom_to_base,
        )
        self.has_map_lock = True
        self.last_marker_time = self.get_clock().now()

        if self.vision_base_pose_pub is not None:
            self.vision_base_pose_pub.publish(
                self.build_pose_stamped(
                    self.map_frame,
                    fused_map_to_base,
                    self.last_marker_time.to_msg(),
                )
            )
        if self.vision_camera_pose_pub is not None:
            fused_map_to_camera = fused_map_to_base @ self.t_base_camera
            self.vision_camera_pose_pub.publish(
                self.build_pose_stamped(
                    self.map_frame,
                    fused_map_to_camera,
                    self.last_marker_time.to_msg(),
                )
            )

        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.odom_frame,
                self.last_map_to_odom,
                self.last_marker_time.to_msg(),
            )
        )

        if (self.last_marker_time - self.last_update_log_time).nanoseconds > int(2e9):
            marker_ids = ",".join(str(candidate["marker_id"]) for candidate in candidates)
            self.get_logger().info(
                f"Updated {self.map_frame} -> {self.odom_frame} using markers [{marker_ids}] at "
                f"x={smoothed_x:.3f}, y={smoothed_y:.3f}, yaw={math.degrees(smoothed_yaw):.1f}° "
                f"(raw x={x:.3f}, y={y:.3f}, yaw={math.degrees(measured_yaw):.1f}°)"
            )
            self.last_update_log_time = self.last_marker_time

        cv.putText(
            annotated_frame,
            f"x={smoothed_x:.3f}, y={smoothed_y:.3f}, yaw={smoothed_yaw:.3f}",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 0),
            2,
            cv.LINE_AA,
        )
        self.publish_debug_image(annotated_frame, capture_stamp)

    def publish_debug_image(self, frame, stamp):
        if self.debug_image_pub is None:
            return

        overlay_lines = [
            f"detected={self.last_detection_count} accepted={self.last_candidate_count} "
            f"gate={self.last_rejected_for_gating} reproj={self.last_rejected_for_reprojection} "
            f"missing_tf={self.last_missing_marker_tf}",
            f"tf_fallback={'yes' if self.last_used_tf_fallback else 'no'} total={self.last_tf_fallback_count}",
        ]
        if self.last_best_position_error is not None:
            overlay_lines.append(
                "best raw delta: "
                f"{self.last_best_position_error:.3f} m, {self.last_best_yaw_error_deg:.1f} deg"
            )
        if self.last_best_reprojection_error is not None:
            overlay_lines.append(f"best reprojection: {self.last_best_reprojection_error:.2f} px")
        overlay_lines.append(self.last_debug_summary)

        for index, line in enumerate(overlay_lines):
            y = 60 + index * 24
            cv.putText(
                frame,
                line,
                (10, y),
                cv.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                3,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                line,
                (10, y),
                cv.FONT_HERSHEY_SIMPLEX,
                0.55,
                (30, 30, 30),
                1,
                cv.LINE_AA,
            )

        publish_frame = frame
        if 0.0 < self.debug_image_scale < 1.0:
            publish_frame = cv.resize(
                frame,
                None,
                fx=self.debug_image_scale,
                fy=self.debug_image_scale,
                interpolation=cv.INTER_AREA,
            )

        msg = Image()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.camera_frame
        msg.height = int(publish_frame.shape[0])
        msg.width = int(publish_frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(publish_frame.shape[1] * publish_frame.shape[2])
        msg.data = publish_frame.tobytes()
        self.debug_image_pub.publish(msg)

    def publish_diagnostics(self):
        now = self.get_clock().now()
        frame_age = None
        marker_age = None
        if self.last_frame_time is not None:
            frame_age = (now - self.last_frame_time).nanoseconds / 1e9
        if self.last_marker_time is not None:
            marker_age = (now - self.last_marker_time).nanoseconds / 1e9

        frame_timeout = max(1.0, 3.0 / self.process_rate_hz) if self.process_rate_hz > 0.0 else 1.0
        marker_timeout = max(2.0, 5.0 / self.process_rate_hz) if self.process_rate_hz > 0.0 else 2.0

        level = DiagnosticStatus.OK
        message = "Localization healthy"
        if self.last_camera_error:
            level = DiagnosticStatus.ERROR
            message = "Camera capture is failing"
        elif frame_age is None or frame_age > frame_timeout:
            level = DiagnosticStatus.WARN
            message = "Camera frames are stale"
        elif not self.has_map_lock:
            level = DiagnosticStatus.WARN
            message = "No localization map lock yet"
        elif marker_age is None or marker_age > marker_timeout:
            level = DiagnosticStatus.WARN
            message = "No recent marker-based localization update"

        status = build_diagnostic_status(
            "em_robot/localization",
            level,
            message,
            values={
                "camera_backend": self.camera_backend,
                "camera_source": self.camera_source,
                "camera_loop": self.camera_loop,
                "map_frame": self.map_frame,
                "odom_frame": self.odom_frame,
                "base_frame": self.base_frame,
                "camera_frame": self.camera_frame,
                "debug_image_scale": self.debug_image_scale,
                "has_map_lock": self.has_map_lock,
                "last_frame_age_s": "n/a" if frame_age is None else f"{frame_age:.3f}",
                "last_marker_age_s": "n/a" if marker_age is None else f"{marker_age:.3f}",
                "last_detection_count": self.last_detection_count,
                "accepted_candidate_count": self.last_candidate_count,
                "rejected_for_gating": self.last_rejected_for_gating,
                "rejected_for_reprojection": self.last_rejected_for_reprojection,
                "missing_marker_tf": self.last_missing_marker_tf,
                "used_tf_fallback_this_frame": self.last_used_tf_fallback,
                "total_tf_fallback_count": self.last_tf_fallback_count,
                "best_position_error_m": (
                    "n/a" if self.last_best_position_error is None else f"{self.last_best_position_error:.3f}"
                ),
                "best_yaw_error_deg": (
                    "n/a" if self.last_best_yaw_error_deg is None else f"{self.last_best_yaw_error_deg:.1f}"
                ),
                "best_reprojection_error_px": (
                    "n/a"
                    if self.last_best_reprojection_error is None
                    else f"{self.last_best_reprojection_error:.2f}"
                ),
                "debug_summary": self.last_debug_summary,
                "camera_error": self.last_camera_error or "none",
            },
        )
        self.diagnostics_pub.publish(
            build_diagnostic_array([status], self.get_clock().now().to_msg())
        )

    def destroy_node(self):
        if hasattr(self, "camera") and self.camera is not None:
            self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down localization node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
