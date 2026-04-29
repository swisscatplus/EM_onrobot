#!/usr/bin/env python3
import math
import os
import time

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from em_robot.aruco_utils import (
    build_marker_object_points,
    create_aruco_detector,
    detect_aruco_markers,
    estimate_marker_pose,
)
from em_robot.camera_sources import create_camera_source, normalize_to_bgr8
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
        self.declare_parameter(
            "camera_source",
            "0",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("camera_loop", False)
        self.declare_parameter("debug_image_topic", "/localization/debug_image")
        self.declare_parameter("debug_image_scale", 1.0)
        self.declare_parameter("debug_image_rate_hz", 0.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("vision_base_pose_topic", "/localization/vision_base_pose")
        self.declare_parameter("vision_camera_pose_topic", "/localization/vision_camera_pose")
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("map_odom_publish_rate_hz", 20.0)
        self.declare_parameter("publish_map_to_base", False)
        self.declare_parameter("processing_scale", 1.0)
        self.declare_parameter("transform_tolerance_sec", 0.2)
        self.declare_parameter("odom_lookup_timeout_sec", 0.3)
        self.declare_parameter("odom_future_tolerance_sec", 0.12)
        self.declare_parameter("max_capture_age_sec", 1.0)
        self.declare_parameter("max_position_jump", 0.45)
        self.declare_parameter("max_yaw_jump_deg", 35.0)
        self.declare_parameter("position_smoothing_alpha", 0.35)
        self.declare_parameter("yaw_smoothing_alpha", 0.35)
        self.declare_parameter("min_update_translation", 0.005)
        self.declare_parameter("min_update_yaw_deg", 0.5)

        config_path = self.get_parameter("config_file").value or os.path.join(
            get_package_share_directory("em_robot"),
            "config",
            "calibration.yaml",
        )

        self.camera_backend = self.get_parameter("camera_backend").value
        self.camera_source = self.get_parameter("camera_source").value
        self.camera_loop = bool(self.get_parameter("camera_loop").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.debug_image_scale = float(self.get_parameter("debug_image_scale").value)
        self.debug_image_rate_hz = float(self.get_parameter("debug_image_rate_hz").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.vision_base_pose_topic = str(self.get_parameter("vision_base_pose_topic").value)
        self.vision_camera_pose_topic = str(
            self.get_parameter("vision_camera_pose_topic").value
        )
        self.process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self.map_odom_publish_rate_hz = float(
            self.get_parameter("map_odom_publish_rate_hz").value
        )
        self.publish_map_to_base = bool(self.get_parameter("publish_map_to_base").value)
        self.processing_scale = float(self.get_parameter("processing_scale").value)
        self.transform_tolerance_sec = float(
            self.get_parameter("transform_tolerance_sec").value
        )
        self.odom_lookup_timeout_sec = float(
            self.get_parameter("odom_lookup_timeout_sec").value
        )
        self.odom_future_tolerance_sec = float(
            self.get_parameter("odom_future_tolerance_sec").value
        )
        self.max_capture_age_sec = float(self.get_parameter("max_capture_age_sec").value)
        self.max_position_jump = float(self.get_parameter("max_position_jump").value)
        self.max_yaw_jump = math.radians(
            float(self.get_parameter("max_yaw_jump_deg").value)
        )
        self.position_smoothing_alpha = float(
            self.get_parameter("position_smoothing_alpha").value
        )
        self.yaw_smoothing_alpha = float(self.get_parameter("yaw_smoothing_alpha").value)
        self.min_update_translation = float(
            self.get_parameter("min_update_translation").value
        )
        self.min_update_yaw = math.radians(
            float(self.get_parameter("min_update_yaw_deg").value)
        )

        self.get_logger().info(f"Loading configuration: {config_path}")
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        self.camera_matrix = np.array(config.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get("dist_coeff", []), dtype=np.float32)
        self.marker_size = float(config.get("marker_size", 0.040))
        self.image_size = tuple(config.get("image_size", [1280, 720]))
        self.lens_position = float(config.get("lens_position", 18.0))
        self.max_reprojection_error = float(config.get("max_reprojection_error_px", 8.0))

        self.marker_object_points = build_marker_object_points(self.marker_size)
        self.aruco_detector = create_aruco_detector()
        self.processing_camera_matrix = self.camera_matrix.copy()
        if self.processing_scale <= 0.0 or self.processing_scale > 1.0:
            raise ValueError("processing_scale must be in the range (0.0, 1.0].")
        if not 0.0 <= self.position_smoothing_alpha <= 1.0:
            raise ValueError("position_smoothing_alpha must be in the range [0.0, 1.0].")
        if not 0.0 <= self.yaw_smoothing_alpha <= 1.0:
            raise ValueError("yaw_smoothing_alpha must be in the range [0.0, 1.0].")
        if self.processing_scale != 1.0:
            self.processing_camera_matrix[0, 0] *= self.processing_scale
            self.processing_camera_matrix[1, 1] *= self.processing_scale
            self.processing_camera_matrix[0, 2] *= self.processing_scale
            self.processing_camera_matrix[1, 2] *= self.processing_scale

        camera_offset = config.get("camera_to_base", {})

        self.t_base_camera = concatenate_matrices(
            translation_matrix(
                [
                    float(camera_offset.get("x", 0.17446)),
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

        self.camera = create_camera_source(
            camera_backend=self.camera_backend,
            source=self.camera_source,
            image_size=self.image_size,
            lens_position=self.lens_position,
            loop=self.camera_loop,
        )

        self.get_logger().info(
            f"Camera backend '{self.camera_backend}' initialized with source "
            f"'{self.camera_source}'"
        )
        if self.processing_scale != 1.0:
            self.get_logger().info(
                f"Localization processing scale set to {self.processing_scale:.2f}"
            )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()

        # Important: spin_thread=True lets the TF listener keep receiving /tf and /tf_static
        # while this node is inside timer callbacks.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.odom_frame_available = False
        self.last_localization_matrix = None
        self.has_localization_lock = False
        self.localization_child_frame = (
            self.base_frame if self.publish_map_to_base else self.odom_frame
        )
        self.last_camera_to_marker_by_id = {}
        self.last_debug_image_publish_time = None
        self.last_update_log_time = self.get_clock().now()
        self.last_odom_fallback_log_time = self.get_clock().now()

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

        self.publish_static_transform()
        self.publish_initial_localization_transform()

        process_timer_period = 1.0 / self.process_rate_hz if self.process_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(process_timer_period, self.process_frame)

        map_odom_timer_period = (
            1.0 / self.map_odom_publish_rate_hz
            if self.map_odom_publish_rate_hz > 0.0
            else 0.05
        )
        self.map_odom_timer = self.create_timer(
            map_odom_timer_period,
            self.broadcast_last_localization_transform,
        )

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

    def current_transform_stamp(self):
        stamp = self.get_clock().now()
        if self.transform_tolerance_sec > 0.0:
            stamp = stamp + Duration(seconds=self.transform_tolerance_sec)
        return stamp.to_msg()

    def publish_initial_localization_transform(self):
        initial_matrix = build_planar_transform(0.0, 0.0, 0.0)
        self.last_localization_matrix = initial_matrix
        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.localization_child_frame,
                initial_matrix,
                self.current_transform_stamp(),
            )
        )

        self.get_logger().info(
            f"Published initial transform: {self.map_frame} -> {self.localization_child_frame}"
        )

    def broadcast_last_localization_transform(self):
        if self.last_localization_matrix is None:
            return

        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.localization_child_frame,
                self.last_localization_matrix,
                self.current_transform_stamp(),
            )
        )

    def build_pose_stamped(self, frame_id, transform_matrix, stamp):
        from tf_transformations import translation_from_matrix

        translation = translation_from_matrix(transform_matrix)
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

    def planarize_transform(self, transform_matrix):
        return build_planar_transform(
            float(transform_matrix[0, 3]),
            float(transform_matrix[1, 3]),
            yaw_from_matrix(transform_matrix),
        )

    def pose_delta(self, first_transform, second_transform):
        position_delta = math.hypot(
            float(first_transform[0, 3] - second_transform[0, 3]),
            float(first_transform[1, 3] - second_transform[1, 3]),
        )
        yaw_delta = wrap_angle(
            yaw_from_matrix(first_transform) - yaw_from_matrix(second_transform)
        )
        return position_delta, yaw_delta

    def fuse_candidate_map_to_base(self, candidates):
        weights = np.array(
            [1.0 / max(candidate["reprojection_error"], 1e-3) for candidate in candidates],
            dtype=np.float64,
        )
        weights /= np.sum(weights)

        x = float(
            np.sum(
                [
                    weight * candidate["map_to_base"][0, 3]
                    for weight, candidate in zip(weights, candidates)
                ]
            )
        )
        y = float(
            np.sum(
                [
                    weight * candidate["map_to_base"][1, 3]
                    for weight, candidate in zip(weights, candidates)
                ]
            )
        )
        yaw_values = [yaw_from_matrix(candidate["map_to_base"]) for candidate in candidates]
        yaw = math.atan2(
            float(
                np.sum(
                    [weight * math.sin(value) for weight, value in zip(weights, yaw_values)]
                )
            ),
            float(
                np.sum(
                    [weight * math.cos(value) for weight, value in zip(weights, yaw_values)]
                )
            ),
        )
        return build_planar_transform(x, y, yaw)

    def smooth_map_to_base(self, previous_map_to_base, measured_map_to_base):
        smoothed_x = (
            (1.0 - self.position_smoothing_alpha) * previous_map_to_base[0, 3]
            + self.position_smoothing_alpha * measured_map_to_base[0, 3]
        )
        smoothed_y = (
            (1.0 - self.position_smoothing_alpha) * previous_map_to_base[1, 3]
            + self.position_smoothing_alpha * measured_map_to_base[1, 3]
        )
        smoothed_yaw = blend_angles(
            yaw_from_matrix(previous_map_to_base),
            yaw_from_matrix(measured_map_to_base),
            self.yaw_smoothing_alpha,
        )
        return build_planar_transform(smoothed_x, smoothed_y, smoothed_yaw)

    def process_frame(self):
        if not self.publish_map_to_base and not self.odom_frame_available:
            try:
                self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                self.odom_frame_available = True
                self.get_logger().info(f"{self.odom_frame} frame is now available")
            except Exception:
                return

        try:
            captured_frame = (
                self.camera.capture_with_timestamp()
                if hasattr(self.camera, "capture_with_timestamp")
                else None
            )
            if captured_frame is None:
                frame = self.camera.capture()
                capture_timestamp_ns = time.time_ns()
                capture_stamp = self.get_clock().now()
                timestamp_source = "node_capture_time"
            else:
                frame = captured_frame.frame
                capture_timestamp_ns = int(captured_frame.timestamp_ns)
                timestamp_source = getattr(
                    captured_frame,
                    "timestamp_source",
                    "camera_capture_time",
                )
                capture_stamp = Time(
                    nanoseconds=capture_timestamp_ns,
                    clock_type=self.get_clock().clock_type,
                )
        except Exception as exc:
            self.get_logger().warn(f"Camera capture failed: {exc}")
            return

        now = self.get_clock().now()
        capture_age_sec = max(0.0, (now - capture_stamp).nanoseconds / 1e9)
        if (
            self.max_capture_age_sec > 0.0
            and capture_age_sec > self.max_capture_age_sec
        ):
            self.get_logger().warn(
                f"Skipping stale camera frame: age={capture_age_sec:.3f}s "
                f"source={timestamp_source}"
            )
            return

        processing_frame = frame
        if self.processing_scale != 1.0:
            processing_frame = cv.resize(
                frame,
                None,
                fx=self.processing_scale,
                fy=self.processing_scale,
                interpolation=cv.INTER_AREA,
            )

        annotated_frame = processing_frame.copy()

        detections = detect_aruco_markers(processing_frame, detector=self.aruco_detector)

        if not detections:
            if self.debug_image_pub:
                cv.putText(
                    annotated_frame,
                    "No markers detected",
                    (10, 30),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )
                self.publish_debug_image(annotated_frame, capture_stamp)
            return

        odom_to_base = None
        planar_odom_to_base = None
        predicted_map_to_base = None
        if not self.publish_map_to_base:
            try:
                odom_to_base = self.lookup_matrix(
                    self.odom_frame,
                    self.base_frame,
                    capture_stamp,
                    timeout_sec=self.odom_lookup_timeout_sec,
                    allow_latest_fallback=True,
                    max_fallback_delta_sec=self.odom_future_tolerance_sec,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Skipping vision update: no exact-enough {self.odom_frame} -> "
                    f"{self.base_frame} transform at camera stamp "
                    f"{capture_stamp.nanoseconds}: {exc}"
                )
                self.odom_frame_available = False
                return

            planar_odom_to_base = self.planarize_transform(odom_to_base)
            if self.has_localization_lock:
                predicted_map_to_base = self.last_localization_matrix @ planar_odom_to_base
        elif self.has_localization_lock:
            predicted_map_to_base = self.last_localization_matrix

        candidates = []
        rejected_for_gating = 0
        rejected_for_reprojection = 0
        missing_marker_tf = 0

        for detection in detections:
            marker_id = detection["id"]
            marker_frame = f"aruco_{marker_id}"
            corners = detection["corners"]
            polyline = corners.reshape((-1, 1, 2)).astype(np.int32)
            cv.polylines(annotated_frame, [polyline], True, (0, 255, 0), 2)

            camera_to_marker, reprojection_error = estimate_marker_pose(
                corners,
                self.marker_object_points,
                self.processing_camera_matrix,
                self.dist_coeffs,
                self.max_reprojection_error,
                reference_transform=self.last_camera_to_marker_by_id.get(marker_id),
            )

            if camera_to_marker is None:
                rejected_for_reprojection += 1
                error_text = "n/a" if reprojection_error is None else f"{reprojection_error:.2f}"
                cv.putText(
                    annotated_frame,
                    f"id={marker_id} rejected err={error_text}",
                    tuple(polyline[0, 0]),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv.LINE_AA,
                )
                continue

            self.last_camera_to_marker_by_id[marker_id] = camera_to_marker

            cv.putText(
                annotated_frame,
                f"id={marker_id} err={reprojection_error:.2f}",
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
                    marker_frame,
                    capture_stamp,
                    timeout_sec=0.0,
                    allow_latest_fallback=True,
                )
            except Exception:
                missing_marker_tf += 1
                cv.putText(
                    annotated_frame,
                    f"{marker_frame} missing TF",
                    tuple(polyline[0, 0] + np.array([0, 18])),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1,
                    cv.LINE_AA,
                )
                continue

            map_to_base = self.planarize_transform(
                compute_map_to_base_from_marker(
                    map_to_marker,
                    camera_to_marker,
                    self.t_camera_base,
                )
            )

            position_error = None
            yaw_error = None
            if predicted_map_to_base is not None:
                position_error, yaw_error = self.pose_delta(map_to_base, predicted_map_to_base)
                if (
                    position_error > self.max_position_jump
                    or abs(yaw_error) > self.max_yaw_jump
                ):
                    rejected_for_gating += 1
                    continue

            candidates.append(
                {
                    "marker_id": marker_id,
                    "map_to_base": map_to_base,
                    "reprojection_error": float(reprojection_error),
                    "position_error": position_error,
                    "yaw_error": yaw_error,
                }
            )

        if not candidates:
            cv.putText(
                annotated_frame,
                "Markers seen, no accepted pose",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv.LINE_AA,
            )
            self.get_logger().info(
                "Markers detected, but no localization update accepted: "
                f"reprojection={rejected_for_reprojection}, "
                f"missing_tf={missing_marker_tf}, gating={rejected_for_gating}"
            )
            self.publish_debug_image(annotated_frame, capture_stamp)
            return

        measured_map_to_base = self.fuse_candidate_map_to_base(candidates)
        if predicted_map_to_base is not None:
            fused_map_to_base = self.smooth_map_to_base(
                predicted_map_to_base,
                measured_map_to_base,
            )
        else:
            fused_map_to_base = measured_map_to_base

        if predicted_map_to_base is not None:
            update_translation, update_yaw = self.pose_delta(
                fused_map_to_base,
                predicted_map_to_base,
            )
            if (
                update_translation < self.min_update_translation
                and abs(update_yaw) < self.min_update_yaw
            ):
                self.publish_debug_image(annotated_frame, capture_stamp)
                return

        if self.publish_map_to_base:
            localization_matrix = fused_map_to_base
        else:
            localization_matrix = compute_map_to_odom_from_map_to_base(
                fused_map_to_base,
                planar_odom_to_base,
            )

        self.last_localization_matrix = localization_matrix
        self.has_localization_lock = True
        self.tf_broadcaster.sendTransform(
            build_transform(
                self.map_frame,
                self.localization_child_frame,
                localization_matrix,
                self.current_transform_stamp(),
            )
        )

        if self.vision_base_pose_pub is not None:
            self.vision_base_pose_pub.publish(
                self.build_pose_stamped(
                    self.map_frame,
                    measured_map_to_base,
                    capture_stamp.to_msg(),
                )
            )
        if self.vision_camera_pose_pub is not None:
            self.vision_camera_pose_pub.publish(
                self.build_pose_stamped(
                    self.map_frame,
                    measured_map_to_base @ self.t_base_camera,
                    capture_stamp.to_msg(),
                )
            )

        update_time = self.get_clock().now()
        if (update_time - self.last_update_log_time).nanoseconds > int(1e9):
            marker_ids = ",".join(str(candidate["marker_id"]) for candidate in candidates)
            measured_delta = (
                self.pose_delta(measured_map_to_base, predicted_map_to_base)
                if predicted_map_to_base is not None
                else (0.0, 0.0)
            )
            self.get_logger().info(
                f"Localization update from markers [{marker_ids}]: "
                f"map->base x={fused_map_to_base[0, 3]:.3f}, "
                f"y={fused_map_to_base[1, 3]:.3f}, "
                f"yaw={math.degrees(yaw_from_matrix(fused_map_to_base)):.1f} deg | "
                f"raw_delta={measured_delta[0]:.3f} m, "
                f"{math.degrees(measured_delta[1]):.1f} deg | "
                f"latency={capture_age_sec * 1000.0:.0f} ms | "
                f"stamp={timestamp_source}"
            )
            self.last_update_log_time = update_time

        cv.putText(
            annotated_frame,
            f"accepted {len(candidates)}/{len(detections)} markers "
            f"age={capture_age_sec * 1000.0:.0f} ms",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )
        cv.putText(
            annotated_frame,
            f"map->base x={fused_map_to_base[0, 3]:.3f}, "
            f"y={fused_map_to_base[1, 3]:.3f}, "
            f"yaw={math.degrees(yaw_from_matrix(fused_map_to_base)):.1f} deg",
            (10, 60),
            cv.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv.LINE_AA,
        )

        self.publish_debug_image(annotated_frame, capture_stamp)

    def lookup_matrix(
        self,
        target_frame,
        source_frame,
        stamp,
        timeout_sec=0.5,
        allow_latest_fallback=False,
        max_fallback_delta_sec=None,
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

            if "extrapolation" not in str(exc).lower():
                raise exc

            latest_transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )

            if max_fallback_delta_sec is not None and stamp.nanoseconds != 0:
                latest_stamp = latest_transform.header.stamp
                latest_ns = latest_stamp.sec * 1_000_000_000 + latest_stamp.nanosec
                fallback_delta_sec = (stamp.nanoseconds - latest_ns) / 1e9
                if (
                    fallback_delta_sec < 0.0
                    or fallback_delta_sec > max_fallback_delta_sec
                ):
                    raise exc

                now = self.get_clock().now()
                if (now - self.last_odom_fallback_log_time).nanoseconds > int(2e9):
                    self.get_logger().warn(
                        f"Using latest {target_frame} -> {source_frame} transform "
                        f"{fallback_delta_sec * 1000.0:.0f} ms before camera stamp."
                    )
                    self.last_odom_fallback_log_time = now
            else:
                self.get_logger().debug(
                    f"Using latest {target_frame} -> {source_frame} transform"
                )

            return transform_to_matrix(latest_transform)

    def publish_debug_image(self, frame, stamp):
        if self.debug_image_pub is None:
            return
        if self.debug_image_rate_hz > 0.0:
            if self.last_debug_image_publish_time is not None:
                elapsed = (stamp - self.last_debug_image_publish_time).nanoseconds / 1e9
                if elapsed < (1.0 / self.debug_image_rate_hz):
                    return
            self.last_debug_image_publish_time = stamp

        publish_frame = normalize_to_bgr8(frame)

        if 0.0 < self.debug_image_scale < 1.0:
            publish_frame = cv.resize(
                publish_frame,
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
