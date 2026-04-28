#!/usr/bin/env python3
import math
import os

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
from em_robot.camera_sources import create_camera_source
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
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("vision_base_pose_topic", "/localization/vision_base_pose")
        self.declare_parameter("process_rate_hz", 5.0)
        self.declare_parameter("map_odom_publish_rate_hz", 20.0)
        self.declare_parameter("publish_map_to_base", False)

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
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.vision_base_pose_topic = str(self.get_parameter("vision_base_pose_topic").value)
        self.process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self.map_odom_publish_rate_hz = float(
            self.get_parameter("map_odom_publish_rate_hz").value
        )
        self.publish_map_to_base = bool(self.get_parameter("publish_map_to_base").value)

        self.get_logger().info(f"Loading configuration: {config_path}")
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        self.camera_matrix = np.array(config.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get("dist_coeff", []), dtype=np.float32)
        self.marker_size = float(config.get("marker_size", 0.038))
        self.image_size = tuple(config.get("image_size", [1536, 864]))
        self.lens_position = float(config.get("lens_position", 8.0))
        self.max_reprojection_error = float(config.get("max_reprojection_error_px", 8.0))
        self.max_position_jump = float(config.get("max_position_jump", 0.25))
        self.max_yaw_jump = math.radians(float(config.get("max_yaw_jump_deg", 20.0)))
        self.position_smoothing_alpha = float(config.get("position_smoothing_alpha", 0.2))
        self.yaw_smoothing_alpha = float(config.get("yaw_smoothing_alpha", 0.2))
        self.min_update_translation = float(config.get("min_update_translation", 0.01))
        self.min_update_yaw = math.radians(float(config.get("min_update_yaw_deg", 1.0)))
        self.stationary_linear_speed_threshold = float(
            config.get("stationary_linear_speed_threshold", 0.01)
        )
        self.stationary_angular_speed_threshold = math.radians(
            float(config.get("stationary_angular_speed_threshold_deg", 1.0))
        )
        self.stationary_position_smoothing_alpha = float(
            config.get("stationary_position_smoothing_alpha", 0.05)
        )
        self.stationary_yaw_smoothing_alpha = float(
            config.get("stationary_yaw_smoothing_alpha", 0.05)
        )
        self.stationary_min_update_translation = float(
            config.get("stationary_min_update_translation", 0.03)
        )
        self.stationary_min_update_yaw = math.radians(
            float(config.get("stationary_min_update_yaw_deg", 2.0))
        )
        self.stationary_large_correction_translation = float(
            config.get("stationary_large_correction_translation", 0.10)
        )
        self.stationary_large_correction_yaw = math.radians(
            float(config.get("stationary_large_correction_yaw_deg", 8.0))
        )

        self.marker_object_points = build_marker_object_points(self.marker_size)
        self.aruco_detector = create_aruco_detector()

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
            f"Camera backend '{self.camera_backend}' initialized with source '{self.camera_source}'"
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()

        # Important: spin_thread=True lets the TF listener keep receiving /tf and /tf_static
        # while this node is inside timer callbacks.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.odom_frame_available = False
        self.last_localization_transform = None
        self.last_map_to_odom = np.identity(4, dtype=np.float64)
        self.has_map_lock = False
        self.last_odom_motion_matrix = None
        self.last_odom_motion_time = None
        self.last_camera_to_marker_by_id = {}

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

    def publish_initial_localization_transform(self):
        child_frame = self.base_frame if self.publish_map_to_base else self.odom_frame
        initial_transform = build_transform(
            self.map_frame,
            child_frame,
            build_planar_transform(0.0, 0.0, 0.0),
            self.get_clock().now().to_msg(),
        )

        self.last_localization_transform = initial_transform
        self.tf_broadcaster.sendTransform(initial_transform)

        self.get_logger().info(
            f"Published initial transform: {self.map_frame} -> {child_frame}"
        )

    def broadcast_last_localization_transform(self):
        if self.last_localization_transform is None:
            return

        self.last_localization_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.last_localization_transform)

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

    def update_odom_motion_state(self, odom_to_base, stamp):
        is_stationary = False

        if self.last_odom_motion_matrix is not None and self.last_odom_motion_time is not None:
            dt = (stamp - self.last_odom_motion_time).nanoseconds / 1e9
            if dt > 0.0:
                dx = float(odom_to_base[0, 3] - self.last_odom_motion_matrix[0, 3])
                dy = float(odom_to_base[1, 3] - self.last_odom_motion_matrix[1, 3])
                linear_speed = math.hypot(dx, dy) / dt
                yaw_speed = (
                    abs(
                        wrap_angle(
                            yaw_from_matrix(odom_to_base)
                            - yaw_from_matrix(self.last_odom_motion_matrix)
                        )
                    )
                    / dt
                )
                is_stationary = (
                    linear_speed <= self.stationary_linear_speed_threshold
                    and yaw_speed <= self.stationary_angular_speed_threshold
                )

        self.last_odom_motion_matrix = odom_to_base.copy()
        self.last_odom_motion_time = stamp
        return is_stationary

    def process_frame(self):
        from tf_transformations import translation_from_matrix

        capture_stamp = self.get_clock().now()

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
            frame = self.camera.capture()
        except Exception as exc:
            self.get_logger().warn(f"Camera capture failed: {exc}")
            return

        annotated_frame = frame.copy()

        detections = detect_aruco_markers(frame, detector=self.aruco_detector)

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

        if not self.publish_map_to_base:
            try:
                odom_to_base = self.lookup_matrix(
                    self.odom_frame,
                    self.base_frame,
                    capture_stamp,
                    timeout_sec=0.2,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Cannot get {self.odom_frame} -> {self.base_frame}: {exc}"
                )
                self.odom_frame_available = False
                return
            odom_stationary = self.update_odom_motion_state(odom_to_base, capture_stamp)
        else:
            odom_stationary = False

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
                self.camera_matrix,
                self.dist_coeffs,
                self.max_reprojection_error,
                reference_transform=self.last_camera_to_marker_by_id.get(marker_id),
            )

            if camera_to_marker is None:
                rejected_for_reprojection += 1
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
                    Time(),
                    timeout_sec=1.0,
                    allow_latest_fallback=False,
                )
            except Exception:
                missing_marker_tf += 1
                continue

            # map -> base_link from vision:
            # map -> marker @ marker -> camera @ camera -> base
            map_to_base = compute_map_to_base_from_marker(
                map_to_marker,
                camera_to_marker,
                self.t_camera_base,
            )
            measured_yaw = yaw_from_matrix(map_to_base)
            planar_map_to_base = build_planar_transform(
                map_to_base[0, 3],
                map_to_base[1, 3],
                measured_yaw,
            )

            candidates.append(
                {
                    "marker_id": marker_id,
                    "map_to_base": planar_map_to_base,
                    "reprojection_error": reprojection_error,
                }
            )

        if not candidates:
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
            cv.putText(
                annotated_frame,
                (
                    f"gate={rejected_for_gating} "
                    f"reproj={rejected_for_reprojection} "
                    f"missing_tf={missing_marker_tf}"
                ),
                (10, 60),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv.LINE_AA,
            )
            self.publish_debug_image(annotated_frame, capture_stamp)
            return

        weights = np.array(
            [1.0 / max(candidate["reprojection_error"], 1e-3) for candidate in candidates],
            dtype=np.float64,
        )
        weights /= np.sum(weights)

        measured_x = float(
            np.sum(
                [
                    weight * candidate["map_to_base"][0, 3]
                    for weight, candidate in zip(weights, candidates)
                ]
            )
        )
        measured_y = float(
            np.sum(
                [
                    weight * candidate["map_to_base"][1, 3]
                    for weight, candidate in zip(weights, candidates)
                ]
            )
        )
        measured_yaw = math.atan2(
            float(
                np.sum(
                    [
                        weight * math.sin(yaw_from_matrix(candidate["map_to_base"]))
                        for weight, candidate in zip(weights, candidates)
                    ]
                )
            ),
            float(
                np.sum(
                    [
                        weight * math.cos(yaw_from_matrix(candidate["map_to_base"]))
                        for weight, candidate in zip(weights, candidates)
                    ]
                )
            ),
        )

        measured_map_to_base = build_planar_transform(
            measured_x,
            measured_y,
            measured_yaw,
        )

        marker_ids = ",".join(str(candidate["marker_id"]) for candidate in candidates)

        if self.publish_map_to_base:
            if self.has_map_lock:
                target_delta = math.hypot(
                    measured_map_to_base[0, 3] - self.last_map_to_base[0, 3],
                    measured_map_to_base[1, 3] - self.last_map_to_base[1, 3],
                )
                target_yaw_delta = abs(
                    wrap_angle(measured_yaw - yaw_from_matrix(self.last_map_to_base))
                )
                if (
                    target_delta > self.max_position_jump
                    or target_yaw_delta > self.max_yaw_jump
                ):
                    self.publish_debug_image(annotated_frame, capture_stamp)
                    return

                fused_x = (
                    (1.0 - self.position_smoothing_alpha) * self.last_map_to_base[0, 3]
                    + self.position_smoothing_alpha * measured_x
                )
                fused_y = (
                    (1.0 - self.position_smoothing_alpha) * self.last_map_to_base[1, 3]
                    + self.position_smoothing_alpha * measured_y
                )
                fused_yaw = blend_angles(
                    yaw_from_matrix(self.last_map_to_base),
                    measured_yaw,
                    self.yaw_smoothing_alpha,
                )
                planar_map_to_base = build_planar_transform(fused_x, fused_y, fused_yaw)
            else:
                planar_map_to_base = measured_map_to_base
                fused_x = measured_x
                fused_y = measured_y
                fused_yaw = measured_yaw

            self.last_map_to_base = planar_map_to_base
            localization_transform = build_transform(
                self.map_frame,
                self.base_frame,
                planar_map_to_base,
                self.get_clock().now().to_msg(),
            )
            map_odom_translation = None
            map_odom_yaw = None
            self.get_logger().info(
                f"Camera-only localization update from markers [{marker_ids}]: "
                f"map->{self.base_frame} x={fused_x:.3f}, "
                f"y={fused_y:.3f}, "
                f"yaw={math.degrees(fused_yaw):.1f} deg"
            )
        else:
            target_map_to_odom = compute_map_to_odom_from_map_to_base(
                measured_map_to_base,
                odom_to_base,
            )
            target_translation = translation_from_matrix(target_map_to_odom)
            target_yaw = yaw_from_matrix(target_map_to_odom)

            if self.has_map_lock:
                correction_delta = math.hypot(
                    target_translation[0] - self.last_map_to_odom[0, 3],
                    target_translation[1] - self.last_map_to_odom[1, 3],
                )
                correction_yaw_delta = abs(
                    wrap_angle(target_yaw - yaw_from_matrix(self.last_map_to_odom))
                )
                if (
                    correction_delta > self.max_position_jump
                    or correction_yaw_delta > self.max_yaw_jump
                ):
                    rejected_for_gating = len(candidates)
                    self.publish_debug_image(annotated_frame, capture_stamp)
                    return

                large_stationary_correction = odom_stationary and (
                    correction_delta >= self.stationary_large_correction_translation
                    or correction_yaw_delta >= self.stationary_large_correction_yaw
                )
                if odom_stationary and not large_stationary_correction:
                    position_alpha = self.stationary_position_smoothing_alpha
                    yaw_alpha = self.stationary_yaw_smoothing_alpha
                    min_update_translation = self.stationary_min_update_translation
                    min_update_yaw = self.stationary_min_update_yaw
                else:
                    position_alpha = self.position_smoothing_alpha
                    yaw_alpha = self.yaw_smoothing_alpha
                    min_update_translation = self.min_update_translation
                    min_update_yaw = self.min_update_yaw

                last_translation = translation_from_matrix(self.last_map_to_odom)
                fused_odom_x = (
                    (1.0 - position_alpha) * last_translation[0]
                    + position_alpha * target_translation[0]
                )
                fused_odom_y = (
                    (1.0 - position_alpha) * last_translation[1]
                    + position_alpha * target_translation[1]
                )
                fused_odom_yaw = blend_angles(
                    yaw_from_matrix(self.last_map_to_odom),
                    target_yaw,
                    yaw_alpha,
                )

                correction_step = math.hypot(
                    fused_odom_x - last_translation[0],
                    fused_odom_y - last_translation[1],
                )
                correction_yaw_step = abs(
                    wrap_angle(
                        fused_odom_yaw - yaw_from_matrix(self.last_map_to_odom)
                    )
                )
                if (
                    correction_step < min_update_translation
                    and correction_yaw_step < min_update_yaw
                ):
                    self.publish_debug_image(annotated_frame, capture_stamp)
                    return

                map_to_odom = build_planar_transform(
                    fused_odom_x,
                    fused_odom_y,
                    fused_odom_yaw,
                )
            else:
                map_to_odom = target_map_to_odom
                position_alpha = 1.0
                yaw_alpha = 1.0

            self.last_map_to_odom = map_to_odom
            planar_map_to_base = map_to_odom @ odom_to_base
            fused_x = planar_map_to_base[0, 3]
            fused_y = planar_map_to_base[1, 3]
            fused_yaw = yaw_from_matrix(planar_map_to_base)

            map_odom_translation = translation_from_matrix(map_to_odom)
            map_odom_yaw = yaw_from_matrix(map_to_odom)

            self.get_logger().info(
                f"Localization update from markers [{marker_ids}]: "
                f"map->base x={fused_x:.3f}, "
                f"y={fused_y:.3f}, "
                f"yaw={math.degrees(fused_yaw):.1f} deg | "
                f"map->odom x={map_odom_translation[0]:.3f}, "
                f"y={map_odom_translation[1]:.3f}, "
                f"yaw={math.degrees(map_odom_yaw):.1f} deg | "
                f"accepted={len(candidates)}/{len(detections)} "
                f"gate={rejected_for_gating} reproj={rejected_for_reprojection} "
                f"missing_tf={missing_marker_tf} "
                f"stationary={odom_stationary} "
                f"alpha={position_alpha:.2f}/{yaw_alpha:.2f}"
            )

            localization_transform = build_transform(
                self.map_frame,
                self.odom_frame,
                map_to_odom,
                self.get_clock().now().to_msg(),
            )

        self.has_map_lock = True
        self.last_localization_transform = localization_transform
        self.tf_broadcaster.sendTransform(localization_transform)

        if self.vision_base_pose_pub is not None:
            self.vision_base_pose_pub.publish(
                self.build_pose_stamped(
                    self.map_frame,
                    planar_map_to_base,
                    self.get_clock().now().to_msg(),
                )
            )

        cv.putText(
            annotated_frame,
            f"markers [{marker_ids}] map->base "
            f"x={fused_x:.3f}, y={fused_y:.3f}, "
            f"yaw={math.degrees(fused_yaw):.1f} deg",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )

        cv.putText(
            annotated_frame,
            (
                f"map->{self.base_frame} x={fused_x:.3f}, "
                f"y={fused_y:.3f}, "
                f"yaw={math.degrees(fused_yaw):.1f} deg"
                if self.publish_map_to_base
                else (
                    f"map->odom x={map_odom_translation[0]:.3f}, "
                    f"y={map_odom_translation[1]:.3f}, "
                    f"yaw={math.degrees(map_odom_yaw):.1f} deg"
                )
            ),
            (10, 60),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
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
        allow_latest_fallback=True,
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

            self.get_logger().debug(
                f"Using latest {target_frame} -> {source_frame} transform"
            )

            return transform_to_matrix(latest_transform)

    def publish_debug_image(self, frame, stamp):
        if self.debug_image_pub is None:
            return

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
