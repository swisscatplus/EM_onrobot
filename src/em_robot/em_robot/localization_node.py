#!/usr/bin/env python3
import math
import os

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from em_robot.aruco_utils import (
    build_marker_object_points,
    create_aruco_detector,
    detect_aruco_markers,
    estimate_marker_pose,
)
from em_robot.camera_sources import create_camera_source
from em_robot.transform_utils import (
    build_planar_transform,
    build_transform,
    compute_map_to_base_from_marker,
    compute_map_to_odom_from_map_to_base,
    transform_to_matrix,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
from tf_transformations import (
    concatenate_matrices,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
    inverse_matrix,
)


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization")
        self.get_logger().info("Starting localization node...")

        self.declare_parameter("config_file", "")
        self.declare_parameter("camera_backend", "picamera2")
        self.declare_parameter("camera_source", "0")
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

        self.get_logger().info(f"Loading configuration: {config_path}")
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        self.camera_matrix = np.array(config.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get("dist_coeff", []), dtype=np.float32)
        self.marker_size = float(config.get("marker_size", 0.038))
        self.image_size = tuple(config.get("image_size", [1536, 864]))
        self.lens_position = float(config.get("lens_position", 8.0))
        self.max_reprojection_error = float(config.get("max_reprojection_error_px", 8.0))

        self.marker_object_points = build_marker_object_points(self.marker_size)
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

        self.odom_frame_available = False
        self.last_map_to_odom = None

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
        self.publish_initial_map_to_odom()

        process_timer_period = 1.0 / self.process_rate_hz if self.process_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(process_timer_period, self.process_frame)

        map_odom_timer_period = (
            1.0 / self.map_odom_publish_rate_hz
            if self.map_odom_publish_rate_hz > 0.0
            else 0.05
        )
        self.map_odom_timer = self.create_timer(
            map_odom_timer_period,
            self.broadcast_last_map_to_odom,
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

    def publish_initial_map_to_odom(self):
        map_to_odom = build_planar_transform(0.0, 0.0, 0.0)

        transform = build_transform(
            self.map_frame,
            self.odom_frame,
            map_to_odom,
            self.get_clock().now().to_msg(),
        )

        self.last_map_to_odom = transform
        self.tf_broadcaster.sendTransform(transform)

        self.get_logger().info(
            f"Published initial transform: {self.map_frame} -> {self.odom_frame}"
        )

    def broadcast_last_map_to_odom(self):
        if self.last_map_to_odom is None:
            return

        self.last_map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.last_map_to_odom)

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

    def process_frame(self):
        from tf_transformations import euler_from_quaternion, translation_from_matrix

        capture_stamp = self.get_clock().now()

        if not self.odom_frame_available:
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

        try:
            odom_to_base = self.lookup_matrix(
                self.odom_frame,
                self.base_frame,
                Time(),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Cannot get {self.odom_frame} -> {self.base_frame}: {exc}"
            )
            self.odom_frame_available = False
            return

        detection = detections[0]
        marker_id = detection["id"]
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
            self.get_logger().warn(
                f"Marker {marker_id} reprojection error too high: "
                f"{reprojection_error:.2f}px"
            )

            cv.putText(
                annotated_frame,
                f"Marker {marker_id}: pose estimation failed",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )

            self.publish_debug_image(annotated_frame, capture_stamp)
            return

        cv.putText(
            annotated_frame,
            f"Marker {marker_id}: reproj_err={reprojection_error:.2f}px",
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
                f"aruco_{marker_id}",
                Time(),
                timeout_sec=0.5,
            )
        except Exception as exc:
            self.get_logger().warn(f"Cannot find marker {marker_id} in map: {exc}")

            cv.putText(
                annotated_frame,
                f"Marker {marker_id} not found in map frame",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )

            self.publish_debug_image(annotated_frame, capture_stamp)
            return

        map_to_base = compute_map_to_base_from_marker(
            map_to_marker,
            camera_to_marker,
            self.t_camera_base,
        )

        translation = translation_from_matrix(map_to_base)
        quaternion = quaternion_from_matrix(map_to_base)
        _, _, yaw = euler_from_quaternion(quaternion)

        planar_map_to_base = build_planar_transform(
            translation[0],
            translation[1],
            yaw,
        )

        odom_translation = translation_from_matrix(odom_to_base)
        odom_quaternion = quaternion_from_matrix(odom_to_base)
        _, _, odom_yaw = euler_from_quaternion(odom_quaternion)

        planar_odom_to_base = build_planar_transform(
            odom_translation[0],
            odom_translation[1],
            odom_yaw,
        )

        map_to_odom = compute_map_to_odom_from_map_to_base(
            planar_map_to_base,
            planar_odom_to_base,
        )

        t_map_to_odom = build_transform(
            self.map_frame,
            self.odom_frame,
            map_to_odom,
            self.get_clock().now().to_msg(),
        )

        self.last_map_to_odom = t_map_to_odom
        self.tf_broadcaster.sendTransform(t_map_to_odom)

        if self.vision_base_pose_pub is not None:
            self.vision_base_pose_pub.publish(
                self.build_pose_stamped(
                    self.map_frame,
                    planar_map_to_base,
                    self.get_clock().now().to_msg(),
                )
            )

        self.get_logger().info(
            f"Localization update from marker {marker_id}: "
            f"x={translation[0]:.3f}, "
            f"y={translation[1]:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg"
        )

        cv.putText(
            annotated_frame,
            f"x={translation[0]:.3f}, y={translation[1]:.3f}, "
            f"yaw={math.degrees(yaw):.1f} deg",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
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

            try:
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

            except Exception:
                raise exc

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