#!/usr/bin/env python3
from __future__ import annotations

import os

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from tf_transformations import (
    concatenate_matrices,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
)

from em_robot.aruco_utils import (
    build_marker_object_points,
    create_aruco_detector,
    detect_aruco_markers,
    estimate_marker_pose,
)
from em_robot.camera_sources import create_camera_source, normalize_to_bgr8
from em_robot.diagnostic_utils import build_diagnostic_array, build_diagnostic_status
from em_robot.marker_map_loader import load_marker_map_config
from em_robot.transform_utils import build_transform, yaw_from_matrix


class ArucoCameraTestNode(Node):
    def __init__(self):
        super().__init__("aruco_camera_test")

        self.declare_parameter("config_file", "")
        self.declare_parameter("marker_map_file", "")
        self.declare_parameter("camera_backend", "opencv")
        self.declare_parameter(
            "camera_source",
            "0",
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("camera_loop", False)
        self.declare_parameter("diagnostics_rate_hz", 1.0)
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("map_camera_frame", "camera_test")
        self.declare_parameter("debug_image_topic", "/aruco_test/debug_image")
        self.declare_parameter("marker_pose_topic", "/aruco_test/marker_poses")
        self.declare_parameter("camera_pose_topic", "/aruco_test/camera_pose")
        self.declare_parameter("summary_topic", "/aruco_test/summary")

        config_path = self.get_parameter("config_file").value or os.path.join(
            get_package_share_directory("em_robot"),
            "config",
            "calibration_ubuntu_test.yaml",
        )
        marker_map_file = str(self.get_parameter("marker_map_file").value)
        self.camera_backend = str(self.get_parameter("camera_backend").value)
        self.camera_source = str(self.get_parameter("camera_source").value)
        self.camera_loop = bool(self.get_parameter("camera_loop").value)
        self.diagnostics_rate_hz = float(self.get_parameter("diagnostics_rate_hz").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.map_camera_frame = str(self.get_parameter("map_camera_frame").value)

        debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        marker_pose_topic = str(self.get_parameter("marker_pose_topic").value)
        camera_pose_topic = str(self.get_parameter("camera_pose_topic").value)
        summary_topic = str(self.get_parameter("summary_topic").value)

        self.get_logger().info(f"Loading ArUco camera test config: {config_path}")
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)

        self.camera_matrix = np.array(config.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get("dist_coeff", []), dtype=np.float32)
        self.marker_size = float(config.get("marker_size", 0.038))
        self.image_size = tuple(config.get("image_size", [1280, 720]))
        self.process_rate_hz = float(config.get("process_rate_hz", 5.0))
        self.lens_position = float(config.get("lens_position", 0.0))
        self.max_reprojection_error = float(config.get("max_reprojection_error_px", 8.0))

        self.marker_object_points = build_marker_object_points(self.marker_size)
        self.aruco_detector = create_aruco_detector()
        self.camera = create_camera_source(
            camera_backend=self.camera_backend,
            source=self.camera_source,
            image_size=self.image_size,
            lens_position=self.lens_position,
            loop=self.camera_loop,
        )

        self.marker_map = {"map_frame": "map", "markers": []}
        self.marker_transforms = {}
        if marker_map_file:
            self.marker_map = load_marker_map_config(marker_map_file)
            for marker in self.marker_map["markers"]:
                self.marker_transforms[marker["id"]] = concatenate_matrices(
                    translation_matrix([marker["x"], marker["y"], marker["z"]]),
                    quaternion_matrix(
                        quaternion_from_euler(
                            marker["roll"],
                            marker["pitch"],
                            marker["yaw"],
                        )
                    ),
                )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.debug_image_pub = self.create_publisher(Image, debug_image_topic, 10)
        self.marker_pose_pub = self.create_publisher(PoseArray, marker_pose_topic, 10)
        self.camera_pose_pub = self.create_publisher(PoseStamped, camera_pose_topic, 10)
        self.summary_pub = self.create_publisher(String, summary_topic, 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        self.last_frame_time = None
        self.last_detection_time = None
        self.last_camera_error = ""
        self.last_detection_count = 0
        self.last_detected_ids = []
        self.last_pose_marker_id = None
        self.has_map_pose = False
        self.last_camera_to_marker_by_id = {}

        timer_period = 1.0 / self.process_rate_hz if self.process_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.process_frame)
        diagnostics_period = (
            1.0 / self.diagnostics_rate_hz if self.diagnostics_rate_hz > 0.0 else 1.0
        )
        self.diagnostics_timer = self.create_timer(diagnostics_period, self.publish_diagnostics)

    def _pose_from_matrix(self, transform_matrix):
        translation = transform_matrix[:3, 3]
        quaternion = quaternion_from_matrix(transform_matrix)

        pose = Pose()
        pose.position.x = float(translation[0])
        pose.position.y = float(translation[1])
        pose.position.z = float(translation[2])
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        return pose

    def _publish_summary(self, summary_text: str):
        msg = String()
        msg.data = summary_text
        self.summary_pub.publish(msg)

    def _publish_debug_image(self, frame, stamp):
        publish_frame = normalize_to_bgr8(frame)
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

    def process_frame(self):
        capture_stamp = self.get_clock().now()
        try:
            frame = self.camera.capture()
        except Exception as exc:
            self.last_camera_error = str(exc)
            self.get_logger().warn(f"ArUco test camera capture failed: {exc}")
            return

        self.last_frame_time = capture_stamp
        self.last_camera_error = ""
        detections = detect_aruco_markers(frame, detector=self.aruco_detector)
        self.last_detection_count = len(detections)
        self.last_detected_ids = [detection["id"] for detection in detections]

        annotated_frame = frame.copy()
        pose_array = PoseArray()
        pose_array.header.stamp = capture_stamp.to_msg()
        pose_array.header.frame_id = self.camera_frame

        candidate_map_poses = []
        marker_transforms = []

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
                reference_transform=self.last_camera_to_marker_by_id.get(detection["id"]),
            )
            if camera_to_marker is None:
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

            self.last_camera_to_marker_by_id[detection["id"]] = camera_to_marker

            pose_array.poses.append(self._pose_from_matrix(camera_to_marker))
            marker_transforms.append(
                build_transform(
                    self.camera_frame,
                    f"aruco_detected_{detection['id']}",
                    camera_to_marker,
                    capture_stamp.to_msg(),
                )
            )

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

            if detection["id"] not in self.marker_transforms:
                continue

            map_to_marker = self.marker_transforms[detection["id"]]
            map_to_camera = map_to_marker @ inverse_matrix(camera_to_marker)
            candidate_map_poses.append(
                {
                    "marker_id": detection["id"],
                    "map_to_camera": map_to_camera,
                    "reprojection_error": reprojection_error,
                }
            )

        self.marker_pose_pub.publish(pose_array)
        if marker_transforms:
            self.tf_broadcaster.sendTransform(marker_transforms)

        summary_text = f"detected_ids={self.last_detected_ids}"
        self.has_map_pose = False
        self.last_pose_marker_id = None

        if candidate_map_poses:
            best_candidate = min(
                candidate_map_poses,
                key=lambda candidate: candidate["reprojection_error"],
            )
            best_map_to_camera = best_candidate["map_to_camera"]
            camera_pose = PoseStamped()
            camera_pose.header.stamp = capture_stamp.to_msg()
            camera_pose.header.frame_id = self.marker_map["map_frame"]
            camera_pose.pose = self._pose_from_matrix(best_map_to_camera)
            self.camera_pose_pub.publish(camera_pose)
            self.tf_broadcaster.sendTransform(
                build_transform(
                    self.marker_map["map_frame"],
                    self.map_camera_frame,
                    best_map_to_camera,
                    capture_stamp.to_msg(),
                )
            )

            self.last_detection_time = capture_stamp
            self.has_map_pose = True
            self.last_pose_marker_id = int(best_candidate["marker_id"])
            summary_text = (
                f"detected_ids={self.last_detected_ids}, "
                f"best_marker={self.last_pose_marker_id}, "
                f"x={best_map_to_camera[0, 3]:.3f}, "
                f"y={best_map_to_camera[1, 3]:.3f}, "
                f"yaw={yaw_from_matrix(best_map_to_camera):.3f}"
            )

        if self.has_map_pose:
            cv.putText(
                annotated_frame,
                summary_text,
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
                cv.LINE_AA,
            )
        elif detections:
            cv.putText(
                annotated_frame,
                "Markers detected, but no map pose available",
                (10, 30),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
                cv.LINE_AA,
            )

        self._publish_summary(summary_text)
        self._publish_debug_image(annotated_frame, capture_stamp)

    def publish_diagnostics(self):
        now = self.get_clock().now()
        frame_age = None
        detection_age = None
        if self.last_frame_time is not None:
            frame_age = (now - self.last_frame_time).nanoseconds / 1e9
        if self.last_detection_time is not None:
            detection_age = (now - self.last_detection_time).nanoseconds / 1e9

        frame_timeout = max(1.0, 3.0 / self.process_rate_hz) if self.process_rate_hz > 0.0 else 1.0
        level = DiagnosticStatus.OK
        message = "ArUco camera test healthy"
        if self.last_camera_error:
            level = DiagnosticStatus.ERROR
            message = "Camera capture is failing"
        elif frame_age is None or frame_age > frame_timeout:
            level = DiagnosticStatus.WARN
            message = "Camera frames are stale"
        elif self.last_detection_count == 0:
            level = DiagnosticStatus.WARN
            message = "No ArUco markers detected"
        elif not self.has_map_pose:
            level = DiagnosticStatus.WARN
            message = "Markers detected, but no map pose available"

        status = build_diagnostic_status(
            "em_robot/aruco_camera_test",
            level,
            message,
            values={
                "camera_backend": self.camera_backend,
                "camera_source": self.camera_source,
                "camera_loop": self.camera_loop,
                "marker_size_m": self.marker_size,
                "marker_map_count": len(self.marker_transforms),
                "last_frame_age_s": "n/a" if frame_age is None else f"{frame_age:.3f}",
                "last_detection_age_s": "n/a" if detection_age is None else f"{detection_age:.3f}",
                "last_detection_count": self.last_detection_count,
                "detected_ids": ",".join(str(marker_id) for marker_id in self.last_detected_ids)
                or "none",
                "has_map_pose": self.has_map_pose,
                "best_marker_id": "none"
                if self.last_pose_marker_id is None
                else str(self.last_pose_marker_id),
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
    node = ArucoCameraTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArUco camera test node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
