#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from collections import deque

import cv2 as cv
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
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
from em_robot.camera_sources import create_camera_source
from em_robot.marker_map_io import (
    load_raw_marker_map_file,
    merge_marker_entries,
    save_marker_map_config,
)
from em_robot.marker_map_loader import load_marker_map_config
from em_robot.transform_utils import build_planar_transform, build_transform
from em_robot_srv.srv import SetInitialPose


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class MarkerSurveyNode(Node):
    def __init__(self):
        super().__init__("marker_survey")

        self.declare_parameter("config_file", "")
        self.declare_parameter("marker_map_file", "")
        self.declare_parameter("camera_backend", "picamera2")
        self.declare_parameter("camera_source", "0")
        self.declare_parameter("camera_loop", False)
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("debug_image_topic", "/marker_survey/debug_image")
        self.declare_parameter("summary_topic", "/marker_survey/summary")
        self.declare_parameter("measured_pose_topic", "/marker_survey/measured_pose")
        self.declare_parameter("process_rate_hz", 10.0)
        self.declare_parameter("history_size", 30)
        self.declare_parameter("min_observations", 10)
        self.declare_parameter("max_position_std_m", 0.02)
        self.declare_parameter("max_yaw_std_deg", 3.0)
        self.declare_parameter("max_observation_age_sec", 2.0)
        self.declare_parameter("overwrite_existing_markers", True)
        self.declare_parameter("known_robot_x", 0.0)
        self.declare_parameter("known_robot_y", 0.0)
        self.declare_parameter("known_robot_yaw", 0.0)

        config_path = self.get_parameter("config_file").value or os.path.join(
            get_package_share_directory("em_robot"),
            "config",
            "calibration.yaml",
        )
        default_marker_map_path = os.path.join(
            get_package_share_directory("em_robot"),
            "config",
            "marker_map.yaml",
        )
        marker_map_path = self.get_parameter("marker_map_file").value or default_marker_map_path

        self.marker_map_file = str(marker_map_path)
        self.camera_backend = str(self.get_parameter("camera_backend").value)
        self.camera_source = str(self.get_parameter("camera_source").value)
        self.camera_loop = bool(self.get_parameter("camera_loop").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.summary_topic = str(self.get_parameter("summary_topic").value)
        self.measured_pose_topic = str(self.get_parameter("measured_pose_topic").value)
        self.process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self.history_size = max(1, int(self.get_parameter("history_size").value))
        self.min_observations = max(1, int(self.get_parameter("min_observations").value))
        self.max_position_std_m = float(self.get_parameter("max_position_std_m").value)
        self.max_yaw_std_deg = float(self.get_parameter("max_yaw_std_deg").value)
        self.max_observation_age_sec = float(
            self.get_parameter("max_observation_age_sec").value
        )
        self.overwrite_existing_markers = bool(
            self.get_parameter("overwrite_existing_markers").value
        )

        self.get_logger().info(f"Loading marker survey calibration: {config_path}")
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}

        self.camera_matrix = np.array(config.get("camera_matrix", []), dtype=np.float32)
        self.dist_coeffs = np.array(config.get("dist_coeff", []), dtype=np.float32)
        self.marker_size = float(config.get("marker_size", 0.038))
        self.image_size = tuple(config.get("image_size", [1536, 864]))
        self.lens_position = float(config.get("lens_position", 8.0))
        self.max_reprojection_error = float(config.get("max_reprojection_error_px", 8.0))

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

        self.marker_object_points = build_marker_object_points(self.marker_size)
        self.aruco_detector = create_aruco_detector()
        self.camera = create_camera_source(
            camera_backend=self.camera_backend,
            source=self.camera_source,
            image_size=self.image_size,
            lens_position=self.lens_position,
            loop=self.camera_loop,
        )

        self.raw_marker_map = load_raw_marker_map_file(self.marker_map_file)
        if self.raw_marker_map:
            loaded_marker_map = load_marker_map_config(self.marker_map_file)
            self.map_frame = str(loaded_marker_map["map_frame"])
            self.marker_prefix = str(loaded_marker_map["marker_prefix"])
        else:
            self.map_frame = "map"
            self.marker_prefix = "aruco_"

        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
        self.measured_pose_pub = self.create_publisher(PoseStamped, self.measured_pose_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.set_pose_srv = self.create_service(
            SetInitialPose,
            "set_known_robot_pose",
            self.handle_set_known_robot_pose,
        )
        self.save_visible_markers_srv = self.create_service(
            Trigger,
            "save_visible_markers",
            self.handle_save_visible_markers,
        )

        self.current_robot_pose = build_planar_transform(
            float(self.get_parameter("known_robot_x").value),
            float(self.get_parameter("known_robot_y").value),
            float(self.get_parameter("known_robot_yaw").value),
        )
        self.current_robot_pose_valid = True

        self.observations_by_marker_id: dict[int, deque] = {}
        self.last_detection_summary = "waiting for detections"
        self.last_camera_to_marker_by_id = {}

        timer_period = 1.0 / self.process_rate_hz if self.process_rate_hz > 0.0 else 0.1
        self.timer = self.create_timer(timer_period, self.process_frame)

        self.get_logger().info(
            f"Marker survey ready. Output map file: {self.marker_map_file}. "
            "Use /set_known_robot_pose then call /save_visible_markers when observations are stable."
        )
        if self.marker_map_file == default_marker_map_path:
            self.get_logger().warn(
                "marker_map_file was not set explicitly. Saving into the installed package "
                "config can be overwritten by a future rebuild; prefer passing an explicit "
                "workspace or external YAML path."
            )

    def handle_set_known_robot_pose(self, request, response):
        self.current_robot_pose = build_planar_transform(request.x, request.y, request.yaw)
        self.current_robot_pose_valid = True
        self.get_logger().info(
            f"Updated known robot pose: x={request.x:.3f}, y={request.y:.3f}, "
            f"yaw={math.degrees(request.yaw):.1f} deg"
        )
        response.success = True
        return response

    def _publish_summary(self, text: str):
        summary = String()
        summary.data = text
        self.summary_pub.publish(summary)
        self.last_detection_summary = text

    def _publish_debug_image(self, frame, stamp):
        msg = Image()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.camera_frame
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        self.debug_image_pub.publish(msg)

    def _publish_measured_pose(self, transform_matrix, stamp):
        translation = transform_matrix[:3, 3]
        quaternion = quaternion_from_matrix(transform_matrix)

        msg = PoseStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(translation[0])
        msg.pose.position.y = float(translation[1])
        msg.pose.position.z = float(translation[2])
        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])
        self.measured_pose_pub.publish(msg)

    def _append_observation(self, marker_id, map_to_marker, reprojection_error, stamp):
        history = self.observations_by_marker_id.setdefault(
            marker_id,
            deque(maxlen=self.history_size),
        )
        history.append(
            {
                "stamp_ns": stamp.nanoseconds,
                "translation": map_to_marker[:3, 3].copy(),
                "quaternion": quaternion_from_matrix(map_to_marker),
                "yaw": self._yaw_from_matrix(map_to_marker),
                "reprojection_error": float(reprojection_error),
            }
        )

    def _history_to_stable_estimate(self, marker_id):
        history = self.observations_by_marker_id.get(marker_id)
        if not history or len(history) < self.min_observations:
            return None

        newest_stamp_ns = history[-1]["stamp_ns"]
        cutoff_ns = newest_stamp_ns - int(self.max_observation_age_sec * 1e9)
        recent_samples = [sample for sample in history if sample["stamp_ns"] >= cutoff_ns]
        if len(recent_samples) < self.min_observations:
            return None

        translations = np.array([sample["translation"] for sample in recent_samples])
        translation_mean = np.mean(translations, axis=0)
        position_std = np.std(translations, axis=0)
        planar_position_std = float(np.linalg.norm(position_std[:2]))

        yaws = np.array([sample["yaw"] for sample in recent_samples], dtype=np.float64)
        yaw_mean = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
        yaw_errors = np.array([_wrap_angle(yaw - yaw_mean) for yaw in yaws], dtype=np.float64)
        yaw_std_deg = math.degrees(float(np.std(yaw_errors)))

        if planar_position_std > self.max_position_std_m:
            return None
        if yaw_std_deg > self.max_yaw_std_deg:
            return None

        quaternion_mean = self._average_quaternions(
            [sample["quaternion"] for sample in recent_samples]
        )
        reprojection_error_mean = float(
            np.mean([sample["reprojection_error"] for sample in recent_samples])
        )

        transform = concatenate_matrices(
            translation_matrix(translation_mean.tolist()),
            quaternion_matrix(quaternion_mean),
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion_mean)

        return {
            "id": int(marker_id),
            "transform": transform,
            "x": float(translation_mean[0]),
            "y": float(translation_mean[1]),
            "z": float(translation_mean[2]),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "samples": len(recent_samples),
            "planar_position_std_m": planar_position_std,
            "yaw_std_deg": yaw_std_deg,
            "reprojection_error_px": reprojection_error_mean,
        }

    def _average_quaternions(self, quaternions):
        reference = np.array(quaternions[0], dtype=np.float64)
        aligned = []
        for quaternion in quaternions:
            candidate = np.array(quaternion, dtype=np.float64)
            if np.dot(candidate, reference) < 0.0:
                candidate = -candidate
            aligned.append(candidate)

        mean_quaternion = np.mean(np.array(aligned), axis=0)
        norm = np.linalg.norm(mean_quaternion)
        if norm <= 1e-9:
            return reference
        return mean_quaternion / norm

    def _yaw_from_matrix(self, transform_matrix):
        quaternion = quaternion_from_matrix(transform_matrix)
        _, _, yaw = euler_from_quaternion(quaternion)
        return yaw

    def _build_stable_marker_transforms(self, stamp):
        transforms = []
        for marker_id in sorted(self.observations_by_marker_id):
            estimate = self._history_to_stable_estimate(marker_id)
            if estimate is None:
                continue

            transforms.append(
                build_transform(
                    self.map_frame,
                    f"{self.marker_prefix}survey_{marker_id}",
                    estimate["transform"],
                    stamp.to_msg(),
                )
            )
        return transforms

    def process_frame(self):
        capture_stamp = self.get_clock().now()

        try:
            frame = self.camera.capture()
        except Exception as exc:
            self.get_logger().warn(f"Marker survey camera capture failed: {exc}")
            return

        annotated_frame = frame.copy()
        detections = detect_aruco_markers(frame, detector=self.aruco_detector)
        map_to_camera = self.current_robot_pose @ self.t_base_camera

        stable_marker_ids = []
        visible_marker_ids = []

        for detection in detections:
            marker_id = detection["id"]
            visible_marker_ids.append(marker_id)
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
                cv.putText(
                    annotated_frame,
                    f"id={marker_id} rejected",
                    tuple(polyline[0, 0]),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    1,
                    cv.LINE_AA,
                )
                continue

            self.last_camera_to_marker_by_id[marker_id] = camera_to_marker

            map_to_marker = map_to_camera @ camera_to_marker
            self._append_observation(marker_id, map_to_marker, reprojection_error, capture_stamp)

            estimate = self._history_to_stable_estimate(marker_id)
            if estimate is not None:
                stable_marker_ids.append(marker_id)

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

        measured_pose_transform = build_transform(
            self.map_frame,
            f"{self.base_frame}_survey_pose",
            self.current_robot_pose,
            capture_stamp.to_msg(),
        )
        stable_transforms = self._build_stable_marker_transforms(capture_stamp)
        self.tf_broadcaster.sendTransform([measured_pose_transform] + stable_transforms)
        self._publish_measured_pose(self.current_robot_pose, capture_stamp)

        summary = (
            f"visible={visible_marker_ids} stable={stable_marker_ids} "
            f"known_pose_valid={self.current_robot_pose_valid}"
        )
        self._publish_summary(summary)

        cv.putText(
            annotated_frame,
            f"known pose: x={self.current_robot_pose[0, 3]:.3f}, "
            f"y={self.current_robot_pose[1, 3]:.3f}",
            (10, 30),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
            cv.LINE_AA,
        )
        cv.putText(
            annotated_frame,
            f"visible={visible_marker_ids} stable={stable_marker_ids}",
            (10, 60),
            cv.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv.LINE_AA,
        )
        self._publish_debug_image(annotated_frame, capture_stamp)

    def handle_save_visible_markers(self, request, response):
        del request

        stable_markers = []
        for marker_id in sorted(self.observations_by_marker_id):
            estimate = self._history_to_stable_estimate(marker_id)
            if estimate is not None:
                stable_markers.append(estimate)

        if not stable_markers:
            response.success = False
            response.message = (
                "No stable marker observations available yet. "
                "Move the robot to a known pose, wait for detections to stabilize, then try again."
            )
            return response

        if self.raw_marker_map:
            existing_markers = load_marker_map_config(self.marker_map_file)["markers"]
        else:
            existing_markers = []
        stable_entries = [
            {
                "id": marker["id"],
                "x": marker["x"],
                "y": marker["y"],
                "z": marker["z"],
                "roll": marker["roll"],
                "pitch": marker["pitch"],
                "yaw": marker["yaw"],
            }
            for marker in stable_markers
        ]

        if self.overwrite_existing_markers:
            merged_markers = merge_marker_entries(existing_markers, stable_entries)
        else:
            existing_ids = {int(marker["id"]) for marker in existing_markers}
            markers_to_add = [
                marker for marker in stable_entries if int(marker["id"]) not in existing_ids
            ]
            merged_markers = merge_marker_entries(existing_markers, markers_to_add)

        save_marker_map_config(
            self.marker_map_file,
            merged_markers,
            map_frame=self.map_frame,
            marker_prefix=self.marker_prefix,
        )
        self.raw_marker_map = load_raw_marker_map_file(self.marker_map_file)

        marker_summaries = ", ".join(
            (
                f"id={marker['id']} n={marker['samples']} "
                f"pos_std={marker['planar_position_std_m']:.3f}m "
                f"yaw_std={marker['yaw_std_deg']:.2f}deg"
            )
            for marker in stable_markers
        )
        response.success = True
        response.message = (
            f"Saved {len(stable_markers)} stable marker(s) to {self.marker_map_file}: "
            f"{marker_summaries}"
        )
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        if hasattr(self, "camera") and self.camera is not None:
            self.camera.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerSurveyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down marker survey node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
