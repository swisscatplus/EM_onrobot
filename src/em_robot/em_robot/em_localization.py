#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import (
    quaternion_from_euler,
    quaternion_matrix,
    translation_matrix,
    concatenate_matrices,
    translation_from_matrix,
    quaternion_from_matrix
)
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration

from .submodules.aruco_detection import detect_aruco_corners


class MarkerLocalizationNode(Node):
    def __init__(self):
        super().__init__('marker_localization_node')

        package_name = 'em_robot'
        config_filename = 'config/calibration.yaml'
        pkg_share = get_package_share_directory(package_name)
        config_path = os.path.join(pkg_share, config_filename)
        self.get_logger().info(f"Loading configuration: {config_path}")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        cammat_list = config.get('camera_matrix', [])
        dist_list = config.get('dist_coeff', [])
        self.camera_matrix = np.array(cammat_list, dtype=np.float32)
        self.dist_coeffs = np.array(dist_list, dtype=np.float32)

        if self.camera_matrix.shape != (3, 3):
            raise ValueError("Camera matrix must be 3x3 in the YAML!")
        if self.dist_coeffs.shape[0] < 4:
            raise ValueError("dist_coeff must have at least 4 values (k1, k2, p1, p2)!")

        self.camera_height = config.get('camera_height', 0.44)

        self.size = (4608/3, 2592/3)
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()

        lens_position = 1.0 / self.camera_height
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position
        })

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.frequency = 5
        self.timer_period = 1/self.frequency
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "camera_frame"
        static_transform.child_frame_id = "base_link"

        static_transform.transform.translation.x = -0.192
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform: camera_frame -> base_link ")

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if not detections:
            self.get_logger().info("No markers detected.")
            return

        for marker in detections:
            marker_id = marker['id']
            corners = marker['corners']  # shape: (4, 2)

            # Undistort corners
            undistorted_corners = cv.undistortPoints(
                np.expand_dims(corners, axis=1),
                self.camera_matrix,
                self.dist_coeffs,
                P=self.camera_matrix
            ).reshape(-1, 2)

            marker_center = np.mean(undistorted_corners, axis=0)
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            pixel_offset = np.array([cx, cy]) - marker_center

            x_cam = self.camera_height * pixel_offset[0] / fx
            y_cam = self.camera_height * pixel_offset[1] / fy
            z_cam = 0.0

            cam_in_marker_pos = np.array([x_cam, y_cam, z_cam])

            top_center = np.mean(undistorted_corners[0:2], axis=0)
            bottom_center = np.mean(undistorted_corners[2:4], axis=0)
            dx = top_center[0] - bottom_center[0]
            dy = top_center[1] - bottom_center[1]
            yaw = np.arctan2(dx, -dy)

            quat = quaternion_from_euler(0.0, 0.0, yaw)

            T_marker_cam = concatenate_matrices(
                translation_matrix(cam_in_marker_pos),
                quaternion_matrix(quat)
            )

            T_cam_marker = np.linalg.inv(T_marker_cam)
            inv_trans = translation_from_matrix(T_cam_marker)
            inv_quat = quaternion_from_matrix(T_cam_marker)

            t_camera_marker = TransformStamped()
            t_camera_marker.header.stamp = self.get_clock().now().to_msg()
            t_camera_marker.header.frame_id = f"aruco_{marker_id}"
            t_camera_marker.child_frame_id = "camera_frame"

            t_camera_marker.transform.translation.x = inv_trans[0]
            t_camera_marker.transform.translation.y = inv_trans[1]
            t_camera_marker.transform.translation.z = inv_trans[2]

            t_camera_marker.transform.rotation.x = inv_quat[0]
            t_camera_marker.transform.rotation.y = inv_quat[1]
            t_camera_marker.transform.rotation.z = inv_quat[2]
            t_camera_marker.transform.rotation.w = inv_quat[3]

            self.tf_broadcaster.sendTransform(t_camera_marker)

            self.get_logger().info(
                f"[TF] Published: camera_frame ← aruco_{marker_id}: "
                f"x={inv_trans[0]:.2f}, y={inv_trans[1]:.2f}, z={inv_trans[2]:.2f}, yaw={np.degrees(yaw):.1f}°"
            )

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
