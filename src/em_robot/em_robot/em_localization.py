#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv
import math

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
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

from .submodules.aruco_detection import detect_aruco_corners


class MarkerLocalizationNode(Node):
    def __init__(self):
        super().__init__('marker_localization_node')

        # === Load calibration ===
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

        self.camera_height = config.get('camera_height', 0.381)

        # === Camera setup ===
        self.size = (1536, 864)
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": 1.0 / self.camera_height
        })

        # === TF and Odom setup ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_odom = None
        self.prev_odom = None

        self.estimated_x = 0.0
        self.estimated_y = 0.0
        self.estimated_theta = 0.0

        self.odom_subscription = self.create_subscription(
            Odometry,
            'odomWheel',
            self.odom_callback,
            10
        )

        self.timer_period = 1 / 5  # 5 Hz
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
        self.get_logger().info("Published static transform: camera_frame -> base_link")

    def odom_callback(self, msg):
        if self.prev_odom is not None:
            dx = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y

            def yaw_from_quat(q):
                siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                return math.atan2(siny_cosp, cosy_cosp)

            theta_prev = yaw_from_quat(self.prev_odom.pose.pose.orientation)
            theta_now = yaw_from_quat(msg.pose.pose.orientation)
            dtheta = theta_now - theta_prev

            # Rotate dx, dy into the map frame
            self.estimated_x += dx * math.cos(self.estimated_theta) - dy * math.sin(self.estimated_theta)
            self.estimated_y += dx * math.sin(self.estimated_theta) + dy * math.cos(self.estimated_theta)
            self.estimated_theta += dtheta

        self.prev_odom = msg

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if not detections:
            self.get_logger().info("No markers detected.")
            self.publish_fallback_transform()
            return

        for marker in detections:
            marker_id = marker['id']
            corners = marker['corners']

            undistorted_corners = cv.undistortPoints(
                np.expand_dims(corners, axis=1),
                self.camera_matrix,
                self.dist_coeffs,
                P=self.camera_matrix
            ).reshape(-1, 2)

            marker_center = np.mean(corners, axis=0)
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            pixel_offset = np.array([cx, cy]) - marker_center

            x_cam = self.camera_height * pixel_offset[0] / (fx * 2)
            y_cam = self.camera_height * pixel_offset[1] / (fy * 2)
            z_cam = 0.0
            cam_in_marker_pos = np.array([x_cam, y_cam, z_cam])

            top_center = np.mean(corners[0:2], axis=0)
            bottom_center = np.mean(corners[2:4], axis=0)
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

            # Update estimated pose from marker
            self.estimated_x = inv_trans[0]
            self.estimated_y = inv_trans[1]
            self.estimated_theta = yaw

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"aruco_{marker_id}"
            t.child_frame_id = "camera_frame"

            t.transform.translation.x = inv_trans[0]
            t.transform.translation.y = inv_trans[1]
            t.transform.translation.z = inv_trans[2]

            t.transform.rotation.x = inv_quat[0]
            t.transform.rotation.y = inv_quat[1]
            t.transform.rotation.z = inv_quat[2]
            t.transform.rotation.w = inv_quat[3]

            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(
                f"[TF] Marker detected: camera_frame ← aruco_{marker_id} | "
                f"x={inv_trans[0]:.2f}, y={inv_trans[1]:.2f}, yaw={np.degrees(yaw):.1f}°"
            )

    def publish_fallback_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.estimated_x
        t.transform.translation.y = self.estimated_y
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, self.estimated_theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"[TF-Fallback] Published from odom delta: x={self.estimated_x:.2f}, "
            f"y={self.estimated_y:.2f}, θ={math.degrees(self.estimated_theta):.1f}°"
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
