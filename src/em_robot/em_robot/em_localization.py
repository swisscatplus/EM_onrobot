#!/usr/bin/env python3

import os
import yaml
import math
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
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

        # === Load camera calibration ===
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

        self.camera_height = config.get('camera_height', 0.381)

        # === Camera Setup ===
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

        # === TF Setup ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # === Odometry + State ===
        self.prev_odom = None
        self.prev_odom_pose = None
        self.last_marker_pose = None  # (x, y, theta)
        self.estimated_pose = (0.0, 0.0, 0.0)
        self.marker_visible = False

        self.odom_subscription = self.create_subscription(
            Odometry, 'odomWheel', self.odom_callback, 10
        )

        self.timer = self.create_timer(1/5, self.process_frame)  # 5 Hz
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
        self.get_logger().info("Published static transform: camera_frame → base_link")

    def extract_pose_from_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)  # Assuming qx=qy=0
        return (x, y, theta)

    def odom_callback(self, msg):
        if not self.marker_visible and self.prev_odom and self.prev_odom_pose:
            current_pose = self.extract_pose_from_odom(msg)
            dx = current_pose[0] - self.prev_odom_pose[0]
            dy = current_pose[1] - self.prev_odom_pose[1]
            dtheta = current_pose[2] - self.prev_odom_pose[2]

            x0, y0, theta0 = self.estimated_pose
            self.estimated_pose = (
                x0 + dx * math.cos(theta0) - dy * math.sin(theta0),
                y0 + dx * math.sin(theta0) + dy * math.cos(theta0),
                theta0 + dtheta
            )

        self.prev_odom = msg

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if not detections:
            self.marker_visible = False
            self.publish_pose_transform(*self.estimated_pose)
            self.get_logger().info("[Fallback] No marker, publishing odometry-integrated pose.")
            return

        for marker in detections:
            marker_id = marker['id']
            corners = marker['corners']

            # Undistort
            undistorted_corners = cv.undistortPoints(
                np.expand_dims(corners, axis=1),
                self.camera_matrix,
                self.dist_coeffs,
                P=self.camera_matrix
            ).reshape(-1, 2)

            # Estimate marker center
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

            # Compute orientation
            top_center = np.mean(corners[0:2], axis=0)
            bottom_center = np.mean(corners[2:4], axis=0)
            dx = top_center[0] - bottom_center[0]
            dy = top_center[1] - bottom_center[1]
            yaw = np.arctan2(dx, -dy)

            T_marker_cam = concatenate_matrices(
                translation_matrix(cam_in_marker_pos),
                quaternion_matrix(quaternion_from_euler(0.0, 0.0, yaw))
            )

            T_base_camera = concatenate_matrices(
                translation_matrix([-0.192, 0.0, 0.0]),
                quaternion_matrix(quaternion_from_euler(0.0, 0.0, 0.0))
            )

            T_map_base = T_marker_cam @ T_base_camera
            trans = translation_from_matrix(T_map_base)
            quat = quaternion_from_matrix(T_map_base)
            yaw_est = math.atan2(2.0 * (quat[3] * quat[2]), 1.0 - 2.0 * (quat[2] ** 2))

            # Save new true pose
            self.last_marker_pose = (trans[0], trans[1], yaw_est)
            self.estimated_pose = self.last_marker_pose
            self.marker_visible = True

            if self.prev_odom:
                self.prev_odom_pose = self.extract_pose_from_odom(self.prev_odom)

            self.publish_pose_transform(*self.estimated_pose)
            self.get_logger().info(
                f"[Marker] Detected ArUco {marker_id} | x={trans[0]:.2f}, y={trans[1]:.2f}, yaw={math.degrees(yaw_est):.1f}°"
            )

    def publish_pose_transform(self, x, y, theta):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)


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
