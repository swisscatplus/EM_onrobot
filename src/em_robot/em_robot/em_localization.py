#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, concatenate_matrices, translation_from_matrix, quaternion_from_matrix
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration

from .submodules.aruco_detection import detect_aruco_corners

from tf_transformations import quaternion_from_euler

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

        self.camera_height = config.get('camera_height', 0.38)

        self.size = (1280, 720)
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

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"
        static_transform.child_frame_id = "camera_frame"

        static_transform.transform.translation.x = 0.198
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        quat = quaternion_from_euler(0.0, np.pi, np.pi/2)
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform: base_link -> camera_frame")

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if not detections:
            self.get_logger().info("No markers detected.")
            return

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        for marker in detections:
            marker_id = marker['id']
            corners = marker['corners']  # shape: (4, 2)

            # Estimate center of marker in image
            marker_center = np.mean(corners, axis=0)
            pixel_offset = marker_center - np.array([cx, cy])

            # Project camera position into marker frame (unit depth)
            x_cam = -pixel_offset[0] / fx  # Negative because marker X is to the left
            y_cam = -pixel_offset[1] / fy  # Negative because marker Y is up
            z_cam = 1.0  # Marker Z points into image

            # Position of camera in marker frame
            cam_in_marker_pos = np.array([x_cam, y_cam, z_cam])

            # Estimate yaw: vector from bottom to top (since Y is up)
            top_center = np.mean(corners[0:2], axis=0)
            bottom_center = np.mean(corners[2:4], axis=0)
            dx, dy = top_center - bottom_center

            # Y-axis goes up in marker, so angle is atan2(dx, dy)
            yaw = np.arctan2(dx, dy)

            # Camera's orientation in marker frame is opposite to what it sees
            quat = quaternion_from_euler(0.0, 0.0, yaw)

            # Create transform: aruco_<id> → camera_frame
            t_marker_camera = TransformStamped()
            t_marker_camera.header.stamp = self.get_clock().now().to_msg()
            t_marker_camera.header.frame_id = f"aruco_{marker_id}"
            t_marker_camera.child_frame_id = "camera_frame"

            t_marker_camera.transform.translation.x = cam_in_marker_pos[0]
            t_marker_camera.transform.translation.y = cam_in_marker_pos[1]
            t_marker_camera.transform.translation.z = cam_in_marker_pos[2]

            t_marker_camera.transform.rotation.x = quat[0]
            t_marker_camera.transform.rotation.y = quat[1]
            t_marker_camera.transform.rotation.z = quat[2]
            t_marker_camera.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(t_marker_camera)
            self.get_logger().info(
                f"[TF] Published: aruco_{marker_id} → camera_frame: "
                f"x={x_cam:.2f}, y={y_cam:.2f}, z={z_cam:.2f}, yaw={np.degrees(yaw):.1f}°"
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
