#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration

# Use the detection function from your aruco_detection.py
from .submodules.aruco_detection import detect_aruco_corners

class MarkerLocalizationNode(Node):
    """
    ROS Node that:
      - Loads camera calibration data from YAML.
      - Captures images from PiCamera2.
      - Detects ArUco markers.
      - Uses an offset-based method (with TF lookup) to compute the camera's 2D pose (x, y, yaw).
        It uses the marker's detected center and the known marker map position (via TF).
      - Publishes a PoseWithCovarianceStamped in the map frame.
    """
    def __init__(self):
        super().__init__('marker_localization_node')

        # 1) Load camera configuration from YAML.
        package_name = 'em_robot'
        config_filename = 'config/calibration.yaml'
        pkg_share = get_package_share_directory(package_name)
        config_path = os.path.join(pkg_share, config_filename)
        self.get_logger().info(f"Loading configuration: {config_path}")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Extract camera intrinsics.
        cammat_list = config.get('camera_matrix', [])
        dist_list = config.get('dist_coeff', [])
        self.camera_matrix = np.array(cammat_list, dtype=np.float32)
        self.dist_coeffs = np.array(dist_list, dtype=np.float32)
        if self.camera_matrix.shape != (3, 3):
            raise ValueError("Camera matrix must be 3x3 in the YAML!")
        if self.dist_coeffs.shape[0] < 4:
            raise ValueError("dist_coeff must have at least 4 values (k1, k2, p1, p2)!")

        self.get_logger().info(f"Loaded camera_matrix=\n{self.camera_matrix}")
        self.get_logger().info(f"Loaded dist_coeffs={self.dist_coeffs}")

        # Fixed camera height (in meters). This is used to convert pixel offsets into meters.
        self.camera_height = config.get('camera_height', 0.38)

        # 2) Initialize the camera.
        self.size = (1280, 720)  # Image size (or set from config)
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()

        # Example: set manual focus.
        lens_position = 1.0 / self.camera_height
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position
        })

        # 3) Setup TF broadcasters (for publishing static transforms, if needed).
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # Create a TF buffer and listener to look up marker transforms.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4) Publisher for sensor message (PoseWithCovarianceStamped).
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'camera_pose', 10)

        # 5) Timer to process frames.
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        """
        Publishes a static transform from `base_link` to `camera_frame`
        representing the fixed mounting of the camera on the robot.
        """
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"
        static_transform.child_frame_id = "camera_frame"

        # Adjust these values based on your actual camera mounting.
        static_transform.transform.translation.x = 0.198  # e.g., 15 cm forward.
        static_transform.transform.translation.y = 0.0    # Centered.
        static_transform.transform.translation.z = 0.0    # e.g., 20 cm up.
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 1.57
        static_transform.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform: base_link -> camera_frame")

    def process_frame(self):
        # A) Capture image.
        frame = self.picam2.capture_array()

        # B) Detect all markers (corners and IDs).
        detections = detect_aruco_corners(frame)
        if len(detections) == 0:
            self.get_logger().info("No markers detected.")
            return

        # Extract intrinsic parameters from the camera matrix.
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        # C) Process each detected marker.
        for marker in detections:
            marker_id = marker['id']
            corners = marker['corners']  # shape (4,2)

            # Compute marker center in pixel coordinates.
            marker_center = np.mean(corners, axis=0)
            pixel_offset = marker_center - np.array([cx, cy])
            # Convert pixel offset to meters using the fixed camera height.
            offset_x = pixel_offset[0] * (self.camera_height / fx)
            offset_y = pixel_offset[1] * (self.camera_height / fy)
            offset_m = np.array([offset_x, offset_y])

            # Retrieve the marker's map position via TF lookup with a timeout.
            try:
                t_map_marker = self.tf_buffer.lookup_transform(
                    "map", f"aruco_{marker_id}", rclpy.time.Time(), Duration(seconds=0.1)
                )
            except Exception as e:
                self.get_logger().warn(f"Could not lookup TF for marker {marker_id}: {e}")
                continue

            # Marker map position (in meters).
            marker_map_position = np.array([
                t_map_marker.transform.translation.x,
                t_map_marker.transform.translation.y
            ])

            # Compute the camera's map position by subtracting the offset.
            camera_map_position = marker_map_position - offset_m

            # Compute marker orientation (yaw) using marker corners.
            # Assuming corners are ordered as: top-left, top-right, bottom-right, bottom-left.
            top_center = np.mean(corners[0:2], axis=0)
            bottom_center = np.mean(corners[2:4], axis=0)
            dx, dy = top_center - bottom_center
            rad_angle = np.arctan2(dy, dx)  # yaw in radians

            # --- Publish sensor message (PoseWithCovarianceStamped) in the map frame ---
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            pose_msg.pose.pose.position.x = float(camera_map_position[0])
            pose_msg.pose.pose.position.y = float(camera_map_position[1])
            pose_msg.pose.pose.position.z = 0.0  # For 2D localization

            quat = quaternion_from_euler(0.0, 0.0, rad_angle)
            pose_msg.pose.pose.orientation.x = quat[0]
            pose_msg.pose.pose.orientation.y = quat[1]
            pose_msg.pose.pose.orientation.z = quat[2]
            pose_msg.pose.pose.orientation.w = quat[3]

            # Example covariance matrix.
            cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
            pose_msg.pose.covariance = cov

            self.pose_pub.publish(pose_msg)
            self.get_logger().info(
                f"Marker {marker_id}: camera pose in map: x={camera_map_position[0]:.3f} m, "
                f"y={camera_map_position[1]:.3f} m, yaw={rad_angle:.3f} rad"
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
