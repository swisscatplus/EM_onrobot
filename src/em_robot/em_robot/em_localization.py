#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import (
    quaternion_from_euler,
    euler_from_matrix,
)
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory

# Use the detection function from your aruco_detection.py
from .submodules.aruco_detection import detect_aruco_corners

class MarkerLocalizationNode(Node):
    """
    ROS Node that:
      - Reads camera intrinsics from YAML
      - Captures images from PiCamera2
      - Detects ArUco markers
      - Uses solvePnP to compute the transform marker->camera
      - Publishes:
        1) Dynamic TF: aruco_<id> -> camera_frame (computed from detections)
        2) Static TF: base_link -> camera_frame (fixed camera mounting position)
    """

    def __init__(self):
        super().__init__('marker_localization_node')

        # 1) Load config from YAML
        package_name = 'em_robot'
        config_filename = 'config/calibration.yaml'
        pkg_share = get_package_share_directory(package_name)
        config_path = os.path.join(pkg_share, config_filename)
        self.get_logger().info(f"Loading camera config: {config_path}")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Extract intrinsics from the YAML
        cammat_list = config.get('camera_matrix', [])
        dist_list = config.get('dist_coeff', [])

        # Convert to NumPy
        self.camera_matrix = np.array(cammat_list, dtype=np.float32)
        self.dist_coeffs = np.array(dist_list, dtype=np.float32)
        if self.camera_matrix.shape != (3,3):
            raise ValueError("Camera matrix must be 3x3 in the YAML!")
        if self.dist_coeffs.shape[0] < 4:
            raise ValueError("dist_coeff must have at least 4 values (k1, k2, p1, p2)!")

        self.get_logger().info(f"Loaded camera_matrix=\n{self.camera_matrix}")
        self.get_logger().info(f"Loaded dist_coeffs={self.dist_coeffs}")

        # Marker dimensions in meters
        self.marker_size_m = 0.036  # e.g. 3.6 cm

        # 3D Marker Corner Points (in marker's coordinate system)
        half_side = self.marker_size_m / 2.0
        self.objp_4corners = np.array([
            [-half_side,  half_side, 0.0],
            [ half_side,  half_side, 0.0],
            [ half_side, -half_side, 0.0],
            [-half_side, -half_side, 0.0]
        ], dtype=np.float32)

        # 2) Initialize camera
        self.size = (4608, 2592)  # or read from config
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()

        # Example: set manual focus
        camera_height = 0.63
        lens_position = 1.0 / camera_height
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position
        })

        # 3) TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 4) Publish the static transform (base_link -> camera_frame)
        self.publish_static_transform()

        # 5) Timer to detect markers
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        """
        Publishes a static transform from `base_link` to `camera_frame`,
        representing the physical mounting of the camera on the robot.
        """
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"  # Parent frame
        static_transform.child_frame_id = "camera_frame"  # Child frame

        # Adjust these values based on actual camera mounting position
        static_transform.transform.translation.x = 0.15  # 15 cm forward
        static_transform.transform.translation.y = 0.0   # Centered
        static_transform.transform.translation.z = 0.2   # 20 cm up

        # Assuming the camera is facing forward
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Broadcast this static transform
        self.tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform: base_link -> camera_frame")

    def process_frame(self):
        # A) Capture image
        frame = self.picam2.capture_array()

        # B) Detect all markers (just corners & IDs)
        detections = detect_aruco_corners(frame)
        if len(detections) == 0:
            self.get_logger().info("No markers detected.")
            return

        # C) Process each detected marker
        for marker_dict in detections:
            marker_id = marker_dict['id']
            corners_2d = marker_dict['corners']

            retval, rvec, tvec = cv.solvePnP(
                self.objp_4corners,
                corners_2d,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv.SOLVEPNP_ITERATIVE
            )

            if not retval:
                self.get_logger().warn(f"solvePnP failed for marker {marker_id}")
                continue

            # Convert rvec to rotation matrix
            R_ca, _ = cv.Rodrigues(rvec)
            t_ca = tvec.reshape((3,1))

            # Build 4x4 transform
            T_marker_camera = np.eye(4, dtype=np.float32)
            T_marker_camera[0:3, 0:3] = R_ca
            T_marker_camera[0:3, 3] = t_ca[:,0]

            # Extract 2D (x, y, yaw)
            tx = float(T_marker_camera[0, 3])
            ty = float(T_marker_camera[1, 3])
            tz = float(T_marker_camera[2, 3])

            roll, pitch, yaw = euler_from_matrix(T_marker_camera, 'sxyz')

            # Convert yaw into quaternion (ignoring roll & pitch)
            yaw_quat = quaternion_from_euler(0.0, 0.0, yaw)
            qx, qy, qz, qw = [float(q) for q in yaw_quat]

            # Publish dynamic transform: aruco_<id> -> camera_frame
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"aruco_{marker_id}"  # Parent
            t.child_frame_id = "camera_frame"  # Child

            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = 0.0  # Force 2D

            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

            self.get_logger().info(
                f"Marker {marker_id} -> camera in 2D: "
                f"x={tx:.3f}m, y={ty:.3f}m, yaw={yaw:.3f}rad (ignored tz={tz:.3f})"
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
