#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf_transformations import (
    quaternion_from_euler,
    euler_from_matrix,
    quaternion_matrix,
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
      - Uses solvePnP to compute the transform from marker to camera
      - Inverts that transform so that you obtain the camera pose in the marker frame.
      - Uses a TF lookup to get the marker’s pose in the map frame.
      - Computes the camera’s pose in the map by chaining the transforms.
      - Publishes:
          1) A dynamic TF from the marker to the camera (for visualization).
          2) A sensor message (PoseWithCovarianceStamped) in the map frame.
    """

    def __init__(self):
        super().__init__('marker_localization_node')

        # 1) Load camera configuration from YAML.
        package_name = 'em_robot'
        config_filename = 'config/calibration.yaml'
        pkg_share = get_package_share_directory(package_name)
        config_path = os.path.join(pkg_share, config_filename)
        self.get_logger().info(f"Loading camera config: {config_path}")

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

        # Marker dimensions (in meters)
        self.marker_size_m = 0.036  # e.g., 3.6 cm

        # 3D coordinates for the marker corners (in marker coordinate system)
        half_side = self.marker_size_m / 2.0
        self.objp_4corners = np.array([
            [-half_side,  half_side, 0.0],
            [ half_side,  half_side, 0.0],
            [ half_side, -half_side, 0.0],
            [-half_side, -half_side, 0.0]
        ], dtype=np.float32)

        # 2) Initialize the camera.
        self.size = (4608, 2592)  # or read from config
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()

        # Example: set manual focus.
        camera_height = 0.63
        lens_position = 1.0 / camera_height
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": lens_position
        })

        # 3) Setup TF broadcasters.
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Also create a TF buffer and listener to lookup transforms.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for sensor message (PoseWithCovarianceStamped).
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'camera_pose', 10)

        # 4) Publish the static transform from base_link to camera_frame.
        self.publish_static_transform()

        # 5) Timer to process frames.
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        """
        Publishes a static transform from `base_link` to `camera_frame`
        representing the fixed mounting of the camera on the robot.
        Uses StaticTransformBroadcaster.
        """
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "base_link"  # Parent frame.
        static_transform.child_frame_id = "camera_frame"  # Child frame.

        # Adjust these values based on your actual camera mounting.
        static_transform.transform.translation.x = 0.198  # e.g., 15 cm forward.
        static_transform.transform.translation.y = 0.0    # Centered.
        static_transform.transform.translation.z = 0.0    # e.g., 20 cm up.

        # Assume the camera is facing forward.
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Broadcast the static transform once (latched).
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

        # C) Process each detected marker.
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

            # --- Invert the transform ---
            # solvePnP gives: X_cam = R * X_marker + t, where t is the marker position in the camera frame.
            # To get the camera pose relative to the marker, invert the transform:
            R_marker_camera, _ = cv.Rodrigues(rvec)
            t_marker_camera = tvec.reshape((3, 1))
            # Inversion:
            R_camera_marker = R_marker_camera.T
            t_camera_marker = -R_camera_marker.dot(t_marker_camera)
            # Build the 4x4 homogeneous transform from marker to camera.
            T_camera_marker = np.eye(4, dtype=np.float32)
            T_camera_marker[0:3, 0:3] = R_camera_marker
            T_camera_marker[0:3, 3] = t_camera_marker[:, 0]

            # --- Lookup the transform from map to the marker ---
            # (Assumes a TF broadcaster is publishing a transform from "map" to "aruco_{marker_id}")
            try:
                # Use the latest available transform.
                t_map_marker = self.tf_buffer.lookup_transform("map", f"aruco_{marker_id}", rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f"Could not get transform from map to aruco_{marker_id}: {e}")
                continue

            # Convert the TransformStamped to a 4x4 matrix.
            T_map_marker = np.eye(4, dtype=np.float32)
            q = [t_map_marker.transform.rotation.x,
                 t_map_marker.transform.rotation.y,
                 t_map_marker.transform.rotation.z,
                 t_map_marker.transform.rotation.w]
            R_map_marker = quaternion_matrix(q)[0:3, 0:3]
            T_map_marker[0:3, 0:3] = R_map_marker
            T_map_marker[0, 3] = t_map_marker.transform.translation.x
            T_map_marker[1, 3] = t_map_marker.transform.translation.y
            T_map_marker[2, 3] = t_map_marker.transform.translation.z

            # --- Chain the transforms to get the camera pose in the map ---
            # T_map_camera = T_map_marker * T_camera_marker
            T_map_camera = T_map_marker.dot(T_camera_marker)

            # Extract position from T_map_camera.
            tx = float(T_map_camera[0, 3])
            ty = float(T_map_camera[1, 3])
            # For 2D localization, force z to zero.
            tz = 0.0

            # Extract yaw from T_map_camera.
            roll, pitch, yaw = euler_from_matrix(T_map_camera, 'sxyz')
            yaw_quat = quaternion_from_euler(0.0, 0.0, yaw)
            qx, qy, qz, qw = [float(q) for q in yaw_quat]

            # --- Publish a dynamic TF for visualization (marker -> camera) ---
            dynamic_tf = TransformStamped()
            dynamic_tf.header.stamp = self.get_clock().now().to_msg()
            dynamic_tf.header.frame_id = f"aruco_{marker_id}"  # Parent: marker frame.
            dynamic_tf.child_frame_id = "camera_frame"           # Child: camera frame.
            # For dynamic TF, we can publish the original (inverted) transform.
            dynamic_tf.transform.translation.x = float(t_camera_marker[0])
            dynamic_tf.transform.translation.y = float(t_camera_marker[1])
            dynamic_tf.transform.translation.z = float(t_camera_marker[2])
            # Compute yaw from R_camera_marker.
            r_dyn, p_dyn, y_dyn = euler_from_matrix(
                np.vstack((np.hstack((R_camera_marker, t_camera_marker)), [0, 0, 0, 1])),
                'sxyz'
            )
            quat_dyn = quaternion_from_euler(0.0, 0.0, y_dyn)
            dynamic_tf.transform.rotation.x = quat_dyn[0]
            dynamic_tf.transform.rotation.y = quat_dyn[1]
            dynamic_tf.transform.rotation.z = quat_dyn[2]
            dynamic_tf.transform.rotation.w = quat_dyn[3]
            self.tf_broadcaster.sendTransform(dynamic_tf)

            self.get_logger().info(
                f"Marker {marker_id}: camera pose in map: x={tx:.3f}m, y={ty:.3f}m, yaw={yaw:.3f}rad"
            )

            # --- Publish sensor message (PoseWithCovarianceStamped) in the map frame ---
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            # Since the computed pose is in the map, set the header accordingly.
            pose_msg.header.frame_id = "map"

            pose_msg.pose.pose.position.x = tx
            pose_msg.pose.pose.position.y = ty
            pose_msg.pose.pose.position.z = tz

            pose_msg.pose.pose.orientation.x = qx
            pose_msg.pose.pose.orientation.y = qy
            pose_msg.pose.pose.orientation.z = qz
            pose_msg.pose.pose.orientation.w = qw

            # Example covariance matrix: low uncertainty in x, y, and yaw; high in z, roll, pitch.
            cov = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 1000.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 1000.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 1000.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.01]
            pose_msg.pose.covariance = cov

            self.pose_pub.publish(pose_msg)
            self.get_logger().info(f"Published PoseWithCovarianceStamped in map frame for marker {marker_id}")

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
