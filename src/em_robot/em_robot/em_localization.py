#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_from_matrix

from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory

# Use the detection function from your aruco_detection.py
from .aruco_detection import detect_aruco_corners

class MarkerLocalizationNode(Node):
    """
    ROS Node that:
      - Reads camera intrinsics from YAML
      - Captures images from PiCamera2
      - Detects ArUco markers
      - Uses solvePnP to compute the transform marker->camera
      - Publishes TF where parent=aruco_<id>, child=camera_frame
        (So you can see the camera in the marker’s coordinate system.)
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
        # We expect e.g.:
        #   camera_matrix:
        #     - [2078,    0.0, 2304]
        #     - [   0, 2068.0, 1296]
        #     - [   0,    0.0,    1]
        #   dist_coeff: [k1, k2, p1, p2, k3]
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

        # Example marker side length in METERS
        # (You might store this in your YAML or pass as a param.)
        self.marker_size_m = 0.036  # 3.6cm, for example

        # Prepare the corners in 3D object coords (marker frame)
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

        # 4) Timer to detect markers
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def process_frame(self):
        # A) Capture image
        frame = self.picam2.capture_array()

        # B) Detect all markers
        detections = detect_aruco_corners(frame)
        if len(detections) == 0:
            self.get_logger().info("No markers detected.")
            return

        # C) For each marker, run solvePnP to get marker->camera
        for marker_dict in detections:
            marker_id = marker_dict['id']
            corners_2d = marker_dict['corners']  # shape (4,2), each row = (x_px, y_px)

            # solvePnP wants shape (N,3) and (N,2), so we have 4 corners in each
            #   objectPoints = self.objp_4corners
            #   imagePoints = corners_2d
            # Distortion = self.dist_coeffs
            # Intrinsic = self.camera_matrix
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

            # rvec, tvec describe the transform from marker frame -> camera frame
            # That is: camera = R * marker + t

            # D) Convert rvec, tvec into a 4x4 transform
            # rotation vector -> rotation matrix
            R_ca, _ = cv.Rodrigues(rvec)  # 3x3
            t_ca = tvec.reshape((3,1))    # 3x1

            # We'll build a 4x4 homogeneous transform for "marker->camera"
            T_marker_camera = np.eye(4, dtype=np.float32)
            T_marker_camera[0:3,0:3] = R_ca
            T_marker_camera[0:3,3]   = t_ca[:,0]

            # E) Convert that to a TF (marker->camera)
            #    parent = "aruco_<id>", child = "camera_frame"
            # translation in meters is simply T_marker_camera[:3,3]
            tx = float(T_marker_camera[0,3])
            ty = float(T_marker_camera[1,3])
            tz = float(T_marker_camera[2,3])

            # For the rotation, we can either:
            #   (1) convert the 3x3 directly to a quaternion, or
            #   (2) re-generate from rvec with tf_transformations
            # We'll do direct matrix->quaternion:
            # tf_transformations.quaternion_from_matrix wants a 4x4
            import tf_transformations
            quat = tf_transformations.quaternion_from_matrix(T_marker_camera)
            qx, qy, qz, qw = [float(q) for q in quat]

            # F) Publish TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = f"aruco_{marker_id}"   # parent
            t.child_frame_id = "camera_frame"          # child

            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)
            self.get_logger().info(
                f"Marker {marker_id}: T_marker->camera = "
                f"({tx:.3f},{ty:.3f},{tz:.3f}) + quaternion({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})"
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
