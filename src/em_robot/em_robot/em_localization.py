#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf_transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_matrix,
    translation_from_matrix,
    concatenate_matrices
)
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory


# --- ArUco Detection Function (inlined) ---
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
detector_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, detector_params)

def detect_aruco_corners(frame):
    if len(frame.shape) == 3:
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    else:
        gray_frame = frame

    corners_list, ids, _ = detector.detectMarkers(gray_frame)
    if ids is None or len(ids) == 0:
        return []

    results = []
    for corners, id_ in zip(corners_list, ids.flatten()):
        c = corners[0]  # (1,4,2) → (4,2)
        results.append({'id': int(id_), 'corners': c})
    return results


# --- Main Node ---
class MarkerLocalizationNode(Node):
    def __init__(self):
        super().__init__('marker_localization_node')

        # Load camera config
        pkg_share = get_package_share_directory('em_robot')
        config_path = os.path.join(pkg_share, 'config/calibration.yaml')
        self.get_logger().info(f"Loading configuration: {config_path}")

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        cammat_list = config.get('camera_matrix', [])
        dist_list = config.get('dist_coeff', [])
        if isinstance(dist_list, str):
            dist_list = [float(x) for x in dist_list.strip().split()]

        self.camera_matrix = np.array(cammat_list, dtype=np.float32)
        self.dist_coeffs = np.array(dist_list, dtype=np.float32)
        self.camera_height = config.get('camera_height', 0.375)#0.625)
        self.marker_size = config.get('marker_size', 0.038)

        # Init camera
        self.size = (4608, 2592) #(1536, 864)
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": 6.0
            #"LensPosition": 6.225 // Quand on teste à côté du bureau
        })

        # ROS setup
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_markers_pose', 10)

        self.publish_static_transform()

        self.timer = self.create_timer(1/2, self.process_frame)  # 5 Hz
        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "camera_frame"
        static_transform.child_frame_id = "base_link"
        static_transform.transform.translation.x = -0.192
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform: camera_frame → base_link")

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if not detections:
            self.get_logger().info("No markers detected.")
            return

        for marker in detections:
            marker_id = marker['id']
            corners = marker['corners'].astype(np.float32)

            # Undistort corners
            undistorted = cv.undistortPoints(
                np.expand_dims(corners, axis=1),
                self.camera_matrix,
                self.dist_coeffs,
                P=self.camera_matrix
            ).reshape(-1, 2)

            # Marker position (approx)
            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            marker_center = np.mean(undistorted, axis=0)
            offset = np.array([cx, cy]) - marker_center
            x_cam = self.camera_height * offset[0] / (fx)
            y_cam = self.camera_height * offset[1] / (fy)

            # Yaw estimation from marker orientation
            top_center = np.mean(undistorted[0:2], axis=0)
            bottom_center = np.mean(undistorted[2:4], axis=0)
            dx = top_center[0] - bottom_center[0]
            dy = top_center[1] - bottom_center[1]
            yaw = np.arctan2(dx, -dy)

            # Transform: marker → camera
            quat = quaternion_from_euler(0.0, 0.0, yaw)
            T_marker_cam = concatenate_matrices(
                translation_matrix([x_cam, y_cam, 0.0]),
                quaternion_matrix(quat)
            )
            T_cam_marker = np.linalg.inv(T_marker_cam)
            trans = translation_from_matrix(T_cam_marker)
            quat_inv = quaternion_from_matrix(T_cam_marker)

            # Broadcast TF
            t_camera_marker = TransformStamped()
            t_camera_marker.header.stamp = self.get_clock().now().to_msg()
            t_camera_marker.header.frame_id = f"aruco_{marker_id}"
            t_camera_marker.child_frame_id = "camera_frame"
            t_camera_marker.transform.translation.x = trans[0]
            t_camera_marker.transform.translation.y = trans[1]
            t_camera_marker.transform.translation.z = 0.0
            t_camera_marker.transform.rotation.x = quat_inv[0]
            t_camera_marker.transform.rotation.y = quat_inv[1]
            t_camera_marker.transform.rotation.z = quat_inv[2]
            t_camera_marker.transform.rotation.w = quat_inv[3]
            self.tf_broadcaster.sendTransform(t_camera_marker)

            # Publish PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_frame"
            pose_msg.pose.position.x = x_cam
            pose_msg.pose.position.y = y_cam
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(pose_msg)

            self.get_logger().info(
                f"[Undistorted] Marker {marker_id}: x={x_cam:.3f}, y={y_cam:.3f}, yaw={np.degrees(yaw):.1f}°"
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
