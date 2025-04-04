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
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
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
        pkg_path = get_package_share_directory('em_robot')
        config_path = os.path.join(pkg_path, 'config', 'calibration.yaml')
        self.get_logger().info(f"Loading configuration: {config_path}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.camera_matrix = np.array(config['camera_matrix'], dtype=np.float32)
        self.dist_coeffs = np.array(config['dist_coeff'], dtype=np.float32)
        self.camera_height = config.get('camera_height', 0.381)

        # === Camera setup ===
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (1536, 864)}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": 1.0 / self.camera_height
        })

        # === TF setup ===
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # === Odometry handling ===
        self.odom_subscription = self.create_subscription(
            Odometry, 'odomWheel', self.odom_callback, 10
        )
        self.prev_odom_pose = None  # (x, y, theta)

        # === Internal state ===
        self.last_camera_pose = None  # (x, y, theta) in aruco_<id>
        self.last_marker_id = None
        self.marker_visible = False

        self.timer = self.create_timer(1/5.0, self.process_frame)
        self.get_logger().info("MarkerLocalizationNode: started.")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_frame"
        t.child_frame_id = "base_link"
        t.transform.translation.x = -0.192
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.static_tf_broadcaster.sendTransform(t)

    def extract_odom_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        theta = 2.0 * math.atan2(q.z, q.w)  # assuming qx = qy = 0
        return (x, y, theta)

    def odom_callback(self, msg):
        if not self.marker_visible and self.prev_odom_pose and self.last_camera_pose:
            current = self.extract_odom_pose(msg)
            prev = self.prev_odom_pose
            dx = current[0] - prev[0]
            dy = current[1] - prev[1]
            dtheta = current[2] - prev[2]

            # Prevent "creep" when stationary
            if math.hypot(dx, dy) < 1e-4 and abs(dtheta) < 1e-3:
                return

            x_c, y_c, theta_c = self.last_camera_pose
            new_x = x_c + dx * math.cos(theta_c) - dy * math.sin(theta_c)
            new_y = y_c + dx * math.sin(theta_c) + dy * math.cos(theta_c)
            new_theta = theta_c + dtheta

            self.last_camera_pose = (new_x, new_y, new_theta)
            self.publish_camera_tf(self.last_camera_pose, self.last_marker_id)

        self.prev_odom_pose = self.extract_odom_pose(msg)

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if not detections:
            self.marker_visible = False
            return

        # Marker seen
        marker = detections[0]  # just use first marker for now
        marker_id = marker['id']
        corners = marker['corners']

        # Undistort corners
        corners = cv.undistortPoints(
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

        top = np.mean(corners[0:2], axis=0)
        bottom = np.mean(corners[2:4], axis=0)
        dx = top[0] - bottom[0]
        dy = top[1] - bottom[1]
        yaw = np.arctan2(dx, -dy)

        self.last_camera_pose = (x_cam, y_cam, yaw)
        self.prev_odom_pose = None  # reset odom reference
        self.last_marker_id = marker_id
        self.marker_visible = True

        self.publish_camera_tf(self.last_camera_pose, marker_id)

        self.get_logger().info(
            f"[Aruco] Updated camera_frame ← aruco_{marker_id} | x={x_cam:.2f}, y={y_cam:.2f}, yaw={math.degrees(yaw):.1f}°"
        )

    def publish_camera_tf(self, pose, marker_id):
        if marker_id is None:
            return  # not valid

        x, y, theta = pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"aruco_{marker_id}"
        t.child_frame_id = "camera_frame"
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
