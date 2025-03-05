#!/usr/bin/env python3

import os
import yaml
import cv2 as cv
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from tf_transformations import quaternion_from_euler
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory

import math

from .submodules.detect_aruco import detector  # or however you import your ArUco detector

class MarkerLocalizationNode(Node):
    """
    ROS 2 Node for detecting ArUco (or QR) markers and publishing their poses in the camera frame.
    """

    def __init__(self):
        super().__init__('marker_localization_node')

        # Load configuration parameters from YAML
        package_name = 'em_robot'
        config_filename = 'config/cam.yaml'
        pkg_share = get_package_share_directory(package_name)
        config_file_path = os.path.join(pkg_share, config_filename)

        self.get_logger().info(f"Loading configuration from {config_file_path}...")
        if not os.path.exists(config_file_path):
            self.get_logger().error(f"Configuration file not found: {config_file_path}")
            raise FileNotFoundError(f"Configuration file not found: {config_file_path}")

        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)

        # Extract ROS parameters for this node
        node_config = config.get(self.get_name(), {}).get('ros__parameters', {})
        if not node_config:
            self.get_logger().warn("No parameters found under this node's name. Using defaults.")
        cam_params = node_config.get('cam_params', {})
        aruco_params = node_config.get('aruco_params', {})

        # Camera resolution
        self.size = tuple(cam_params.get('size', [4608, 2592]))
        # How often to process frames (seconds)
        self.timer_period = cam_params.get('timer_period', 0.5)
        # For manual focus (example logic)
        self.camera_height = cam_params.get('camera_height', 0.63)
        self.lens_position = 1.0 / self.camera_height

        # ArUco known markers from YAML
        # e.g., aruco_params is a dictionary like: { 11: {t_x:..., t_y:..., yaw:...}, 618: {...}, ... }
        self.known_markers = aruco_params

        # Initialize camera
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": self.lens_position
        })

        # TF broadcaster (to publish transforms of markers)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a publisher for Odometry or Pose messages
        # We’ll publish to "marker_odom" for example
        self.odom_publisher = self.create_publisher(Odometry, 'marker_odom', 10)
        # Alternatively, if you prefer PoseStamped:
        # self.pose_publisher = self.create_publisher(PoseStamped, 'marker_pose', 10)

        # Timer to grab frames and detect markers
        self.timer = self.create_timer(self.timer_period, self.process_frame)

        self.get_logger().info("MarkerLocalizationNode: started.")

    def process_frame(self):
        """
        Capture image, detect markers, and publish TF + Odometry in camera frame.
        """
        frame = self.picam2.capture_array()
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)
        if markerIds is None:
            return

        # For each detected marker, check if ID is in known_markers
        for corners, marker_id in zip(markerCorners, markerIds.flatten()):
            if marker_id not in self.known_markers:
                # Skip if not in YAML
                continue

            # corners is an array of 4 points (x,y) of the detected marker in the image
            # For a simple approximation, we can take the center of corners as the marker's 2D location in the image.
            # Then you'd use your known camera intrinsics to estimate the 3D position in front of the camera.
            # For demonstration, let's just set some dummy transformations or
            # do a quick planar approximation from the corners.

            # Approximate marker center in image coordinates
            center_x = 0.0
            center_y = 0.0
            for pt in corners[0]:
                center_x += pt[0]
                center_y += pt[1]
            center_x /= 4.0
            center_y /= 4.0

            # Convert from image coords -> camera coords (you should replace with your own geometry)
            # Simple pinhole camera model: X = (u - cx)/fx * Z, Y = (v - cy)/fy * Z, etc.
            # This example is just a placeholder to demonstrate we have a position in the camera frame.
            # Suppose we guess Z = 0.5 m in front of camera just to show the transform works:
            Z_est = 0.5
            fx = float(self.known_markers[marker_id].get('fx', 1000.0))
            fy = float(self.known_markers[marker_id].get('fy', 1000.0))
            cx = self.size[0] / 2.0
            cy = self.size[1] / 2.0

            X = (center_x - cx) / fx * Z_est
            Y = (center_y - cy) / fy * Z_est
            # In many coordinate systems, the camera points +Z forward, +X to the right, +Y down.
            # Adapt as needed. For now, we’ll define camera_frame with +X right, +Y up, +Z forward.

            # Orientation: for this demo, we’ll say marker is "facing" camera with no rotation
            # If you want the actual orientation from solvePnP or known marker orientation, do so.
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, 0.0)

            # ----------------------
            # 1) Publish TF transform
            # ----------------------
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "camera_frame"  # name of your camera frame
            transform.child_frame_id = f"marker_{marker_id}"
            transform.transform.translation.x = X
            transform.transform.translation.y = Y
            transform.transform.translation.z = Z_est
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(transform)

            # ----------------------
            # 2) Publish Odometry (or Pose) for visualization
            # ----------------------
            odom_msg = Odometry()
            odom_msg.header.stamp = transform.header.stamp
            odom_msg.header.frame_id = "camera_frame"     # where the pose is expressed
            odom_msg.child_frame_id = f"marker_{marker_id}"  # the "moving" frame (though it's actually stationary in camera frame)
            odom_msg.pose.pose.position.x = X
            odom_msg.pose.pose.position.y = Y
            odom_msg.pose.pose.position.z = Z_est
            odom_msg.pose.pose.orientation.x = qx
            odom_msg.pose.pose.orientation.y = qy
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw

            self.odom_publisher.publish(odom_msg)

            # If using PoseStamped instead:
            # pose_msg = PoseStamped()
            # pose_msg.header = odom_msg.header
            # pose_msg.pose = odom_msg.pose.pose
            # self.pose_publisher.publish(pose_msg)

            self.get_logger().info(
                f"Detected marker {marker_id} -> Published TF & Odom in camera_frame"
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

if __name__ == '__main__':
    main()
