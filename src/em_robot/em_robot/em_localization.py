#!/usr/bin/env python3
import os
import yaml
import cv2 as cv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
from picamera2 import Picamera2
from libcamera import controls
from ament_index_python.packages import get_package_share_directory

from submodules.detect_aruco import CameraVisionStation, detector


class LocalizationNode(Node):
    """
    ROS 2 Node for camera-based localization using ArUco markers.
    """
    def __init__(self):
        super().__init__('localization_node')

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
        cam_params = node_config.get('cam_params', {})
        aruco_params = node_config.get('aruco_params', {})

        # Set camera resolution (default: 4608x2592)
        size_list = cam_params.get('size', [4608, 2592])
        self.size = tuple(size_list)

        # Read camera height and compute lens position as 1 / camera_height
        self.camera_height = cam_params.get('camera_height', {})
        self.lens_position = 1.0 / self.camera_height

        self.get_logger().info(f"Camera parameters: {cam_params}")
        self.get_logger().info(f"Aruco parameters: {aruco_params}")
        self.get_logger().info(f"Calculated lens position: {self.lens_position:.4f}")

        # Initialize the vision station for ArUco detection
        self.get_logger().info("Initializing CameraVisionStation...")
        self.cam = CameraVisionStation(cam_params=cam_params,
                                       aruco_params=aruco_params,
                                       cam_frame=self.size)

        # Initialize Picamera2 with the configured resolution
        self.get_logger().info("Starting Picamera2...")
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual,
                                  "LensPosition": self.lens_position})

        # Create publisher for robot pose
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'odomCam', 5)
        self.get_logger().info("Pose publisher initialized on topic 'odomCam'.")

        # Set up a timer to process frames using the timer_period from YAML (default: 0.2 seconds)
        timer_period = cam_params.get('timer_period', 0.2)
        self.timer = self.create_timer(timer_period, self.process_frame)

    def process_frame(self):
        """
        Captures an image, detects ArUco markers, computes the robot's pose,
        and publishes it if detection is successful.
        """
        self.get_logger().debug("Capturing frame...")
        frame = self.picam2.capture_array()
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Detect ArUco markers (detailed logs are handled in the detector)
        markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)
        if markerIds is None:
            return

        # Compute the robot's pose
        robot_pose, robot_angle = self.cam.get_robot_pose(gray_frame, markerCorners, markerIds)
        if robot_pose is None:
            return

        # Create and populate the pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = robot_pose[1]
        pose_msg.pose.pose.position.y = -robot_pose[0]

        # Convert Euler angle to quaternion representation
        qx, qy, qz, qw = quaternion_from_euler(0, 0, robot_angle)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        self.get_logger().info("Publishing robot pose...")
        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LocalizationNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
