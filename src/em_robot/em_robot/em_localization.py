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
import math

# Your ArUco detection code
from .submodules.detect_aruco import CameraVisionStation, detector


class LocalizationNode(Node):
    """
    ROS 2 Node for camera-based localization using ArUco markers.

    This example flips the camera image horizontally before detection,
    then publishes the camera pose in 'map' coordinates without further
    manual axis swapping. We rely on TF to handle orientation.
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

        # Read camera height and compute lens position
        self.camera_height = cam_params.get('camera_height', 2.0)
        self.lens_position = 1.0 / float(self.camera_height)

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

        # Set manual focus based on lens position
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": self.lens_position
        })

        # Create publisher for camera pose
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'odomCam', 5)
        self.get_logger().info("Pose publisher initialized on topic 'odomCam'.")

        # Set up a timer to process frames (default: 0.2 seconds)
        timer_period = cam_params.get('timer_period', 0.2)
        self.timer = self.create_timer(timer_period, self.process_frame)

    def process_frame(self):
        """
        Captures an image, flips it horizontally, detects ArUco markers,
        computes the camera's pose in the map frame, and publishes it if
        detection is successful.
        """
        self.get_logger().debug("Capturing frame...")
        # Capture raw frame
        frame = self.picam2.capture_array()

        # Convert to grayscale
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Flip horizontally (flipCode=1). If you need vertical instead, use flipCode=0.
        gray_frame = cv.flip(gray_frame, 1)

        # Detect ArUco markers
        markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)
        if markerIds is None:
            return  # No markers detected

        # Get the camera pose (x, y) and heading angle from your detection routine.
        camera_pose, camera_angle = self.cam.get_robot_pose(gray_frame, markerCorners, markerIds)
        if camera_pose is None:
            return

        # Here, we assume your 'get_robot_pose' now returns x=forward, y=left,
        # or whatever convention you want, because we've handled the flip.
        # We'll just pass them through as-is and rely on TF to interpret orientation.

        map_x = camera_pose[0]
        map_y = camera_pose[1]

        # Create and populate the pose message in the 'map' frame
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        pose_msg.pose.pose.position.x = map_x
        pose_msg.pose.pose.position.y = map_y

        # Convert Euler angle to quaternion (roll=0, pitch=0, yaw=camera_angle)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, camera_angle)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Publishing CAMERA pose in 'map': x={map_x:.2f}, y={map_y:.2f}, angle={math.degrees(camera_angle):.1f}°"
        )
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
