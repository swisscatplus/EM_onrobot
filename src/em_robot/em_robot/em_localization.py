import rclpy  # ROS 2 Python Client Library
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
import cv2 as cv
from picamera2 import Picamera2
from libcamera import controls
import yaml
import os
import pickle
from .submodules.detect_aruco import CameraVisionStation, detector

# ################# CONFIGURATION #################
package_name = 'em_robot'
params_path = 'config/cam.yaml'
timer_period = 0.2  # (seconds) Frequency of the publisher
# #################################################

# Locate the configuration files
pkg_share = os.path.join('/ros2_ws/install', package_name, 'share', package_name)
config_file_path = os.path.join(pkg_share, params_path)
calib_mat_file_path = os.path.join(pkg_share, 'config', 'cameraMatrix.pkl')
calib_dist_file_path = os.path.join(pkg_share, 'config', 'dist.pkl')


class LocalizationNode(Node):
    """
    ROS 2 Node responsible for capturing images, detecting ArUco markers,
    and computing the robot's position.
    """

    def __init__(self):
        super().__init__('localization_node')
        self.get_logger().info("Initializing LocalizationNode...")

        # Load camera parameters from YAML
        self.declare_parameter('config_file', config_file_path)
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        if os.path.exists(config_file):
            with open(config_file, 'r') as file:
                params = yaml.safe_load(file)
                self.get_logger().info(f"Loaded parameters: {params}")

            node_params = params.get(self.get_name(), {}).get('ros__parameters', {})
            cam_params = node_params.get('cam_params', {})
            lens_position = cam_params.get('lens_position', 2.32)
            aruco_params = node_params.get('aruco_params', {})

            self.get_logger().info(f"Camera Lens Position: {lens_position}")
            self.get_logger().info(f"Aruco Parameters Loaded: {aruco_params}")

        else:
            self.get_logger().error(f"Config file {config_file} does not exist")
            return

        # Initialize the camera
        self.size = (1152, 648)  # Image resolution
        self.get_logger().info("Initializing CameraVisionStation...")
        self.cam = CameraVisionStation(cam_params=cam_params, aruco_params=aruco_params, cam_frame=self.size)

        self.get_logger().info("Starting Picamera2...")
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}))
        self.picam2.start()
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": lens_position})

        # Publisher for the robot's pose
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'edi/cam', 5)
        self.get_logger().info("Pose publisher initialized on topic 'edi/cam'.")

        # Timer to capture and process frames periodically
        self.timer = self.create_timer(timer_period, self.process_frame)

    def process_frame(self):
        """
        Captures an image, detects ArUco markers, computes the robot pose,
        and publishes the pose if detection is successful.
        """
        self.get_logger().debug("Capturing frame...")
        frame = self.picam2.capture_array()
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert to grayscale

        # Detect ArUco markers in the frame
        markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)

        if markerIds is None:
            self.get_logger().warn("No ArUco markers detected in frame.")
            return

        self.get_logger().info(f"Detected ArUco markers: {markerIds.flatten().tolist()}")

        # Compute the robot's pose
        robot_pose, robot_angle = self.cam.get_robot_pose(gray_frame, markerCorners, markerIds)

        if robot_pose is None:
            self.get_logger().warn("Could not compute robot pose.")
            return

        self.get_logger().info(f"Computed Robot Pose: X={robot_pose[0]:.4f}, Y={robot_pose[1]:.4f}, Angle={robot_angle:.4f} rad")

        # Publish pose to ROS
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.pose.position.x = robot_pose[1]
        pose_msg.pose.pose.position.y = -robot_pose[0]

        # Convert Euler angle to Quaternion
        qx, qy, qz, qw = quaternion_from_euler(0, 0, robot_angle)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw

        self.get_logger().info("Publishing robot pose...")
        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
