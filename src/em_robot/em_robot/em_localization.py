#!/usr/bin/env python3
import os
import math
import yaml
import cv2 as cv

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# TF
import tf2_ros
from geometry_msgs.msg import (
    TransformStamped,
    PoseWithCovarianceStamped
)
from tf_transformations import (
    quaternion_from_euler
)

# PiCamera2
from picamera2 import Picamera2
from libcamera import controls

# Your ArUco detection code
from .submodules.detect_aruco import CameraVisionStation, detector


def compose_2d(parent, child):
    """
    2D rigid transform composition.
    parent = (px, py, ptheta)
    child = (cx, cy, ctheta)
    Returns parent * child = (rx, ry, rtheta).
    """
    px, py, pth = parent
    cx, cy, cth = child

    rx = px + math.cos(pth)*cx - math.sin(pth)*cy
    ry = py + math.sin(pth)*cx + math.cos(pth)*cy
    rtheta = pth + cth
    return (rx, ry, rtheta)


def invert_2d(x, y, theta):
    """
    Invert a 2D transform (x, y, theta).
    The inverse is a translation in the opposite direction,
    rotated by -theta, plus a -theta for the rotation.
    """
    # Rotate and negate the translation:
    xr = -(x*math.cos(theta) + y*math.sin(theta))
    yr = -(-x*math.sin(theta) + y*math.cos(theta))
    tprime = -theta
    return (xr, yr, tprime)


class MultipleMarkersTFNode(Node):
    """
    A node that:
      1) Publishes a STATIC transform from 'map' to each ArUco marker (marker_<id>),
         based on (t_x, t_y, yaw) given in the YAML file.
      2) Publishes a STATIC transform 'base_link' -> 'camera'
         describing how the camera is physically mounted.
      3) Detects markers in the camera image.
      4) For each detected marker, publishes a DYNAMIC transform marker_<id> -> camera,
         and also computes and publishes map->base_link as a PoseWithCovarianceStamped.
    """
    def __init__(self):
        super().__init__('tf_localization_node')

        # --------------------------------------------------------
        # Load config from YAML
        # --------------------------------------------------------
        package_name = 'em_robot'
        config_filename = 'config/cam.yaml'
        pkg_share = get_package_share_directory(package_name)
        config_file_path = os.path.join(pkg_share, config_filename)

        self.get_logger().info(f"Loading configuration from {config_file_path}...")
        if not os.path.exists(config_file_path):
            self.get_logger().error(f"Configuration file not found: {config_file_path}")
            raise FileNotFoundError(f"Configuration file not found: {config_file_path}")

        with open(config_file_path, 'r') as file:
            full_config = yaml.safe_load(file)

        # Our node name in the YAML is 'localization_node'
        node_config = full_config.get(self.get_name(), {}).get('ros__parameters', {})
        cam_params = node_config.get('cam_params', {})
        self.aruco_params = node_config.get('aruco_params', {})

        # Camera resolution
        size_list = cam_params.get('size', [4608, 2592])
        self.size = tuple(size_list)

        # Camera lens/manual focus
        self.camera_height = cam_params.get('camera_height', 2.0)
        self.lens_position = 1.0 / float(self.camera_height)

        self.get_logger().info(f"Camera parameters: {cam_params}")
        self.get_logger().info(f"Aruco parameters: {self.aruco_params}")
        self.get_logger().info(f"Calculated lens position: {self.lens_position:.4f}")

        # --------------------------------------------------------
        # Initialize PiCamera2
        # --------------------------------------------------------
        self.get_logger().info("Initializing CameraVisionStation and PiCamera2...")
        self.cam = CameraVisionStation(cam_params=cam_params,
                                       aruco_params=self.aruco_params,
                                       cam_frame=self.size)

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

        # --------------------------------------------------------
        # TF Broadcasters
        # --------------------------------------------------------
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --------------------------------------------------------
        # 1) Publish all static transforms: map->marker_<id>
        # --------------------------------------------------------
        self.publish_all_static_map_to_marker()

        # --------------------------------------------------------
        # 2) Publish static transform: base_link -> camera
        #    We'll store its inverse as well, for computing base_link in map
        # --------------------------------------------------------
        self.base_link_to_camera = (0.198, 0.0, math.radians(90))  # (x, y, yaw)
        self.camera_to_base_link = invert_2d(*self.base_link_to_camera)
        self.publish_static_base_link_to_camera()

        # --------------------------------------------------------
        # 3) Publisher for final map->base_link pose
        # --------------------------------------------------------
        self.base_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/camera_base_pose',
            10
        )

        # --------------------------------------------------------
        # Timer for ArUco detection
        # --------------------------------------------------------
        timer_period = cam_params.get('timer_period', 0.5)
        self.timer = self.create_timer(timer_period, self.detect_and_broadcast)

        self.get_logger().info("MultipleMarkersTFNode is up and running!")

    def publish_all_static_map_to_marker(self):
        """
        For each marker ID in 'aruco_params', publish a static transform:
           map -> marker_<id>
        using (t_x, t_y, yaw) from the YAML.
        """
        transforms = []
        now = self.get_clock().now().to_msg()

        for marker_id_str, marker_info in self.aruco_params.items():
            t_x = marker_info.get('t_x', 0.0)
            t_y = marker_info.get('t_y', 0.0)
            yaw = marker_info.get('yaw', 0.0)

            # Build a static transform: map -> marker_<ID>
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.child_frame_id = f"marker_{marker_id_str}"

            t.transform.translation.x = float(t_x)
            t.transform.translation.y = float(t_y)
            t.transform.translation.z = 0.0

            q = quaternion_from_euler(0.0, 0.0, float(yaw))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            transforms.append(t)
            self.get_logger().info(
                f"map->marker_{marker_id_str} at x={t_x}, y={t_y}, yaw={yaw}"
            )

        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published all static transforms map->marker_<id>.")

    def publish_static_base_link_to_camera(self):
        """
        Publish a single static transform: base_link -> camera.

        We have: base_link->camera = (0.198 m forward, +90° about Z).
        """
        now = self.get_clock().now().to_msg()

        x_b, y_b, yaw_b = self.base_link_to_camera
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_link"
        t.child_frame_id = "camera"

        t.transform.translation.x = x_b
        t.transform.translation.y = y_b
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, yaw_b)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(
            "Published static transform base_link->camera at +0.198m, +90° about Z."
        )

    def detect_and_broadcast(self):
        """
        1. Capture image
        2. Detect any ArUco markers
        3. For each detected marker ID (that we have in aruco_params),
           - compute camera pose in that marker's frame,
           - publish dynamic transform marker_<id>->camera
           - also compute map->base_link by composing transforms
             (map->marker_<id>) * (marker_<id>->camera) * (camera->base_link)
             and publish as a PoseWithCovarianceStamped
        """
        self.get_logger().debug("Capturing new frame from the camera...")
        frame = self.picam2.capture_array()

        self.get_logger().debug(f"Converting to grayscale, frame size: {frame.shape}")
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        self.get_logger().debug("Detecting ArUco markers...")
        marker_corners, marker_ids, _ = detector.detectMarkers(gray_frame)

        if marker_ids is None:
            self.get_logger().debug("No markers found in this frame.")
            return  # no markers found

        self.get_logger().info(f"Detected {len(marker_ids)} marker(s): {marker_ids.flatten().tolist()}")

        for i, m_id in enumerate(marker_ids):
            marker_id_int = int(m_id[0])  # e.g. [645] -> 645
            marker_id_str = str(marker_id_int)

            # Skip markers not in our YAML
            if marker_id_str not in self.aruco_params:
                self.get_logger().debug(f"Marker {marker_id_int} not in aruco_params. Skipping.")
                continue

            self.get_logger().debug(f"Processing marker {marker_id_int}...")

            # This function returns camera pose in marker frame
            cam_pose_marker, cam_yaw_marker = self.cam.get_robot_pose(
                gray_frame, marker_corners, marker_ids
            )
            if cam_pose_marker is None:
                self.get_logger().debug(f"No valid pose returned for marker {marker_id_int}.")
                continue

            x_c = cam_pose_marker[0]
            y_c = cam_pose_marker[1]
            th_c = cam_yaw_marker

            self.get_logger().info(
                f"Camera pose relative to marker_{marker_id_str}: "
                f"x={x_c:.3f}, y={y_c:.3f}, yaw(deg)={math.degrees(th_c):.1f}"
            )

            # Publish the dynamic transform marker_<id> -> camera
            self.publish_marker_to_camera_transform(marker_id_str, x_c, y_c, th_c)

            # Now also compute map->base_link by chaining:
            # map->marker_<id> from YAML
            marker_info = self.aruco_params[marker_id_str]
            mx = marker_info.get('t_x', 0.0)
            my = marker_info.get('t_y', 0.0)
            mth = marker_info.get('yaw', 0.0)

            map_to_marker = (mx, my, mth)
            marker_to_camera = (x_c, y_c, th_c)

            # 1) map->camera
            map_to_camera = compose_2d(map_to_marker, marker_to_camera)
            self.get_logger().debug(
                f"map->camera from composition: x={map_to_camera[0]:.3f}, "
                f"y={map_to_camera[1]:.3f}, yaw(deg)={math.degrees(map_to_camera[2]):.1f}"
            )

            # 2) map->base_link = map->camera * camera->base_link
            map_to_base_link = compose_2d(map_to_camera, self.camera_to_base_link)
            bx, by, bth = map_to_base_link

            self.get_logger().info(
                f"map->base_link computed: x={bx:.3f}, y={by:.3f}, yaw(deg)={math.degrees(bth):.1f}"
            )

            # Publish as a PoseWithCovarianceStamped
            self.publish_base_link_pose_in_map(map_to_base_link)

    def publish_marker_to_camera_transform(self, marker_id_str, x_c, y_c, th_c):
        """
        Publish the DYNAMIC transform: marker_<id> -> camera
        specifying that "in marker_<id> frame, the camera is at (x_c, y_c, th_c)."
        """
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = f"marker_{marker_id_str}"
        t.child_frame_id = "camera"

        t.transform.translation.x = float(x_c)
        t.transform.translation.y = float(y_c)
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(th_c))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"Published dynamic TF: marker_{marker_id_str}->camera "
            f"(x={x_c:.3f}, y={y_c:.3f}, yaw={math.degrees(th_c):.1f} deg)"
        )

    def publish_base_link_pose_in_map(self, map_to_base_link):
        """
        Convert the final (x, y, yaw) for base_link in the map frame
        into a PoseWithCovarianceStamped and publish to /camera_base_pose
        so the EKF can fuse it.
        """
        bx, by, bth = map_to_base_link

        # Build the message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # 2D position => z=0
        pose_msg.pose.pose.position.x = bx
        pose_msg.pose.pose.position.y = by
        pose_msg.pose.pose.position.z = 0.0

        # 2D yaw => quaternion
        q = quaternion_from_euler(0.0, 0.0, bth)
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]

        # Simple constant covariance (adjust as needed)
        cov = [0.01]*36
        pose_msg.pose.covariance = cov

        # Publish
        self.base_pose_pub.publish(pose_msg)

        self.get_logger().info(
            f"Published PoseWithCovarianceStamped on /camera_base_pose: "
            f"x={bx:.3f}, y={by:.3f}, yaw(deg)={math.degrees(bth):.1f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultipleMarkersTFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TFLocalizationNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
