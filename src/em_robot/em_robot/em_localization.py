#!/usr/bin/env python3
import os
import yaml
import math
import cv2 as cv

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# TF
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

# PiCamera2
from picamera2 import Picamera2
from libcamera import controls

# Your ArUco detection code
from .submodules.detect_aruco import CameraVisionStation, detector


class MultipleMarkersTFNode(Node):
    """
    A node that:
      1) Publishes a STATIC transform from 'map' to each ArUco marker (marker_<id>),
         based on (t_x, t_y, yaw) given in the YAML file.
      2) Publishes a STATIC transform from base_link -> camera (the physical camera mount).
      3) Detects markers in the camera image.
      4) For each detected marker, publishes a DYNAMIC transform marker_<id> -> camera
         using the pose returned by get_robot_pose().
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
        # --------------------------------------------------------
        self.publish_static_base_link_to_camera()

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
            # marker_id_str might be "645", "11", etc.
            # Convert to int if needed:
            # marker_id = int(marker_id_str)
            # But we can just keep it as string for naming.

            t_x = marker_info.get('t_x', 0.0)
            t_y = marker_info.get('t_y', 0.0)
            yaw = marker_info.get('yaw', 0.0)

            # Build a static transform: map -> marker_<ID>
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = "map"
            t.child_frame_id = f"marker_{marker_id_str}"

            # translation
            t.transform.translation.x = float(t_x)
            t.transform.translation.y = float(t_y)
            t.transform.translation.z = 0.0

            # rotation about Z
            q = quaternion_from_euler(0.0, 0.0, float(yaw))
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            transforms.append(t)

            self.get_logger().info(
                f"map->marker_{marker_id_str} at x={t_x}, y={t_y}, yaw={yaw}"
            )

        # Publish them all at once
        self.static_broadcaster.sendTransform(transforms)
        self.get_logger().info("Published all static transforms map->marker_<id>.")

    def publish_static_base_link_to_camera(self):
        """
        Publish a single static transform: base_link -> camera.

        The camera is 0.198 m in front of the robot (along +X of base_link).
        The camera frame has:
          +X = left of the robot,
          +Y = back (opposite of robot's +X),
          +Z = up.

        => That's a +90° rotation about Z from base_link to camera frame.
        """
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_link"   # parent
        t.child_frame_id = "camera"       # child (or "camera_link")

        # Translation: 0.198 m forward along base_link +X
        t.transform.translation.x = 0.198
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        # Rotation: +90° around Z so that camera X=left, Y=back, Z=up
        roll = 0.0
        pitch = 0.0
        yaw = math.radians(90)  # 1.5708
        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish the static transform
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(
            "Published static transform base_link->camera at +0.198m, +90° about Z."
        )

    def detect_and_broadcast(self):
        """
        1. Capture image
        2. Detect any ArUco markers
        3. For each detected marker ID (that we have in aruco_params),
           compute 'camera pose in marker frame'
        4. Publish a dynamic transform marker_<id> -> camera
        """
        frame = self.picam2.capture_array()
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # If camera is physically upside down, you could rotate:
        # gray_frame = cv.rotate(gray_frame, cv.ROTATE_180)

        marker_corners, marker_ids, _ = detector.detectMarkers(gray_frame)
        if marker_ids is None:
            return  # no markers

        # marker_ids is a numpy array of shape (N,1) if multiple markers are detected
        for i, m_id in enumerate(marker_ids):
            marker_id_int = int(m_id[0])  # e.g. [645] -> 645
            marker_id_str = str(marker_id_int)

            # Only handle markers we have in aruco_params
            if marker_id_str not in self.aruco_params:
                continue

            # For each marker, we get the camera pose in that marker's frame.
            # If 'get_robot_pose()' is only for a single marker or lumps them all,
            # you may need to adapt it. Here we just assume it returns the correct
            # pose for the marker of interest.
            cam_pose_marker, cam_yaw_marker = self.cam.get_robot_pose(
                gray_frame, marker_corners, marker_ids
            )
            if cam_pose_marker is None:
                continue

            x_c = cam_pose_marker[0]
            y_c = cam_pose_marker[1]
            th_c = cam_yaw_marker

            # Publish marker_<id> -> camera
            self.publish_marker_to_camera_transform(marker_id_str, x_c, y_c, th_c)

    def publish_marker_to_camera_transform(self, marker_id_str, x_c, y_c, th_c):
        """
        Publish the DYNAMIC transform: marker_<id> -> camera
        specifying that "in marker_<id> frame, the camera is at (x_c, y_c, th_c)."
        """
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = f"marker_{marker_id_str}"
        t.child_frame_id = "camera"  # or "camera_link"

        t.transform.translation.x = float(x_c)
        t.transform.translation.y = float(y_c)
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(th_c))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish this dynamic transform
        self.tf_broadcaster.sendTransform(t)

        self.get_logger().info(
            f"marker_{marker_id_str}->camera published: "
            f"x={x_c:.2f}, y={y_c:.2f}, yaw(deg)={math.degrees(th_c):.1f}"
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
