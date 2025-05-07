#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node

import numpy as np
import cv2 as cv

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
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

from em_robot_srv.srv import SetInitialPose

# --- ArUco Detection Setup ---
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
        c = corners[0]
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

        self.last_transform = None

        cammat_list = config.get('camera_matrix', [])
        dist_list = config.get('dist_coeff', [])
        if isinstance(dist_list, str):
            dist_list = [float(x) for x in dist_list.strip().split()]

        self.camera_matrix = np.array(cammat_list, dtype=np.float32)
        self.dist_coeffs = np.array(dist_list, dtype=np.float32)
        self.camera_height = config.get('camera_height', 0.337) #375
        self.marker_size = config.get('marker_size', 0.038)

        # Init camera
        self.size = (1536, 864)
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()
        self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 8.0})

        # ROS setup
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, 'aruco_markers_pose', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.latest_odom_pose = None

        self.last_map_to_odom = None
        self.last_marker_time = self.get_clock().now()
        self.publish_initial_map_to_odom()
        self.map_odom_timer = self.create_timer(0.2, self.broadcast_last_map_to_odom)  # 5 Hz

        self.publish_static_transform()
        self.timer = self.create_timer(1/5, self.process_frame)
        self.get_logger().info("MarkerLocalizationNode: started.")

        self.set_pose_srv = self.create_service(SetInitialPose, 'set_initial_pose', self.handle_set_initial_pose)

    def odom_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        pose_matrix = concatenate_matrices(
            translation_matrix([position.x, position.y, position.z]),
            quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        )
        self.latest_odom_pose = pose_matrix

    def publish_static_transform(self):
        static_transform_cam = TransformStamped()
        static_transform_cam.header.stamp = self.get_clock().now().to_msg()
        static_transform_cam.header.frame_id = "camera_frame"
        static_transform_cam.child_frame_id = "cam_base_link"
        static_transform_cam.transform.translation.x = -0.192
        static_transform_cam.transform.translation.y = 0.0
        static_transform_cam.transform.translation.z = 0.0
        static_transform_cam.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform([static_transform_cam])
        self.get_logger().info("Published static transform: camera_frame → cam_base_link")

    def publish_initial_map_to_odom(self):
        """Broadcast identity transform from map to odom at startup"""
        self.get_logger().warn("Publishing initial static map → odom transform at (0, 0, 0)")

        quat = quaternion_from_euler(0.0, 0.0, 0.5*3.1415)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.last_map_to_odom = t
        self.tf_broadcaster.sendTransform(t)

    def handle_set_initial_pose(self, request, response):
        quat = quaternion_from_euler(0.0, 0.0, request.yaw)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = request.x
        t.transform.translation.y = request.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]


        self.last_map_to_odom = t
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Manually set map → odom to x={request.x}, y={request.y}, yaw={request.yaw}")
        response.success = True
        return response

    def broadcast_last_map_to_odom(self):
        if self.last_map_to_odom:
            now = self.get_clock().now()
            self.last_map_to_odom.header.stamp = now.to_msg()
            self.tf_broadcaster.sendTransform(self.last_map_to_odom)

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        detections = None # uncomment when you want to remove camera
        if not detections:

            #self.get_logger().info("No markers detected.")
            return

        for marker in detections:
            marker_id = marker['id']
            #self.get_logger().info(f"Checking marker ID {marker_id}...")
            corners = marker['corners'].astype(np.float32)

            undistorted = cv.undistortPoints(
                np.expand_dims(corners, axis=1),
                self.camera_matrix,
                self.dist_coeffs,
                P=self.camera_matrix
            ).reshape(-1, 2)

            cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
            fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
            marker_center = np.mean(undistorted, axis=0)
            offset = np.array([cx, cy]) - marker_center
            x_cam = self.camera_height * offset[0] / fx
            y_cam = self.camera_height * offset[1] / fy

            top_center = np.mean(undistorted[0:2], axis=0)
            bottom_center = np.mean(undistorted[2:4], axis=0)
            dx = top_center[0] - bottom_center[0]
            dy = top_center[1] - bottom_center[1]
            yaw = np.arctan2(dx, -dy)

            quat = quaternion_from_euler(0.0, 0.0, yaw)
            T_marker_cam = concatenate_matrices(
                translation_matrix([x_cam, y_cam, 0.0]),
                quaternion_matrix(quat)
            )
            T_aruco_camera = np.linalg.inv(T_marker_cam)
            trans = translation_from_matrix(T_aruco_camera)
            quat_inv = quaternion_from_matrix(T_aruco_camera)

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
            self.last_transform = t_camera_marker

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

            aruco_frame = f"aruco_{marker_id}"
            try:
                tf_map_to_aruco = self.tf_buffer.lookup_transform(
                    "map", aruco_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
                )
            except Exception as e:
                #self.get_logger().warn(f"Skipping marker {marker_id}: map → {aruco_frame} TF unavailable: {e}")
                continue

            T_map_aruco = concatenate_matrices(
                translation_matrix([
                    tf_map_to_aruco.transform.translation.x,
                    tf_map_to_aruco.transform.translation.y,
                    tf_map_to_aruco.transform.translation.z
                ]),
                quaternion_matrix([
                    tf_map_to_aruco.transform.rotation.x,
                    tf_map_to_aruco.transform.rotation.y,
                    tf_map_to_aruco.transform.rotation.z,
                    tf_map_to_aruco.transform.rotation.w
                ])
            )

            T_camera_cam_base = translation_matrix([-0.192, 0.0, 0.0])
            T_map_cam_base = T_map_aruco @ T_aruco_camera @ T_camera_cam_base

            if self.latest_odom_pose is None:
                self.get_logger().warn("No EKF pose available. Skipping map->odom update.")
                return

            T_odom_base = self.latest_odom_pose
            T_map_odom = T_map_cam_base @ np.linalg.inv(T_odom_base)

            trans_map_odom = translation_from_matrix(T_map_odom)
            quat_map_odom = quaternion_from_matrix(T_map_odom)

            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = self.get_clock().now().to_msg()
            t_map_odom.header.frame_id = "map"
            t_map_odom.child_frame_id = "odom"
            t_map_odom.transform.translation.x = trans_map_odom[0]
            t_map_odom.transform.translation.y = trans_map_odom[1]
            t_map_odom.transform.translation.z = 0.0
            t_map_odom.transform.rotation.x = quat_map_odom[0]
            t_map_odom.transform.rotation.y = quat_map_odom[1]
            t_map_odom.transform.rotation.z = quat_map_odom[2]
            t_map_odom.transform.rotation.w = quat_map_odom[3]

            self.last_map_to_odom = t_map_odom
            self.last_marker_time = self.get_clock().now()
            #self.get_logger().info(f"Updated map->odom using marker {marker_id}")
            break

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
