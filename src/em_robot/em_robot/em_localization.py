import os
import yaml
import math
import numpy as np
import cv2 as cv

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
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

        pkg_path = get_package_share_directory('em_robot')
        config_path = os.path.join(pkg_path, 'config', 'calibration.yaml')
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.camera_matrix = np.array(config['camera_matrix'], dtype=np.float32)
        self.dist_coeffs = np.array(config['dist_coeff'], dtype=np.float32)
        self.camera_height = config.get('camera_height', 0.381)

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

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publish_static_transform()

        self.last_marker_id = None
        self.last_camera_in_aruco = None  # 4x4 matrix
        self.last_camera_in_odom = None  # 4x4 matrix

        self.timer = self.create_timer(1 / 5.0, self.process_frame)
        self.get_logger().info("MarkerLocalizationNode started.")

    def publish_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_frame"
        t.child_frame_id = "base_link"
        t.transform.translation.x = -0.192
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.static_tf_broadcaster.sendTransform(t)

    def get_tf_matrix(self, source, target):
        try:
            tf = self.tf_buffer.lookup_transform(source, target, rclpy.time.Time())
            trans = tf.transform.translation
            rot = tf.transform.rotation
            trans_mat = translation_matrix([trans.x, trans.y, trans.z])
            quat_mat = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            return concatenate_matrices(trans_mat, quat_mat)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def process_frame(self):
        frame = self.picam2.capture_array()
        detections = detect_aruco_corners(frame)

        if detections:
            marker = detections[0]
            marker_id = marker['id']
            corners = marker['corners']

            undistorted = cv.undistortPoints(
                np.expand_dims(corners, axis=1),
                self.camera_matrix,
                self.dist_coeffs,
                P=self.camera_matrix
            ).reshape(-1, 2)

            center = np.mean(corners, axis=0)
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]

            offset = np.array([cx, cy]) - center
            x_cam = self.camera_height * offset[0] / (fx * 2)
            y_cam = self.camera_height * offset[1] / (fy * 2)
            z_cam = 0.0

            top = np.mean(corners[0:2], axis=0)
            bottom = np.mean(corners[2:4], axis=0)
            dx = top[0] - bottom[0]
            dy = top[1] - bottom[1]
            yaw = math.atan2(dx, -dy)

            quat = quaternion_from_euler(0.0, 0.0, yaw)
            T_cam_marker = concatenate_matrices(
                translation_matrix([x_cam, y_cam, z_cam]),
                quaternion_matrix(quat)
            )
            T_marker_cam = np.linalg.inv(T_cam_marker)

            self.last_marker_id = marker_id
            self.last_camera_in_aruco = T_marker_cam
            self.last_camera_in_odom = self.get_tf_matrix("odom", "camera_frame")

            self.publish_camera_transform(T_marker_cam, marker_id)

        elif self.last_camera_in_aruco is not None and self.last_camera_in_odom is not None:
            current_camera_in_odom = self.get_tf_matrix("odom", "camera_frame")
            if current_camera_in_odom is not None:
                delta = np.dot(current_camera_in_odom, np.linalg.inv(self.last_camera_in_odom))
                updated_T = np.dot(self.last_camera_in_aruco, delta)
                self.last_camera_in_aruco = updated_T
                self.last_camera_in_odom = current_camera_in_odom
                self.publish_camera_transform(updated_T, self.last_marker_id)

    def publish_camera_transform(self, T, marker_id):
        trans = translation_from_matrix(T)
        quat = quaternion_from_matrix(T)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = f"aruco_{marker_id}"
        t.child_frame_id = "camera_frame"
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
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
