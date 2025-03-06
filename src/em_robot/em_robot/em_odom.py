#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import math


class OdomCorrectionNode(Node):
    """
    Corrects wheel odometry using camera pose estimates every 0.5s.
    - Subscribes to `odomWheel` (wheel odometry).
    - Subscribes to `camera_pose` (camera-based localization).
    - Computes `map → odom` correction every 0.5s.
    - Publishes TF for `map → odom`.
    """

    def __init__(self):
        super().__init__('odom_correction_node')
        self.get_logger().info("OdomCorrectionNode started")

        # Subscribe to odometry from wheels
        self.odom_sub = self.create_subscription(
            Odometry, 'odomWheel', self.odom_callback, 10)

        # Subscribe to camera localization data
        self.camera_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'camera_pose', self.camera_callback, 10)

        # Transform Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF Buffer & Listener to get odometry transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for last received values
        self.last_wheel_odom = None
        self.last_camera_pose = None

        # Timer to publish correction every 0.5s
        self.timer = self.create_timer(0.5, self.publish_correction_tf)

    def odom_callback(self, msg):
        """ Stores the latest wheel odometry data """
        self.last_wheel_odom = msg

    def camera_callback(self, msg):
        """ Stores the latest camera pose data """
        self.last_camera_pose = msg

    def publish_correction_tf(self):
        """
        Publishes `map → odom` transform based on the latest camera localization.
        This corrects odometry drift while ensuring smooth operation.
        """
        if self.last_camera_pose is None or self.last_wheel_odom is None:
            self.get_logger().warning("Waiting for both odometry and camera data...")
            return

        # Get Camera Pose in `map` frame
        cam_x = self.last_camera_pose.pose.pose.position.x
        cam_y = self.last_camera_pose.pose.pose.position.y

        # Get Wheel Odometry Pose in `odom` frame
        odom_x = self.last_wheel_odom.pose.pose.position.x
        odom_y = self.last_wheel_odom.pose.pose.position.y

        # Compute the correction needed (error between `map` and `odom`)
        correction_x = cam_x - odom_x
        correction_y = cam_y - odom_y

        # Get heading (yaw) from orientation quaternion (camera pose)
        q = self.last_camera_pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

        # Publish the correction transform `map → odom`
        correction_tf = TransformStamped()
        correction_tf.header.stamp = self.get_clock().now().to_msg()
        correction_tf.header.frame_id = "map"
        correction_tf.child_frame_id = "odom"

        correction_tf.transform.translation.x = correction_x
        correction_tf.transform.translation.y = correction_y
        correction_tf.transform.translation.z = 0.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        correction_tf.transform.rotation.x = 0.0
        correction_tf.transform.rotation.y = 0.0
        correction_tf.transform.rotation.z = qz
        correction_tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(correction_tf)

        self.get_logger().info(
            f"Published correction TF: map → odom (x={correction_x:.3f}, y={correction_y:.3f}, yaw={yaw:.3f} rad)")


def main(args=None):
    rclpy.init(args=args)
    node = OdomCorrectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down OdomCorrectionNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
