#!/usr/bin/env python3
# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math

from geometry_msgs.msg import Twist
import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


class ImuMockNode(Node):
    def __init__(self):
        super().__init__("imu_mock")

        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("yaw_rate", 0.0)
        self.declare_parameter("frame_id", "bno055")
        self.declare_parameter("follow_cmd_vel", False)

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.current_yaw_rate = float(self.get_parameter("yaw_rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.follow_cmd_vel = bool(self.get_parameter("follow_cmd_vel").value)

        self.publisher = self.create_publisher(Imu, "/bno055/imu", 10)
        self.cmd_vel_subscription = None
        self.last_time = self.get_clock().now()
        self.current_yaw = 0.0

        if self.follow_cmd_vel:
            self.cmd_vel_subscription = self.create_subscription(
                Twist,
                "/cmd_vel",
                self.cmd_vel_callback,
                10,
            )

        timer_period = 1.0 / self.rate_hz if self.rate_hz > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.publish_sample)
        if self.follow_cmd_vel:
            self.get_logger().info("IMU mock node started and following /cmd_vel angular.z")
        else:
            self.get_logger().info("IMU mock node started with fixed yaw_rate")

    def cmd_vel_callback(self, msg: Twist):
        self.current_yaw_rate = float(msg.angular.z)

    def publish_sample(self):
        now = self.get_clock().now()
        dt = max((now - self.last_time).nanoseconds / 1e9, 1e-6)
        self.current_yaw += self.current_yaw_rate * dt

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation = quaternion_from_yaw(self.current_yaw)
        msg.angular_velocity.z = self.current_yaw_rate

        msg.orientation_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.angular_velocity_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.linear_acceleration_covariance = [
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0,
        ]

        self.publisher.publish(msg)
        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ImuMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down IMU mock node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
