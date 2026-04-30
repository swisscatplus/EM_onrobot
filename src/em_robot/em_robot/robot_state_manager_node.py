#!/usr/bin/env python3
# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA, String

from em_robot.robot_state_modes import (
    RobotSignalSnapshot,
    compute_health_mode,
    compute_mobility_mode,
    compute_overall_mode,
    health_led_color,
    mobility_led_color,
)


class RobotStateManagerNode(Node):
    def __init__(self):
        super().__init__("robot_state_manager")

        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("blink_period_s", 1.0)
        self.declare_parameter("cmd_vel_timeout", 0.25)
        self.declare_parameter("expected_odom_rate", 30.0)
        self.declare_parameter("expected_imu_rate", 30.0)
        self.declare_parameter("expected_filtered_odom_rate", 30.0)
        self.declare_parameter("moving_linear_threshold", 0.02)
        self.declare_parameter("moving_angular_threshold", 0.1)
        self.declare_parameter("filtered_odom_expected", True)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odomWheel")
        self.declare_parameter("imu_topic", "/bno055/imu")
        self.declare_parameter("filtered_odom_topic", "/odometry/filtered")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("front_led_topic", "/leds/front/color")
        self.declare_parameter("rear_led_topic", "/leds/rear/color")
        self.declare_parameter("mobility_state_topic", "/robot_state/mobility")
        self.declare_parameter("health_state_topic", "/robot_state/health")
        self.declare_parameter("overall_state_topic", "/robot_state/overall")

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.blink_period_s = float(self.get_parameter("blink_period_s").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.expected_odom_rate = float(self.get_parameter("expected_odom_rate").value)
        self.expected_imu_rate = float(self.get_parameter("expected_imu_rate").value)
        self.expected_filtered_odom_rate = float(
            self.get_parameter("expected_filtered_odom_rate").value
        )
        self.moving_linear_threshold = float(
            self.get_parameter("moving_linear_threshold").value
        )
        self.moving_angular_threshold = float(
            self.get_parameter("moving_angular_threshold").value
        )
        self.filtered_odom_expected = bool(
            self.get_parameter("filtered_odom_expected").value
        )

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        filtered_odom_topic = str(self.get_parameter("filtered_odom_topic").value)
        diagnostics_topic = str(self.get_parameter("diagnostics_topic").value)
        front_led_topic = str(self.get_parameter("front_led_topic").value)
        rear_led_topic = str(self.get_parameter("rear_led_topic").value)
        mobility_state_topic = str(self.get_parameter("mobility_state_topic").value)
        health_state_topic = str(self.get_parameter("health_state_topic").value)
        overall_state_topic = str(self.get_parameter("overall_state_topic").value)

        self.last_cmd_time = None
        self.last_nonzero_cmd_time = None
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_odom_time = None
        self.last_odom_linear = 0.0
        self.last_odom_angular = 0.0
        self.last_imu_time = None
        self.last_filtered_odom_time = None
        self.last_diagnostics_time = None
        self.diagnostic_levels = {}

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(
            Odometry,
            filtered_odom_topic,
            self.filtered_odom_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            diagnostics_topic,
            self.diagnostics_callback,
            10,
        )

        self.front_led_pub = self.create_publisher(ColorRGBA, front_led_topic, 10)
        self.rear_led_pub = self.create_publisher(ColorRGBA, rear_led_topic, 10)
        self.mobility_state_pub = self.create_publisher(String, mobility_state_topic, 10)
        self.health_state_pub = self.create_publisher(String, health_state_topic, 10)
        self.overall_state_pub = self.create_publisher(String, overall_state_topic, 10)

        timer_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.2
        self.timer = self.create_timer(timer_period, self.publish_state)

        self.last_mobility_mode = None
        self.last_health_mode = None
        self.last_overall_mode = None
        self.get_logger().info("Robot state manager started")

    def _age_seconds(self, last_time):
        if last_time is None:
            return None
        return (self.get_clock().now() - last_time).nanoseconds / 1e9

    def _blink_on(self) -> bool:
        if self.blink_period_s <= 0.0:
            return True
        current_time_s = self.get_clock().now().nanoseconds / 1e9
        phase = math.fmod(current_time_s, self.blink_period_s)
        return phase < (self.blink_period_s / 2.0)

    def _max_non_motion_diagnostic_level(self):
        relevant_levels = []
        for name, level in self.diagnostic_levels.items():
            if name == "em_robot/cmd_vel":
                continue
            if name.startswith("em_robot/leds/"):
                continue
            relevant_levels.append(level)

        if not relevant_levels:
            return None

        return max(relevant_levels)

    def _publish_string(self, publisher, value: str):
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def _publish_color(self, publisher, color: tuple[float, float, float]):
        msg = ColorRGBA()
        msg.r = float(color[0])
        msg.g = float(color[1])
        msg.b = float(color[2])
        msg.a = 1.0
        publisher.publish(msg)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_linear = float(msg.linear.x)
        self.last_cmd_angular = float(msg.angular.z)

        if (
            abs(self.last_cmd_linear) >= self.moving_linear_threshold
            or abs(self.last_cmd_angular) >= self.moving_angular_threshold
        ):
            self.last_nonzero_cmd_time = self.last_cmd_time

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = self.get_clock().now()
        self.last_odom_linear = float(msg.twist.twist.linear.x)
        self.last_odom_angular = float(msg.twist.twist.angular.z)

    def imu_callback(self, _msg: Imu):
        self.last_imu_time = self.get_clock().now()

    def filtered_odom_callback(self, _msg: Odometry):
        self.last_filtered_odom_time = self.get_clock().now()

    def diagnostics_callback(self, msg: DiagnosticArray):
        self.last_diagnostics_time = self.get_clock().now()
        for status in msg.status:
            self.diagnostic_levels[status.name] = int(status.level)

    def publish_state(self):
        snapshot = RobotSignalSnapshot(
            cmd_vel_age_s=self._age_seconds(self.last_cmd_time),
            last_nonzero_cmd_age_s=self._age_seconds(self.last_nonzero_cmd_time),
            cmd_linear=self.last_cmd_linear,
            cmd_angular=self.last_cmd_angular,
            odom_age_s=self._age_seconds(self.last_odom_time),
            odom_linear=self.last_odom_linear,
            odom_angular=self.last_odom_angular,
            imu_age_s=self._age_seconds(self.last_imu_time),
            filtered_odom_age_s=self._age_seconds(self.last_filtered_odom_time),
            diagnostics_age_s=self._age_seconds(self.last_diagnostics_time),
            max_non_motion_diagnostic_level=self._max_non_motion_diagnostic_level(),
            cmd_vel_timeout=self.cmd_vel_timeout,
            expected_odom_rate=self.expected_odom_rate,
            expected_imu_rate=self.expected_imu_rate,
            expected_filtered_odom_rate=self.expected_filtered_odom_rate,
            moving_linear_threshold=self.moving_linear_threshold,
            moving_angular_threshold=self.moving_angular_threshold,
            filtered_odom_expected=self.filtered_odom_expected,
        )

        mobility_mode = compute_mobility_mode(snapshot)
        health_mode = compute_health_mode(snapshot)
        overall_mode = compute_overall_mode(mobility_mode, health_mode)
        blink_on = self._blink_on()

        self._publish_string(self.mobility_state_pub, mobility_mode)
        self._publish_string(self.health_state_pub, health_mode)
        self._publish_string(self.overall_state_pub, overall_mode)
        self._publish_color(self.front_led_pub, mobility_led_color(mobility_mode, blink_on))
        self._publish_color(self.rear_led_pub, health_led_color(health_mode, blink_on))

        if (
            mobility_mode != self.last_mobility_mode
            or health_mode != self.last_health_mode
            or overall_mode != self.last_overall_mode
        ):
            self.get_logger().info(
                "Robot state updated: "
                f"mobility={mobility_mode}, health={health_mode}, overall={overall_mode}"
            )
            self.last_mobility_mode = mobility_mode
            self.last_health_mode = health_mode
            self.last_overall_mode = overall_mode


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down robot state manager...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
