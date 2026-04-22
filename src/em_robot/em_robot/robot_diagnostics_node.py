#!/usr/bin/env python3
import json

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from em_robot.diagnostic_utils import build_diagnostic_array, build_diagnostic_status


class RobotDiagnosticsNode(Node):
    def __init__(self):
        super().__init__("robot_diagnostics")

        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("cmd_vel_timeout", 0.25)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odomWheel")
        self.declare_parameter("imu_topic", "/bno055/imu")
        self.declare_parameter("filtered_odom_topic", "/odometry/filtered")
        self.declare_parameter("calib_status_topic", "/bno055/calib_status")
        self.declare_parameter("localization_expected", False)
        self.declare_parameter("filtered_odom_expected", True)
        self.declare_parameter("expected_odom_rate", 30.0)
        self.declare_parameter("expected_imu_rate", 30.0)
        self.declare_parameter("expected_filtered_odom_rate", 30.0)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.localization_expected = bool(self.get_parameter("localization_expected").value)
        self.filtered_odom_expected = bool(self.get_parameter("filtered_odom_expected").value)
        self.expected_odom_rate = float(self.get_parameter("expected_odom_rate").value)
        self.expected_imu_rate = float(self.get_parameter("expected_imu_rate").value)
        self.expected_filtered_odom_rate = float(
            self.get_parameter("expected_filtered_odom_rate").value
        )

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        filtered_odom_topic = str(self.get_parameter("filtered_odom_topic").value)
        calib_status_topic = str(self.get_parameter("calib_status_topic").value)

        self.last_cmd_vel_time = None
        self.last_cmd_vel = None
        self.last_odom_time = None
        self.last_odom = None
        self.last_imu_time = None
        self.last_imu = None
        self.last_filtered_odom_time = None
        self.last_filtered_odom = None
        self.last_calib_status = None

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        self.create_subscription(
            Odometry,
            filtered_odom_topic,
            self.filtered_odom_callback,
            10,
        )
        self.create_subscription(String, calib_status_topic, self.calib_status_callback, 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        timer_period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 1.0
        self.timer = self.create_timer(timer_period, self.publish_diagnostics)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_vel_time = self.get_clock().now()
        self.last_cmd_vel = msg

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = self.get_clock().now()
        self.last_odom = msg

    def imu_callback(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        self.last_imu = msg

    def filtered_odom_callback(self, msg: Odometry):
        self.last_filtered_odom_time = self.get_clock().now()
        self.last_filtered_odom = msg

    def calib_status_callback(self, msg: String):
        try:
            self.last_calib_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.last_calib_status = {"raw": msg.data}

    def _age_seconds(self, last_time):
        if last_time is None:
            return None
        return (self.get_clock().now() - last_time).nanoseconds / 1e9

    def _freshness_timeout(self, expected_rate_hz):
        if expected_rate_hz <= 0.0:
            return 1.0
        return max(0.5, 3.0 / expected_rate_hz)

    def _build_cmd_vel_status(self):
        age = self._age_seconds(self.last_cmd_vel_time)
        if age is None:
            return build_diagnostic_status(
                "em_robot/cmd_vel",
                DiagnosticStatus.WARN,
                "Waiting for first cmd_vel",
                values={"timeout_s": self.cmd_vel_timeout},
            )

        level = DiagnosticStatus.OK
        message = "cmd_vel is fresh"
        if self.cmd_vel_timeout > 0.0 and age > self.cmd_vel_timeout:
            level = DiagnosticStatus.WARN
            message = "cmd_vel is stale; base should be watchdog-stopped"

        values = {
            "age_s": f"{age:.3f}",
            "timeout_s": self.cmd_vel_timeout,
        }
        if self.last_cmd_vel is not None:
            values["linear_x"] = f"{self.last_cmd_vel.linear.x:.3f}"
            values["angular_z"] = f"{self.last_cmd_vel.angular.z:.3f}"
        return build_diagnostic_status("em_robot/cmd_vel", level, message, values=values)

    def _build_odom_status(self):
        age = self._age_seconds(self.last_odom_time)
        timeout = self._freshness_timeout(self.expected_odom_rate)
        if age is None:
            return build_diagnostic_status(
                "em_robot/odom_wheel",
                DiagnosticStatus.WARN,
                "Waiting for wheel odometry",
                values={"timeout_s": f"{timeout:.3f}"},
            )

        level = DiagnosticStatus.OK if age <= timeout else DiagnosticStatus.WARN
        message = "Wheel odometry is fresh" if level == DiagnosticStatus.OK else "Wheel odometry is stale"
        values = {"age_s": f"{age:.3f}", "timeout_s": f"{timeout:.3f}"}
        if self.last_odom is not None:
            values["linear_x"] = f"{self.last_odom.twist.twist.linear.x:.3f}"
            values["angular_z"] = f"{self.last_odom.twist.twist.angular.z:.3f}"
        return build_diagnostic_status("em_robot/odom_wheel", level, message, values=values)

    def _build_imu_status(self):
        age = self._age_seconds(self.last_imu_time)
        timeout = self._freshness_timeout(self.expected_imu_rate)
        if age is None:
            return build_diagnostic_status(
                "em_robot/imu",
                DiagnosticStatus.WARN,
                "Waiting for IMU data",
                values={"timeout_s": f"{timeout:.3f}"},
            )

        level = DiagnosticStatus.OK if age <= timeout else DiagnosticStatus.WARN
        message = "IMU is fresh" if level == DiagnosticStatus.OK else "IMU data is stale"
        values = {"age_s": f"{age:.3f}", "timeout_s": f"{timeout:.3f}"}
        if self.last_imu is not None:
            values["frame_id"] = self.last_imu.header.frame_id
            values["angular_z"] = f"{self.last_imu.angular_velocity.z:.3f}"
        if self.last_calib_status:
            for key, value in self.last_calib_status.items():
                values[f"calib_{key}"] = value
        return build_diagnostic_status("em_robot/imu", level, message, values=values)

    def _build_filtered_odom_status(self):
        if not self.filtered_odom_expected:
            return build_diagnostic_status(
                "em_robot/odometry_filtered",
                DiagnosticStatus.OK,
                "Filtered odometry is disabled by profile",
            )

        age = self._age_seconds(self.last_filtered_odom_time)
        timeout = self._freshness_timeout(self.expected_filtered_odom_rate)
        if age is None:
            level = DiagnosticStatus.WARN if self.localization_expected else DiagnosticStatus.WARN
            return build_diagnostic_status(
                "em_robot/odometry_filtered",
                level,
                "Waiting for filtered odometry",
                values={"timeout_s": f"{timeout:.3f}"},
            )

        level = DiagnosticStatus.OK if age <= timeout else DiagnosticStatus.WARN
        message = (
            "Filtered odometry is fresh"
            if level == DiagnosticStatus.OK
            else "Filtered odometry is stale"
        )
        values = {"age_s": f"{age:.3f}", "timeout_s": f"{timeout:.3f}"}
        if self.last_filtered_odom is not None:
            values["linear_x"] = f"{self.last_filtered_odom.twist.twist.linear.x:.3f}"
            values["angular_z"] = f"{self.last_filtered_odom.twist.twist.angular.z:.3f}"
        return build_diagnostic_status(
            "em_robot/odometry_filtered",
            level,
            message,
            values=values,
        )

    def publish_diagnostics(self):
        statuses = [
            self._build_cmd_vel_status(),
            self._build_odom_status(),
            self._build_imu_status(),
            self._build_filtered_odom_status(),
        ]
        self.diagnostics_pub.publish(
            build_diagnostic_array(statuses, self.get_clock().now().to_msg())
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotDiagnosticsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down robot diagnostics...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
