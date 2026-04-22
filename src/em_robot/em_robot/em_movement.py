#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from em_robot.movement_core import (
    PoseState,
    cmd_vel_to_motor_speeds,
    convert_to_signed,
    integrate_fake_motion,
    integrate_wheel_odometry,
)

try:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except ImportError:  # pragma: no cover - depends on runtime image
    COMM_SUCCESS = None
    PacketHandler = None
    PortHandler = None


ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1


class MovementNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.get_logger().info("MovementNode started")

        self.declare_parameter("backend", "real")
        self.declare_parameter("max_speed", 1000.0)
        self.declare_parameter("odom_rate", 30.0)
        self.declare_parameter("device_name", "/dev/dynamixel")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("right_motor_id", 2)
        self.declare_parameter("left_motor_id", 1)

        self.backend = self.get_parameter("backend").value
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.odom_rate = float(self.get_parameter("odom_rate").value)
        self.device_name = self.get_parameter("device_name").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.right_motor_id = int(self.get_parameter("right_motor_id").value)
        self.left_motor_id = int(self.get_parameter("left_motor_id").value)
        self.max_pos_step = self.max_speed / self.odom_rate

        self.subscription = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odomWheel", 10)
        self.odom_timer = self.create_timer(1.0 / self.odom_rate, self.odom_callback)

        self.pose_state = PoseState()
        self.prev_position_r = None
        self.prev_position_l = None
        self.prev_time = self.get_clock().now()
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.port_handler = None
        self.packet_handler = None

        if self.backend == "real":
            self._setup_real_backend()
        elif self.backend == "fake":
            self.get_logger().info("Using fake movement backend")
        else:
            raise ValueError(f"Unsupported movement backend: {self.backend}")

    def _setup_real_backend(self):
        if PortHandler is None or PacketHandler is None:
            raise RuntimeError("dynamixel_sdk is not available for the real movement backend")

        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(2.0)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {self.device_name}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(f"Failed to set Dynamixel baudrate: {self.baudrate}")

        for motor_id in [self.right_motor_id, self.left_motor_id]:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if result != COMM_SUCCESS or error != 0:
                raise RuntimeError(f"Torque enable failed on motor ID={motor_id}")

        self.get_logger().info(f"Using real movement backend on {self.device_name}")

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_linear = float(msg.linear.x)
        self.last_cmd_angular = float(msg.angular.z)

        if self.backend != "real":
            return

        motor_speed_r, motor_speed_l = cmd_vel_to_motor_speeds(
            linear_x=self.last_cmd_linear,
            angular_z=self.last_cmd_angular,
            max_speed=self.max_speed,
        )

        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.right_motor_id, ADDR_GOAL_VELOCITY, motor_speed_r
        )
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.left_motor_id, ADDR_GOAL_VELOCITY, motor_speed_l
        )

    def odom_callback(self):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds / 1e9

        if self.backend == "real":
            odom_data = self._compute_real_odom(dt)
        else:
            odom_data = integrate_fake_motion(
                linear_x=self.last_cmd_linear,
                angular_z=self.last_cmd_angular,
                dt=dt,
                pose_state=self.pose_state,
                max_speed=self.max_speed,
                max_pos_step=self.max_pos_step,
            )

        if odom_data is None:
            self.prev_time = now
            return

        self.publish_odom(now, odom_data["vx"], odom_data["vth"])
        self.prev_time = now

    def _compute_real_odom(self, dt):
        current_position_r, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.right_motor_id, ADDR_PRESENT_POSITION
        )
        current_position_l, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.left_motor_id, ADDR_PRESENT_POSITION
        )

        current_position_r = convert_to_signed(current_position_r)
        current_position_l = convert_to_signed(current_position_l)

        if self.prev_position_r is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            return None

        odom_data = integrate_wheel_odometry(
            delta_r_ticks=current_position_r - self.prev_position_r,
            delta_l_ticks=current_position_l - self.prev_position_l,
            dt=dt,
            pose_state=self.pose_state,
            max_speed=self.max_speed,
            max_pos_step=self.max_pos_step,
        )

        self.prev_position_r = current_position_r
        self.prev_position_l = current_position_l
        return odom_data

    def publish_odom(self, now, vx, vth):
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.pose_state.x
        odom_msg.pose.pose.position.y = self.pose_state.y
        odom_msg.pose.pose.orientation.z = math.sin(self.pose_state.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.pose_state.theta / 2.0)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
        ]
        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]

        self.odom_pub.publish(odom_msg)

    def stop_motors(self):
        if self.backend != "real" or self.packet_handler is None or self.port_handler is None:
            return

        self.packet_handler.write4ByteTxRx(self.port_handler, self.right_motor_id, ADDR_GOAL_VELOCITY, 0)
        self.packet_handler.write4ByteTxRx(self.port_handler, self.left_motor_id, ADDR_GOAL_VELOCITY, 0)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down movement node...")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()
