#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from dynamixel_sdk import *
from tf_transformations import euler_from_quaternion
import math

# --- Constants ---
WHEEL_RADIUS = 0.035  # 35 mm = 0.035 m
WHEEL_BASE = 0.131    # 130 mm = 0.130 m
ENCODER_RESOLUTION = 4096

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132

# Motor IDs and port settings
DXL_ID_1 = 2  # Right
DXL_ID_2 = 1  # Left
BAUDRATE = 57600
#DEVICENAME = '/dev/dynamixel'
DEVICENAME = '/dev/ttyAMA10'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.get_logger().info('MovementNode started')

        # Declare and get parameters
        self.declare_parameter("max_speed", 1000.0)
        self.declare_parameter("odom_rate", 30.0)
        self.max_speed = self.get_parameter("max_speed").value
        self.odom_rate = self.get_parameter("odom_rate").value
        self.max_pos_step = self.max_speed / self.odom_rate

        # Initialize Dynamixel communication
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")
            return
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return

        for dxl_id in [DXL_ID_1, DXL_ID_2]:
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(f"Torque enable failed on ID={dxl_id}")
            else:
                self.get_logger().info(f"Torque enabled on ID={dxl_id}")

        # ROS setup
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odomWheel', 10)
        self.odom_timer = self.create_timer(1.0 / self.odom_rate, self.odom_callback)

        # State tracking
        self.prev_position_r = None
        self.prev_position_l = None
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def convert_to_signed(self, val):
        return val - 4294967296 if val > 2147483647 else val

    def cmd_vel_callback(self, msg: Twist):
        v = max(min(msg.linear.x, self.max_speed), -self.max_speed)
        w = msg.angular.z

        motor_speed_r = int((v + (w * WHEEL_BASE / 2)) * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi))
        motor_speed_l = int((v - (w * WHEEL_BASE / 2)) * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi))

        motor_speed_r = max(min(motor_speed_r, 410), -410)
        motor_speed_l = max(min(motor_speed_l, 410), -410)

        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_GOAL_VELOCITY, motor_speed_r)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_GOAL_VELOCITY, motor_speed_l)

    def odom_callback(self):
        now = self.get_clock().now()

        current_position_r, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_PRESENT_POSITION)
        current_position_l, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_PRESENT_POSITION)

        current_position_r = self.convert_to_signed(current_position_r)
        current_position_l = self.convert_to_signed(current_position_l)

        if self.prev_position_r is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            self.prev_time = now
            return

        delta_r = current_position_r - self.prev_position_r
        delta_l = current_position_l - self.prev_position_l

        if delta_r > ENCODER_RESOLUTION / 2:
            delta_r -= ENCODER_RESOLUTION
        elif delta_r < -ENCODER_RESOLUTION / 2:
            delta_r += ENCODER_RESOLUTION
        if delta_l > ENCODER_RESOLUTION / 2:
            delta_l -= ENCODER_RESOLUTION
        elif delta_l < -ENCODER_RESOLUTION / 2:
            delta_l += ENCODER_RESOLUTION

        rad_r = delta_r * (2.0 * math.pi / ENCODER_RESOLUTION)
        rad_l = delta_l * (2.0 * math.pi / ENCODER_RESOLUTION)

        d_r = rad_r * WHEEL_RADIUS
        d_l = rad_l * WHEEL_RADIUS

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1e-6

        d = (d_r + d_l) / 2.0
        dtheta = (d_r - d_l) / WHEEL_BASE

        vx = d / dt
        vth = dtheta / dt

        # Clamp velocity
        if abs(vx) > self.max_speed:
            self.get_logger().warn(f"Clamping vx {vx:.3f} to max {self.max_speed}")
            vx = math.copysign(self.max_speed, vx)

        # Clamp positional step
        if abs(d) > self.max_pos_step:
            self.get_logger().warn(f"Clamping d {d:.4f} to max step {self.max_pos_step:.4f}")
            d = math.copysign(self.max_pos_step, d)
            vx = d / dt

        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quat_z = math.sin(self.theta / 2.0)
        quat_w = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = quat_z
        odom_msg.pose.pose.orientation.w = quat_w

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        odom_msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
        ]

        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom_msg)

        self.prev_position_r = current_position_r
        self.prev_position_l = current_position_l
        self.prev_time = now

    def stop_motors(self):
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_GOAL_VELOCITY, 0)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_GOAL_VELOCITY, 0)


def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()
