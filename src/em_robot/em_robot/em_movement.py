#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamixel_sdk import *  # Import DYNAMIXEL SDK library
import math

# --- Constants ---
WHEEL_RADIUS = 0.035  # 35 mm = 0.035 m
WHEEL_BASE = 0.130  # 129 mm = 0.129 m
ENCODER_RESOLUTION = 4096  # Ticks per revolution for X-Series

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132

# Default settings
DXL_ID_1 = 1  # Right wheel motor ID
DXL_ID_2 = 2  # Left wheel motor ID
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

# Torque control
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.get_logger().info('MovementNode started')

        # Initialize PortHandler/PacketHandler
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(2.0)

        # Open Port
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")
            return
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return

        # Enable torque on both motors
        for dxl_id in [DXL_ID_1, DXL_ID_2]:
            result, error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(f"Failed to enable torque on Dynamixel ID={dxl_id}")
            else:
                self.get_logger().info(f"Torque enabled on Dynamixel ID={dxl_id}")

        # Subscribers and Publishers
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odomWheel', 10)
        self.odom_timer = self.create_timer(0.1, self.odom_callback)  # 10 Hz

        # Odometry state variables
        self.prev_position_r = None
        self.prev_position_l = None
        self.prev_time = self.get_clock().now()
        self.x, self.y, self.theta = 0.0, 0.0, 0.0

    def cmd_vel_callback(self, msg: Twist):
        """ Convert Twist message into motor speed commands """
        linear_x, angular_z = msg.linear.x, msg.angular.z
        self.get_logger().info(f"Received cmd_vel: linear_x={linear_x}, angular_z={angular_z}")

        # Convert (v, w) to wheel speeds
        motor_speed_r = int((-linear_x - (angular_z * WHEEL_BASE / 2)) * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi))
        motor_speed_l = int((linear_x - (angular_z * WHEEL_BASE / 2)) * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi))

        # Limit the motor speeds
        motor_speed_r = max(min(motor_speed_r, 410), -410)
        motor_speed_l = max(min(motor_speed_l, 410), -410)

        self.get_logger().info(f"Sending speeds: Right={motor_speed_r}, Left={motor_speed_l}")

        # Send speeds to motors
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_GOAL_VELOCITY, motor_speed_r)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_GOAL_VELOCITY, motor_speed_l)

    def odom_callback(self):
        """ Read encoder positions and compute odometry """
        now = self.get_clock().now()
        self.get_logger().debug("Reading encoder positions...")

        # Read encoder positions
        current_position_r, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_PRESENT_POSITION)
        current_position_l, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_PRESENT_POSITION)

        # Initialize first reading
        if self.prev_position_r is None or self.prev_position_l is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            self.prev_time = now
            return

        # Compute delta positions (handle wraparounds)
        delta_r = (current_position_r - self.prev_position_r) % ENCODER_RESOLUTION
        delta_l = (current_position_l - self.prev_position_l) % ENCODER_RESOLUTION

        # Convert to wheel rotations and distance traveled
        rad_r, rad_l = (delta_r / ENCODER_RESOLUTION) * 2 * math.pi, (delta_l / ENCODER_RESOLUTION) * 2 * math.pi
        d_r, d_l = rad_r * WHEEL_RADIUS, rad_l * WHEEL_RADIUS

        # Compute linear and angular movement
        d = (d_r + d_l) / 2
        dtheta = (d_r - d_l) / WHEEL_BASE

        # Compute time difference
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6  # Avoid division by zero

        # Compute velocity
        vx, vth = d / dt, dtheta / dt

        # Update position
        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        self.get_logger().info(f"Odometry update: x={self.x:.3f}, y={self.y:.3f}, theta={self.theta:.3f}")

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth
        self.odom_pub.publish(odom_msg)

        # Update previous values
        self.prev_position_r, self.prev_position_l, self.prev_time = current_position_r, current_position_l, now

    def stop_motors(self):
        """ Stop motors on shutdown """
        for dxl_id in [DXL_ID_1, DXL_ID_2]:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()
        self.get_logger().info("Motors stopped and port closed.")

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_motors()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
