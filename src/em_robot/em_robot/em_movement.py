#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from dynamixel_sdk import *  # Import DYNAMIXEL SDK library
import math

# --- Constants ---
# Physical dimensions (in meters)
WHEEL_RADIUS = 0.035  # 35 mm
WHEEL_BASE = 0.129    # 129 mm

# Dynamixel control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104

# Dynamixel default settings
DXL_ID_1 = 1  # Right wheel motor ID
DXL_ID_2 = 2  # Left wheel motor ID
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.get_logger().info('MovementNode started')

        # Initialize Dynamixel PortHandler and PacketHandler
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(2.0)

        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")
            return
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return

        # Enable torque on both motors
        for dxl_id in [DXL_ID_1, DXL_ID_2]:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        # Create subscription to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def cmd_vel_callback(self, msg: Twist):
        # Extract linear and angular velocities from the Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel speeds (in Dynamixel speed units)
        # The conversion factor here is taken from your original formula.
        # Adjust the constants if necessary.
        motor_speed_r = int((-linear_x - (angular_z * WHEEL_BASE / 2)) * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi))
        motor_speed_l = int((linear_x - (angular_z * WHEEL_BASE / 2)) * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi))

        # Limit the motor speeds to the range [-410, 410]
        motor_speed_r = max(min(motor_speed_r, 410), -410)
        motor_speed_l = max(min(motor_speed_l, 410), -410)

        self.get_logger().info(f"cmd_vel: linear_x={linear_x}, angular_z={angular_z} -> right={motor_speed_r}, left={motor_speed_l}")

        # Send the goal velocities to the motors
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_GOAL_VELOCITY, motor_speed_r)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_GOAL_VELOCITY, motor_speed_l)

    def stop_motors(self):
        # Disable torque on shutdown for all motors
        for dxl_id in [DXL_ID_1, DXL_ID_2]:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, shutting down...")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
