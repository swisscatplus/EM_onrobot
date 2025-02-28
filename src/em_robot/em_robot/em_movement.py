#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamixel_sdk import *  # Import DYNAMIXEL SDK library
import math

# --- Constants ---
WHEEL_RADIUS = 0.035      # 35 mm = 0.035 m
WHEEL_BASE   = 0.130      # 129 mm = 0.129 m
ENCODER_RESOLUTION = 4096 # Ticks per revolution for X-Series in extended position mode

# Control Table Addresses (for X-Series, including XC430-W150)
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_VELOCITY    = 104
ADDR_PRESENT_POSITION = 132

# Default settings
DXL_ID_1  = 1   # Right wheel motor ID
DXL_ID_2  = 2   # Left wheel motor ID
BAUDRATE  = 57600
DEVICENAME = '/dev/ttyUSB0'

# Torque control
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.get_logger().info('MovementNode started')

        # Initialize PortHandler/PacketHandler
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(2.0)

        # Attempt to open port
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

        # Create subscription to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for odometry and timer for periodic updates
        self.odom_pub = self.create_publisher(Odometry, 'odomWheel', 10)
        self.odom_timer = self.create_timer(0.1, self.odom_callback)  # 10 Hz

        # Odometry state variables
        self.prev_position_r = None
        self.prev_position_l = None
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel subscription:
        Converts linear and angular velocity to the motor velocity commands.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Convert from (v, w) to each wheel's velocity in "Dynamixel speed units"
        # The factor "0.229" is a typical 'units-to-rpm' or 'rpm-to-unit' ratio
        # for X-Series in velocity mode. This might vary if you changed Velocity Limits or Operating Mode.
        motor_speed_r = int(
            (-linear_x - (angular_z * WHEEL_BASE / 2))
            * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi)
        )
        motor_speed_l = int(
            ( linear_x - (angular_z * WHEEL_BASE / 2))
            * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi)
        )

        # Safeguard: limit the motor speeds
        motor_speed_r = max(min(motor_speed_r, 410), -410)
        motor_speed_l = max(min(motor_speed_l, 410), -410)

        self.get_logger().info(
            f"cmd_vel: v={linear_x:.3f}, w={angular_z:.3f} -> R={motor_speed_r}, L={motor_speed_l}"
        )

        # Send the goal velocities to the motors
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_GOAL_VELOCITY, motor_speed_r)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_GOAL_VELOCITY, motor_speed_l)

    def odom_callback(self):
        """
        Called by the timer to read the present positions from the Dynamixels,
        calculate odometry, and publish it.
        """
        now = self.get_clock().now()

        # Read current wheel positions
        dxl_comm_result_r, dxl_error_r, current_position_r = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_1, ADDR_PRESENT_POSITION
        )
        dxl_comm_result_l, dxl_error_l, current_position_l = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_2, ADDR_PRESENT_POSITION
        )

        # If this is the first reading, just store and return
        if self.prev_position_r is None or self.prev_position_l is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            self.prev_time = now
            return

        # Calculate differences in encoder ticks
        delta_r = current_position_r - self.prev_position_r
        delta_l = current_position_l - self.prev_position_l

        # Convert from ticks to radians
        rad_r = delta_r * (2.0 * math.pi / ENCODER_RESOLUTION)
        rad_l = delta_l * (2.0 * math.pi / ENCODER_RESOLUTION)

        # Distance traveled by each wheel
        d_r = rad_r * WHEEL_RADIUS
        d_l = rad_l * WHEEL_RADIUS

        # Odometry computations
        d = (d_r + d_l) / 2.0
        dtheta = (d_r - d_l) / WHEEL_BASE

        # Time elapsed
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6  # safeguard against division by zero

        # Robot velocities
        vx = d / dt
        vth = dtheta / dt

        # Update the robot pose (midpoint method is a bit more accurate)
        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # Construct the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation as a quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Linear and angular velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = vth

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Update previous position/time
        self.prev_position_r = current_position_r
        self.prev_position_l = current_position_l
        self.prev_time = now

    def stop_motors(self):
        """
        On shutdown, disable torque and close the port.
        """
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
