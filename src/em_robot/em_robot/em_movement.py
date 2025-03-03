#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from dynamixel_sdk import *  # Import DYNAMIXEL SDK library
import math

# --- Constants ---
WHEEL_RADIUS = 0.035      # 35 mm = 0.035 m
WHEEL_BASE   = 0.130      # 130 mm approx.
ENCODER_RESOLUTION = 4096 # Ticks per revolution in extended position mode

# Control Table Addresses (for X-Series, e.g., XC430-W150)
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116  # Use this register in extended position mode
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
        self.get_logger().info('MovementNode started (Extended Position Mode)')

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
            else:
                self.get_logger().info(f"Torque enabled on Dynamixel ID={dxl_id}")

        # Initialize goal positions from current encoder reading
        dxl_comm_result_r, dxl_error_r, current_position_r = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_1, ADDR_PRESENT_POSITION
        )
        dxl_comm_result_l, dxl_error_l, current_position_l = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_2, ADDR_PRESENT_POSITION
        )
        self.goal_position_r = current_position_r
        self.goal_position_l = current_position_l

        # Create subscription to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publisher for odometry and timer for periodic updates (10 Hz)
        self.odom_pub = self.create_publisher(Odometry, 'odomWheel', 10)
        self.odom_timer = self.create_timer(0.1, self.odom_callback)

        # Odometry state variables
        self.prev_position_r = None
        self.prev_position_l = None
        self.prev_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel subscription.
        Converts linear and angular velocities to incremental position changes.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Assume a control interval dt (matching the timer period, e.g., 0.1 s)
        dt = 0.1

        # Compute the distance each wheel should move in dt.
        # For a differential-drive robot:
        #   Right wheel: -v - (w * wheel_base/2)
        #   Left wheel:   v - (w * wheel_base/2)
        dist_r = (-linear_x - (angular_z * WHEEL_BASE / 2)) * dt
        dist_l = ( linear_x - (angular_z * WHEEL_BASE / 2)) * dt

        # Convert the distance (m) to encoder ticks.
        # Ticks per meter = ENCODER_RESOLUTION / (circumference)
        ticks_per_meter = ENCODER_RESOLUTION / (2 * math.pi * WHEEL_RADIUS)
        delta_ticks_r = int(dist_r * ticks_per_meter)
        delta_ticks_l = int(dist_l * ticks_per_meter)

        # Update goal positions
        self.goal_position_r += delta_ticks_r
        self.goal_position_l += delta_ticks_l

        self.get_logger().info(
            f"cmd_vel: v={linear_x:.3f}, w={angular_z:.3f} -> Δticks: R={delta_ticks_r}, L={delta_ticks_l}"
        )
        self.get_logger().debug(
            f"New goal positions: R={self.goal_position_r}, L={self.goal_position_l}"
        )

        # Write the new goal positions to the motors
        result_r, error_r = self.packetHandler.write4ByteTxRx(
            self.portHandler, DXL_ID_1, ADDR_GOAL_POSITION, self.goal_position_r
        )
        result_l, error_l = self.packetHandler.write4ByteTxRx(
            self.portHandler, DXL_ID_2, ADDR_GOAL_POSITION, self.goal_position_l
        )
        if result_r != COMM_SUCCESS or error_r != 0:
            self.get_logger().error(f"Error writing goal position to right wheel: result={result_r}, error={error_r}")
        if result_l != COMM_SUCCESS or error_l != 0:
            self.get_logger().error(f"Error writing goal position to left wheel: result={result_l}, error={error_l}")

    def odom_callback(self):
        """
        Timer callback to read the present positions from the Dynamixels,
        compute odometry, and publish it.
        """
        now = self.get_clock().now()

        # Read current wheel positions (encoder ticks)
        dxl_comm_result_r, dxl_error_r, current_position_r = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_1, ADDR_PRESENT_POSITION
        )
        dxl_comm_result_l, dxl_error_l, current_position_l = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_2, ADDR_PRESENT_POSITION
        )

        # Initialize previous positions on first read
        if self.prev_position_r is None or self.prev_position_l is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            self.prev_time = now
            return

        # Compute differences in ticks since last callback
        delta_r = current_position_r - self.prev_position_r
        delta_l = current_position_l - self.prev_position_l

        # Convert tick differences to radians (each tick corresponds to an angle)
        rad_r = delta_r * (2.0 * math.pi / ENCODER_RESOLUTION)
        rad_l = delta_l * (2.0 * math.pi / ENCODER_RESOLUTION)

        # Compute the distance traveled by each wheel
        d_r = rad_r * WHEEL_RADIUS
        d_l = rad_l * WHEEL_RADIUS

        # Differential drive odometry: average distance and change in heading
        d = (d_r + d_l) / 2.0
        dtheta = (d_r - d_l) / WHEEL_BASE

        # Compute elapsed time
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-6  # safeguard

        # Optionally, compute velocities (not used for integration here)
        vx = d / dt
        vth = dtheta / dt

        # Update robot pose using midpoint integration
        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # Construct and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert theta to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

        # Update previous readings
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
        self.get_logger().info("Port closed, torque disabled.")

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
