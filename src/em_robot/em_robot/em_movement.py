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
ENCODER_RESOLUTION = 4096  # Ticks per revolution for X-Series in extended position mode

# Control Table Addresses (for X-Series, including XC430-W150)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132

# Default settings
DXL_ID_1 = 2  # Right wheel motor ID
DXL_ID_2 = 1  # Left wheel motor ID
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
                self.get_logger().error(
                    f"Failed to enable torque on Dynamixel ID={dxl_id} (result: {result}, error: {error})")
            else:
                self.get_logger().info(f"Torque enabled on Dynamixel ID={dxl_id}")

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

    def convert_to_signed(self, val):
        """
        Convert the raw encoder value (unsigned 32-bit) to a signed integer.
        """
        if val > 2147483647:  # If the value exceeds the maximum signed 32-bit integer...
            return val - 4294967296  # ...subtract 2**32 to get the signed value.
        return val

    def cmd_vel_callback(self, msg: Twist):
        """
        Callback for cmd_vel subscription:
        Converts linear and angular velocity to the motor velocity commands.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Convert from (v, w) to each wheel's velocity in "Dynamixel speed units"
        motor_speed_r = int(
            (linear_x + (angular_z * WHEEL_BASE / 2))
            * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi)
        )
        motor_speed_l = int(
            (linear_x - (angular_z * WHEEL_BASE / 2))
            * 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi)
        )

        # Safeguard: limit the motor speeds
        motor_speed_r = max(min(motor_speed_r, 410), -410)
        motor_speed_l = max(min(motor_speed_l, 410), -410)

        self.get_logger().info(
            f"cmd_vel: v={linear_x:.3f}, w={angular_z:.3f} -> R={motor_speed_r}, L={motor_speed_l}"
        )

        # Send the goal velocities to the motors and log errors if any
        result_r, error_r = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_1, ADDR_GOAL_VELOCITY,
                                                                motor_speed_r)
        if result_r != COMM_SUCCESS or error_r != 0:
            self.get_logger().error(f"Error sending goal velocity to right wheel: result={result_r}, error={error_r}")
        else:
            self.get_logger().debug("Right wheel velocity command sent successfully.")

        result_l, error_l = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID_2, ADDR_GOAL_VELOCITY,
                                                                motor_speed_l)
        if result_l != COMM_SUCCESS or error_l != 0:
            self.get_logger().error(f"Error sending goal velocity to left wheel: result={result_l}, error={error_l}")
        else:
            self.get_logger().debug("Left wheel velocity command sent successfully.")

    def odom_callback(self):
        """
        Called by the timer to read the present positions from the Dynamixels,
        calculate odometry, and publish it.
        """
        now = self.get_clock().now()

        # Read encoder positions
        current_position_r, dxl_comm_result_r, dxl_error_r = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_1, ADDR_PRESENT_POSITION
        )
        current_position_l, dxl_comm_result_l, dxl_error_l = self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID_2, ADDR_PRESENT_POSITION
        )

        # Convert raw encoder values to signed integers
        current_position_r = self.convert_to_signed(current_position_r)
        current_position_l = self.convert_to_signed(current_position_l)

        # Log encoder readings
        self.get_logger().info(f"Raw Encoder Readings -> Right: {current_position_r}, Left: {current_position_l}")

        if self.prev_position_r is None or self.prev_position_l is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            self.prev_time = now
            return  # Skip first iteration

        # Compute encoder differences
        delta_r = current_position_r - self.prev_position_r
        delta_l = current_position_l - self.prev_position_l

        # Handle wrap-around cases (assuming 32-bit integer)
        if delta_r > ENCODER_RESOLUTION / 2:
            delta_r -= ENCODER_RESOLUTION
        elif delta_r < -ENCODER_RESOLUTION / 2:
            delta_r += ENCODER_RESOLUTION

        if delta_l > ENCODER_RESOLUTION / 2:
            delta_l -= ENCODER_RESOLUTION
        elif delta_l < -ENCODER_RESOLUTION / 2:
            delta_l += ENCODER_RESOLUTION

        self.get_logger().info(f"Encoder Tick Differences -> Right: {delta_r}, Left: {delta_l}")

        # Convert ticks to radians
        rad_r = delta_r * (2.0 * math.pi / ENCODER_RESOLUTION)
        rad_l = delta_l * (2.0 * math.pi / ENCODER_RESOLUTION)

        # Compute distances traveled
        d_r = rad_r * WHEEL_RADIUS
        d_l = rad_l * WHEEL_RADIUS

        self.get_logger().info(f"Wheel Travel Distances -> d_r: {d_r:.6f} m, d_l: {d_l:.6f} m")

        # Compute time elapsed
        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            self.get_logger().error("Time difference (dt) is zero or negative!")
            dt = 1e-6  # Prevent division by zero

        self.get_logger().info(f"Elapsed Time -> dt: {dt:.6f} sec")

        # Compute odometry values
        d = (d_r + d_l) / 2.0
        dtheta = (d_r - d_l) / WHEEL_BASE

        vx = d / dt
        vth = dtheta / dt

        self.get_logger().info(f"Computed Velocities -> Linear: {vx:.6f} m/s, Angular: {vth:.6f} rad/s")

        # Update robot pose
        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        self.get_logger().info(f"Updated Pose -> x: {self.x:.6f}, y: {self.y:.6f}, theta: {self.theta:.6f}")

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y

        # Orientation (Quaternion)
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # Velocities
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)

        # Log publishing event
        self.get_logger().info(f"Published Odometry -> x: {self.x:.6f}, y: {self.y:.6f}, theta: {self.theta:.6f}")

        # Update previous values
        self.prev_position_r = current_position_r
        self.prev_position_l = current_position_l
        self.prev_time = now

    def stop_motors(self):
        """
        On shutdown, disable torque and close the port.
        """
        for dxl_id in [DXL_ID_1, DXL_ID_2]:
            result, error = self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE,
                                                                TORQUE_DISABLE)
            if result != COMM_SUCCESS or error != 0:
                self.get_logger().error(
                    f"Error disabling torque on Dynamixel ID={dxl_id}: result={result}, error={error}")
            else:
                self.get_logger().info(f"Torque disabled on Dynamixel ID={dxl_id}")
        self.portHandler.closePort()
        self.get_logger().info("Port closed.")


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
