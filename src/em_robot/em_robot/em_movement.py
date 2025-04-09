#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from dynamixel_sdk import *
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from tf_transformations import euler_from_quaternion
import math

# --- Constants ---
WHEEL_RADIUS = 0.035  # 35 mm = 0.035 m
WHEEL_BASE = 0.130    # 130 mm = 0.130 m
ENCODER_RESOLUTION = 4096

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_POSITION = 132

# Motor IDs and port settings
DXL_ID_1 = 2  # Right
DXL_ID_2 = 1  # Left
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


class MovementNode(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.get_logger().info('MovementNode started')

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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_timer = self.create_timer(0.1, self.odom_callback)

        # TF listener setup for corrections
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.adjust_timer = self.create_timer(0.5, self.adjust_odometry)
        self.first_correction = True

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
        v = msg.linear.x
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

        if delta_r > ENCODER_RESOLUTION / 2: delta_r -= ENCODER_RESOLUTION
        elif delta_r < -ENCODER_RESOLUTION / 2: delta_r += ENCODER_RESOLUTION
        if delta_l > ENCODER_RESOLUTION / 2: delta_l -= ENCODER_RESOLUTION
        elif delta_l < -ENCODER_RESOLUTION / 2: delta_l += ENCODER_RESOLUTION

        rad_r = delta_r * (2.0 * math.pi / ENCODER_RESOLUTION)
        rad_l = delta_l * (2.0 * math.pi / ENCODER_RESOLUTION)

        d_r = rad_r * WHEEL_RADIUS
        d_l = rad_l * WHEEL_RADIUS

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 1e-6

        d = (d_r + d_l) / 2.0
        dtheta = (d_r - d_l) / WHEEL_BASE

        self.x += d * math.cos(self.theta + dtheta / 2.0)
        self.y += d * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

        # Publish Odometry and TF (odom → base_link)
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
        self.odom_pub.publish(odom_msg)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = quat_z
        t.transform.rotation.w = quat_w
        self.tf_broadcaster.sendTransform(t)

        self.prev_position_r = current_position_r
        self.prev_position_l = current_position_l
        self.prev_time = now

    def adjust_odometry(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now,
                                                    timeout=rclpy.duration.Duration(seconds=0.1))

            # Extraction de la translation et rotation corrigée
            x_corrige = trans.transform.translation.x
            y_corrige = trans.transform.translation.y
            orientation_q = trans.transform.rotation
            _, _, theta_corrige = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w])

            # Correction initiale (ou régulière si besoin)
            if self.first_correction:
                self.x = x_corrige
                self.y = y_corrige
                self.theta = theta_corrige
                self.first_correction = False
                self.get_logger().info("Correction initiale par caméra appliquée.")
            else:
                # Optionnel : corriger régulièrement ou seulement lors de grandes différences
                delta_x = x_corrige - self.x
                delta_y = y_corrige - self.y
                delta_theta = theta_corrige - self.theta

                seuil_correction = 0.05  # 5 cm et ~3°
                if abs(delta_x) > seuil_correction or abs(delta_y) > seuil_correction or abs(delta_theta) > 0.05:
                    self.x += delta_x
                    self.y += delta_y
                    self.theta += delta_theta
                    self.get_logger().info("Odometry corrigée par caméra.")

        except Exception as e:
            self.get_logger().debug(f"Pas de transform disponible (map→base_link): {e}")

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
