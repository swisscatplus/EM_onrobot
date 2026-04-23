#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu

from em_robot.differential_drive import (
    LoadStallGateState,
    PoseState,
    StartupMotionGateState,
    apply_pose_delta,
    cmd_vel_to_motor_speeds,
    compute_wheel_odometry_delta,
    convert_to_signed,
    encoder_delta_limit,
    hardware_overload_detected,
    integrate_fake_motion,
    normalize_encoder_delta,
    update_load_stall_gate,
    update_startup_motion_gate,
)

try:
    from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
except ImportError:  # pragma: no cover - depends on runtime image
    COMM_SUCCESS = None
    PacketHandler = None
    PortHandler = None


ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132
DYNAMIXEL_OVERLOAD_ERROR_BIT = 0x20
TORQUE_ENABLE = 1


class BaseControllerNode(Node):
    def __init__(self):
        super().__init__("base_controller")
        self.get_logger().info("Base controller node started")

        self.declare_parameter("backend", "real")
        self.declare_parameter("max_speed", 1000.0)
        self.declare_parameter("odom_rate", 30.0)
        self.declare_parameter("cmd_vel_timeout", 0.25)
        self.declare_parameter("device_name", "/dev/dynamixel")
        self.declare_parameter("baudrate", 57600)
        self.declare_parameter("right_motor_id", 2)
        self.declare_parameter("left_motor_id", 1)
        self.declare_parameter("imu_topic", "/bno055/imu")
        self.declare_parameter("startup_motion_gate_enabled", False)
        self.declare_parameter("startup_motion_gate_confirmation_time", 0.2)
        self.declare_parameter("startup_motion_gate_linear_velocity_threshold", 0.05)
        self.declare_parameter("startup_motion_gate_angular_velocity_threshold", 0.2)
        self.declare_parameter("startup_motion_gate_accel_threshold", 0.15)
        self.declare_parameter("startup_motion_gate_linear_delta_v_threshold", 0.03)
        self.declare_parameter("startup_motion_gate_gyro_threshold", 0.12)
        self.declare_parameter("startup_motion_gate_imu_timeout", 0.2)
        self.declare_parameter("overload_stall_gate_enabled", False)
        self.declare_parameter("load_stall_gate_enabled", False)
        self.declare_parameter("load_stall_gate_confirmation_time", 0.2)
        self.declare_parameter("load_stall_gate_linear_velocity_threshold", 0.05)
        self.declare_parameter("load_stall_gate_load_threshold", 450.0)

        self.backend = self.get_parameter("backend").value
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.odom_rate = float(self.get_parameter("odom_rate").value)
        self.cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self.device_name = self.get_parameter("device_name").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.right_motor_id = int(self.get_parameter("right_motor_id").value)
        self.left_motor_id = int(self.get_parameter("left_motor_id").value)
        self.imu_topic = self.get_parameter("imu_topic").value
        self.startup_motion_gate_enabled = bool(
            self.get_parameter("startup_motion_gate_enabled").value
        )
        self.startup_motion_gate_confirmation_time = float(
            self.get_parameter("startup_motion_gate_confirmation_time").value
        )
        self.startup_motion_gate_linear_velocity_threshold = float(
            self.get_parameter("startup_motion_gate_linear_velocity_threshold").value
        )
        self.startup_motion_gate_angular_velocity_threshold = float(
            self.get_parameter("startup_motion_gate_angular_velocity_threshold").value
        )
        self.startup_motion_gate_accel_threshold = float(
            self.get_parameter("startup_motion_gate_accel_threshold").value
        )
        self.startup_motion_gate_linear_delta_v_threshold = float(
            self.get_parameter("startup_motion_gate_linear_delta_v_threshold").value
        )
        self.startup_motion_gate_gyro_threshold = float(
            self.get_parameter("startup_motion_gate_gyro_threshold").value
        )
        self.startup_motion_gate_imu_timeout = float(
            self.get_parameter("startup_motion_gate_imu_timeout").value
        )
        self.overload_stall_gate_enabled = bool(
            self.get_parameter("overload_stall_gate_enabled").value
        )
        self.load_stall_gate_enabled = bool(
            self.get_parameter("load_stall_gate_enabled").value
        )
        self.load_stall_gate_confirmation_time = float(
            self.get_parameter("load_stall_gate_confirmation_time").value
        )
        self.load_stall_gate_linear_velocity_threshold = float(
            self.get_parameter("load_stall_gate_linear_velocity_threshold").value
        )
        self.load_stall_gate_load_threshold = float(
            self.get_parameter("load_stall_gate_load_threshold").value
        )
        self.max_pos_step = self.max_speed / self.odom_rate

        self.subscription = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "odomWheel", 10)
        self.odom_timer = self.create_timer(1.0 / self.odom_rate, self.odom_callback)

        self.pose_state = PoseState()
        self.prev_position_r = None
        self.prev_position_l = None
        self.prev_time = self.get_clock().now()
        self.last_cmd_time = None
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.cmd_vel_stale = False
        self.watchdog_trip_count = 0
        self.encoder_glitch_count = 0
        self.port_handler = None
        self.packet_handler = None
        self.imu_subscription = None
        self.last_imu_time = None
        self.last_imu_linear_accel_x = 0.0
        self.last_imu_linear_accel_y = 0.0
        self.last_imu_angular_velocity_z = 0.0
        self.motion_gate_state = StartupMotionGateState()
        self.startup_stall_hold_count = 0
        self.load_stall_gate_state = LoadStallGateState()
        self.load_stall_hold_count = 0
        self.overload_hold_count = 0

        if self.startup_motion_gate_enabled:
            self.imu_subscription = self.create_subscription(
                Imu,
                self.imu_topic,
                self.imu_callback,
                10,
            )

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

    def _read_present_position(self, motor_id):
        present_position, comm_result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_POSITION
        )

        if comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(comm_result)
            self.get_logger().warn(f"Failed to read present position for motor ID={motor_id}: {message}")
            return None

        if error != 0:
            message = self.packet_handler.getRxPacketError(error)
            self.get_logger().warn(
                f"Dynamixel reported a hardware error while reading motor ID={motor_id}: {message}"
            )
            return None

        return convert_to_signed(present_position)

    def _read_present_load(self, motor_id):
        present_load, comm_result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, ADDR_PRESENT_LOAD
        )

        if comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(comm_result)
            self.get_logger().warn(f"Failed to read present load for motor ID={motor_id}: {message}")
            return None

        if error != 0:
            message = self.packet_handler.getRxPacketError(error)
            self.get_logger().warn(
                f"Dynamixel reported a hardware error while reading load for motor ID={motor_id}: {message}"
            )
            return None

        return convert_to_signed(present_load, bits=16)

    def _read_hardware_error_status(self, motor_id):
        hardware_error_status, comm_result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, ADDR_HARDWARE_ERROR_STATUS
        )

        if comm_result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(comm_result)
            self.get_logger().warn(
                f"Failed to read hardware error status for motor ID={motor_id}: {message}"
            )
            return None

        if error != 0:
            message = self.packet_handler.getRxPacketError(error)
            self.get_logger().warn(
                "Dynamixel reported a hardware error while reading hardware error "
                f"status for motor ID={motor_id}: {message}"
            )
            return None

        return hardware_error_status

    def imu_callback(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        self.last_imu_linear_accel_x = float(msg.linear_acceleration.x)
        self.last_imu_linear_accel_y = float(msg.linear_acceleration.y)
        self.last_imu_angular_velocity_z = float(msg.angular_velocity.z)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_linear = float(msg.linear.x)
        self.last_cmd_angular = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

        if self.cmd_vel_stale:
            self.get_logger().info("cmd_vel resumed; base controller watchdog cleared")
            self.cmd_vel_stale = False

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

    def apply_cmd_vel_watchdog(self, now):
        if self.cmd_vel_timeout <= 0.0 or self.last_cmd_time is None:
            return

        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        if cmd_age <= self.cmd_vel_timeout:
            return

        if not self.cmd_vel_stale:
            self.watchdog_trip_count += 1
            self.cmd_vel_stale = True
            self.get_logger().warn(
                f"cmd_vel timed out after {cmd_age:.3f}s; stopping base "
                f"(watchdog trip #{self.watchdog_trip_count})"
            )
            if self.backend == "real":
                self.stop_motors()

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

    def odom_callback(self):
        now = self.get_clock().now()
        self.apply_cmd_vel_watchdog(now)
        dt = (now - self.prev_time).nanoseconds / 1e9

        if self.backend == "real":
            odom_data = self._compute_real_odom(now, dt)
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

    def _reset_startup_motion_gate(self):
        self.motion_gate_state.motion_confirmed = False
        self.motion_gate_state.validation_started_at = None
        self.motion_gate_state.stall_active = False
        self.motion_gate_state.last_update_at = None
        self.motion_gate_state.forward_delta_v = 0.0

    def _reset_load_stall_gate(self):
        self.load_stall_gate_state.high_load_started_at = None
        self.load_stall_gate_state.stall_active = False
        self.load_stall_gate_state.overload_active = False

    def _imu_is_fresh(self, now):
        if self.last_imu_time is None:
            return False
        age_s = (now - self.last_imu_time).nanoseconds / 1e9
        return age_s <= self.startup_motion_gate_imu_timeout

    def _should_hold_startup_motion(self, now, wheel_vx, wheel_vth):
        if not self.startup_motion_gate_enabled:
            return False

        if not self._imu_is_fresh(now):
            self._reset_startup_motion_gate()
            return False

        was_stalled = self.motion_gate_state.stall_active
        action = update_startup_motion_gate(
            self.motion_gate_state,
            wheel_vx=wheel_vx,
            wheel_vth=wheel_vth,
            imu_linear_accel_x=self.last_imu_linear_accel_x,
            imu_angular_velocity_z=self.last_imu_angular_velocity_z,
            now_s=now.nanoseconds / 1e9,
            linear_velocity_threshold=self.startup_motion_gate_linear_velocity_threshold,
            angular_velocity_threshold=self.startup_motion_gate_angular_velocity_threshold,
            linear_accel_deadband=self.startup_motion_gate_accel_threshold,
            linear_delta_v_threshold=self.startup_motion_gate_linear_delta_v_threshold,
            angular_velocity_confirmation_threshold=self.startup_motion_gate_gyro_threshold,
            confirmation_time_s=self.startup_motion_gate_confirmation_time,
        )

        if self.motion_gate_state.stall_active and not was_stalled:
            self.startup_stall_hold_count += 1
            self.get_logger().warn(
                "Holding startup wheel odometry because encoders moved without IMU motion "
                f"(stall hold #{self.startup_stall_hold_count})"
            )
        elif was_stalled and action == "allow":
            self.get_logger().info("Resuming wheel odometry after IMU confirmed body motion")

        return action == "hold"

    def _should_hold_load_stall(self, now, wheel_vx):
        if not self.overload_stall_gate_enabled and not self.load_stall_gate_enabled:
            return False

        if abs(wheel_vx) < self.load_stall_gate_linear_velocity_threshold:
            self._reset_load_stall_gate()
            return False

        right_hardware_error_status = self._read_hardware_error_status(self.right_motor_id)
        left_hardware_error_status = self._read_hardware_error_status(self.left_motor_id)
        if right_hardware_error_status is None or left_hardware_error_status is None:
            self._reset_load_stall_gate()
            return False

        overload_detected = False
        if self.overload_stall_gate_enabled:
            overload_detected = hardware_overload_detected(
                right_hardware_error_status,
                overload_bit_mask=DYNAMIXEL_OVERLOAD_ERROR_BIT,
            ) or hardware_overload_detected(
                left_hardware_error_status,
                overload_bit_mask=DYNAMIXEL_OVERLOAD_ERROR_BIT,
            )

        if overload_detected:
            was_stalled = self.load_stall_gate_state.stall_active
            action = update_load_stall_gate(
                self.load_stall_gate_state,
                wheel_vx=wheel_vx,
                motor_loads=[],
                overload_detected=True,
                now_s=now.nanoseconds / 1e9,
                linear_velocity_threshold=self.load_stall_gate_linear_velocity_threshold,
                load_threshold=self.load_stall_gate_load_threshold,
                confirmation_time_s=self.load_stall_gate_confirmation_time,
            )
            if self.load_stall_gate_state.stall_active and not was_stalled:
                self.overload_hold_count += 1
                self.get_logger().warn(
                    "Holding wheel odometry because a Dynamixel reported overload "
                    f"(overload hold #{self.overload_hold_count})"
                )
            return action == "hold"

        if not self.load_stall_gate_enabled:
            self._reset_load_stall_gate()
            return False

        right_motor_load = self._read_present_load(self.right_motor_id)
        left_motor_load = self._read_present_load(self.left_motor_id)
        if right_motor_load is None or left_motor_load is None:
            self._reset_load_stall_gate()
            return False

        was_stalled = self.load_stall_gate_state.stall_active
        action = update_load_stall_gate(
            self.load_stall_gate_state,
            wheel_vx=wheel_vx,
            motor_loads=[right_motor_load, left_motor_load],
            overload_detected=overload_detected,
            now_s=now.nanoseconds / 1e9,
            linear_velocity_threshold=self.load_stall_gate_linear_velocity_threshold,
            load_threshold=self.load_stall_gate_load_threshold,
            confirmation_time_s=self.load_stall_gate_confirmation_time,
        )

        if self.load_stall_gate_state.stall_active and not was_stalled:
            self.load_stall_hold_count += 1
            self.get_logger().warn(
                "Holding wheel odometry because both wheel motors report high load: "
                f"right={right_motor_load}, left={left_motor_load} "
                f"(load hold #{self.load_stall_hold_count})"
            )
        elif was_stalled and action == "allow":
            self.get_logger().info("Resuming wheel odometry after load/overload stall cleared")

        return action == "hold"

    def _compute_real_odom(self, now, dt):
        current_position_r = self._read_present_position(self.right_motor_id)
        current_position_l = self._read_present_position(self.left_motor_id)

        if current_position_r is None or current_position_l is None:
            return None

        if self.prev_position_r is None:
            self.prev_position_r = current_position_r
            self.prev_position_l = current_position_l
            return None

        delta_r_ticks = current_position_r - self.prev_position_r
        delta_l_ticks = current_position_l - self.prev_position_l
        normalized_delta_r = normalize_encoder_delta(delta_r_ticks)
        normalized_delta_l = normalize_encoder_delta(delta_l_ticks)
        delta_limit_ticks = encoder_delta_limit(dt)

        if abs(normalized_delta_r) > delta_limit_ticks or abs(normalized_delta_l) > delta_limit_ticks:
            self.encoder_glitch_count += 1
            self.get_logger().warn(
                "Ignoring implausible encoder jump: "
                f"raw=({delta_r_ticks}, {delta_l_ticks}), "
                f"normalized=({normalized_delta_r}, {normalized_delta_l}), "
                f"limit={delta_limit_ticks:.1f} ticks "
                f"(glitch #{self.encoder_glitch_count})"
            )
            return None

        odom_delta = compute_wheel_odometry_delta(
            delta_r_ticks=delta_r_ticks,
            delta_l_ticks=delta_l_ticks,
            dt=dt,
            max_speed=self.max_speed,
            max_pos_step=self.max_pos_step,
        )

        self.prev_position_r = current_position_r
        self.prev_position_l = current_position_l

        if self._should_hold_startup_motion(now, odom_delta["vx"], odom_delta["vth"]):
            return {
                "pose": self.pose_state,
                "vx": 0.0,
                "vth": 0.0,
            }

        if self._should_hold_load_stall(now, odom_delta["vx"]):
            return {
                "pose": self.pose_state,
                "vx": 0.0,
                "vth": 0.0,
            }

        apply_pose_delta(self.pose_state, odom_delta["d"], odom_delta["dtheta"])
        return {
            "pose": self.pose_state,
            "vx": odom_delta["vx"],
            "vth": odom_delta["vth"],
        }

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
    node = BaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down base controller node...")
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()
