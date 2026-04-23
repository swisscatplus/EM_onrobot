#!/usr/bin/env python3
import math
from dataclasses import dataclass


WHEEL_RADIUS = 0.035
WHEEL_BASE = 0.131
ENCODER_RESOLUTION = 4096
MAX_GOAL_VELOCITY_TICKS = 410
GOAL_VELOCITY_RPM_PER_TICK = 0.229
MOTOR_RPM_TO_TICKS_FACTOR = 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi)


@dataclass
class PoseState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class StartupMotionGateState:
    motion_confirmed: bool = False
    validation_started_at: float | None = None
    stall_active: bool = False


def convert_to_signed(value):
    return value - 4294967296 if value > 2147483647 else value


def clamp(value, lower, upper):
    return max(min(value, upper), lower)


def cmd_vel_to_motor_speeds(linear_x, angular_z, max_speed):
    linear_x = clamp(linear_x, -max_speed, max_speed)
    motor_speed_r = int((linear_x + (angular_z * WHEEL_BASE / 2.0)) * MOTOR_RPM_TO_TICKS_FACTOR)
    motor_speed_l = int((linear_x - (angular_z * WHEEL_BASE / 2.0)) * MOTOR_RPM_TO_TICKS_FACTOR)
    return clamp(motor_speed_r, -410, 410), clamp(motor_speed_l, -410, 410)


def normalize_encoder_delta(delta):
    if delta > ENCODER_RESOLUTION / 2:
        return delta - ENCODER_RESOLUTION
    if delta < -ENCODER_RESOLUTION / 2:
        return delta + ENCODER_RESOLUTION
    return delta


def encoder_delta_limit(dt, safety_factor=3.0, minimum_ticks=64.0):
    safe_dt = max(dt, 1e-6)
    max_wheel_rps = (MAX_GOAL_VELOCITY_TICKS * GOAL_VELOCITY_RPM_PER_TICK) / 60.0
    max_ticks = max_wheel_rps * ENCODER_RESOLUTION * safe_dt * safety_factor
    return max(minimum_ticks, max_ticks)


def compute_wheel_odometry_delta(delta_r_ticks, delta_l_ticks, dt, max_speed, max_pos_step):
    delta_r_ticks = normalize_encoder_delta(delta_r_ticks)
    delta_l_ticks = normalize_encoder_delta(delta_l_ticks)

    rad_r = delta_r_ticks * (2.0 * math.pi / ENCODER_RESOLUTION)
    rad_l = delta_l_ticks * (2.0 * math.pi / ENCODER_RESOLUTION)

    d_r = rad_r * WHEEL_RADIUS
    d_l = rad_l * WHEEL_RADIUS
    d = (d_r + d_l) / 2.0
    dtheta = (d_r - d_l) / WHEEL_BASE

    safe_dt = max(dt, 1e-6)
    vx = d / safe_dt
    if abs(vx) > max_speed:
        vx = math.copysign(max_speed, vx)

    if abs(d) > max_pos_step:
        d = math.copysign(max_pos_step, d)
        vx = d / safe_dt

    return {
        "d": d,
        "dtheta": dtheta,
        "vx": vx,
        "vth": dtheta / safe_dt,
    }


def apply_pose_delta(pose_state, d, dtheta):
    pose_state.x += d * math.cos(pose_state.theta + dtheta / 2.0)
    pose_state.y += d * math.sin(pose_state.theta + dtheta / 2.0)
    pose_state.theta += dtheta


def integrate_wheel_odometry(delta_r_ticks, delta_l_ticks, dt, pose_state, max_speed, max_pos_step):
    odom_delta = compute_wheel_odometry_delta(
        delta_r_ticks=delta_r_ticks,
        delta_l_ticks=delta_l_ticks,
        dt=dt,
        max_speed=max_speed,
        max_pos_step=max_pos_step,
    )
    apply_pose_delta(pose_state, odom_delta["d"], odom_delta["dtheta"])

    return {
        "pose": pose_state,
        "vx": odom_delta["vx"],
        "vth": odom_delta["vth"],
    }


def integrate_fake_motion(linear_x, angular_z, dt, pose_state, max_speed, max_pos_step):
    linear_x = clamp(linear_x, -max_speed, max_speed)
    safe_dt = max(dt, 1e-6)
    d = clamp(linear_x * safe_dt, -max_pos_step, max_pos_step)
    dtheta = angular_z * safe_dt

    pose_state.x += d * math.cos(pose_state.theta + dtheta / 2.0)
    pose_state.y += d * math.sin(pose_state.theta + dtheta / 2.0)
    pose_state.theta += dtheta

    return {
        "pose": pose_state,
        "vx": d / safe_dt,
        "vth": dtheta / safe_dt,
    }


def update_startup_motion_gate(
    gate_state,
    *,
    wheel_vx,
    wheel_vth,
    imu_motion_seen,
    now_s,
    linear_velocity_threshold,
    angular_velocity_threshold,
    confirmation_time_s,
):
    wheel_motion_seen = (
        abs(wheel_vx) >= linear_velocity_threshold
        or abs(wheel_vth) >= angular_velocity_threshold
    )

    if gate_state.motion_confirmed:
        if not wheel_motion_seen:
            gate_state.motion_confirmed = False
            gate_state.validation_started_at = None
            gate_state.stall_active = False
        return "allow"

    if not wheel_motion_seen:
        gate_state.validation_started_at = None
        gate_state.stall_active = False
        return "allow"

    if imu_motion_seen:
        gate_state.motion_confirmed = True
        gate_state.validation_started_at = None
        gate_state.stall_active = False
        return "allow"

    if gate_state.validation_started_at is None:
        gate_state.validation_started_at = now_s

    if now_s - gate_state.validation_started_at >= confirmation_time_s:
        gate_state.stall_active = True

    return "hold"


def imu_motion_detected(linear_accel_x, linear_accel_y, angular_velocity_z, accel_threshold, gyro_threshold):
    linear_accel_xy = math.hypot(linear_accel_x, linear_accel_y)
    return linear_accel_xy >= accel_threshold or abs(angular_velocity_z) >= gyro_threshold
