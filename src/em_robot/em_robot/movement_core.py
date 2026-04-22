#!/usr/bin/env python3
import math
from dataclasses import dataclass


WHEEL_RADIUS = 0.035
WHEEL_BASE = 0.131
ENCODER_RESOLUTION = 4096
MOTOR_RPM_TO_TICKS_FACTOR = 60 / (0.229 * WHEEL_RADIUS * 2 * math.pi)


@dataclass
class PoseState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


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


def integrate_wheel_odometry(delta_r_ticks, delta_l_ticks, dt, pose_state, max_speed, max_pos_step):
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

    pose_state.x += d * math.cos(pose_state.theta + dtheta / 2.0)
    pose_state.y += d * math.sin(pose_state.theta + dtheta / 2.0)
    pose_state.theta += dtheta

    return {
        "pose": pose_state,
        "vx": vx,
        "vth": dtheta / safe_dt,
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
