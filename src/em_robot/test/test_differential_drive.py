# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math

from em_robot.differential_drive import (
    PoseState,
    StartupMotionGateState,
    apply_pose_delta,
    cmd_vel_to_motor_speeds,
    compute_wheel_odometry_delta,
    encoder_delta_limit,
    imu_motion_detected,
    integrate_fake_motion,
    normalize_encoder_delta,
    update_startup_motion_gate,
)


def test_cmd_vel_to_motor_speeds_clamps_output():
    right_speed, left_speed = cmd_vel_to_motor_speeds(
        linear_x=100.0,
        angular_z=10.0,
        max_speed=0.5,
    )

    assert -410 <= right_speed <= 410
    assert -410 <= left_speed <= 410


def test_integrate_fake_motion_updates_pose():
    pose_state = PoseState()

    result = integrate_fake_motion(
        linear_x=0.2,
        angular_z=0.0,
        dt=1.0,
        pose_state=pose_state,
        max_speed=1.0,
        max_pos_step=1.0,
    )

    assert math.isclose(result["pose"].x, 0.2, rel_tol=1e-6)
    assert math.isclose(result["pose"].y, 0.0, abs_tol=1e-9)
    assert math.isclose(result["vx"], 0.2, rel_tol=1e-6)


def test_normalize_encoder_delta_wraps_single_turn_values():
    assert normalize_encoder_delta(4090) == -6
    assert normalize_encoder_delta(-4090) == 6


def test_encoder_delta_limit_matches_physical_cycle_budget():
    limit = encoder_delta_limit(1.0 / 30.0)

    assert 600.0 <= limit < 700.0


def test_compute_wheel_odometry_delta_matches_applied_pose_update():
    pose_state = PoseState()
    odom_delta = compute_wheel_odometry_delta(
        delta_r_ticks=256,
        delta_l_ticks=256,
        dt=0.1,
        max_speed=10.0,
        max_pos_step=10.0,
    )

    apply_pose_delta(pose_state, odom_delta["d"], odom_delta["dtheta"])

    expected_distance = 256 * (2.0 * math.pi / 4096.0) * 0.035
    assert math.isclose(odom_delta["d"], expected_distance, rel_tol=1e-6)
    assert math.isclose(odom_delta["dtheta"], 0.0, abs_tol=1e-9)
    assert math.isclose(pose_state.x, expected_distance, rel_tol=1e-6)
    assert math.isclose(pose_state.y, 0.0, abs_tol=1e-9)


def test_compute_wheel_odometry_delta_accepts_calibrated_geometry():
    odom_delta = compute_wheel_odometry_delta(
        delta_r_ticks=256,
        delta_l_ticks=128,
        dt=0.1,
        max_speed=10.0,
        max_pos_step=10.0,
        wheel_radius=0.04,
        wheel_base=0.16,
        encoder_resolution=2048,
    )

    expected_d_r = 256 * (2.0 * math.pi / 2048.0) * 0.04
    expected_d_l = 128 * (2.0 * math.pi / 2048.0) * 0.04
    assert math.isclose(odom_delta["d"], (expected_d_r + expected_d_l) / 2.0, rel_tol=1e-6)
    assert math.isclose(odom_delta["dtheta"], (expected_d_r - expected_d_l) / 0.16, rel_tol=1e-6)


def test_compute_wheel_odometry_delta_accepts_per_wheel_scales():
    baseline = compute_wheel_odometry_delta(
        delta_r_ticks=256,
        delta_l_ticks=256,
        dt=0.1,
        max_speed=10.0,
        max_pos_step=10.0,
    )
    scaled = compute_wheel_odometry_delta(
        delta_r_ticks=256,
        delta_l_ticks=256,
        dt=0.1,
        max_speed=10.0,
        max_pos_step=10.0,
        right_wheel_odom_scale=1.02,
        left_wheel_odom_scale=0.98,
    )

    assert scaled["dtheta"] > baseline["dtheta"]
    assert math.isclose(scaled["d"], baseline["d"], rel_tol=0.03)


def test_imu_motion_detected_accepts_linear_acceleration_or_yaw_rate():
    assert imu_motion_detected(0.3, 0.0, 0.0, accel_threshold=0.2, gyro_threshold=0.15)
    assert imu_motion_detected(0.0, 0.0, 0.2, accel_threshold=0.2, gyro_threshold=0.15)
    assert not imu_motion_detected(0.05, 0.05, 0.01, accel_threshold=0.2, gyro_threshold=0.15)


def test_startup_motion_gate_enters_hold_and_then_stall_without_imu_confirmation():
    gate_state = StartupMotionGateState()

    first_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_motion_seen=False,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        confirmation_time_s=0.2,
    )
    second_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_motion_seen=False,
        now_s=1.25,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        confirmation_time_s=0.2,
    )

    assert first_action == "hold"
    assert second_action == "hold"
    assert gate_state.stall_active
    assert not gate_state.motion_confirmed


def test_startup_motion_gate_allows_motion_once_imu_confirms_and_resets_after_stop():
    gate_state = StartupMotionGateState()

    hold_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_motion_seen=False,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        confirmation_time_s=0.2,
    )
    allow_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_motion_seen=True,
        now_s=1.05,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        confirmation_time_s=0.2,
    )
    reset_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.0,
        wheel_vth=0.0,
        imu_motion_seen=False,
        now_s=1.4,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        confirmation_time_s=0.2,
    )

    assert hold_action == "hold"
    assert allow_action == "allow"
    assert reset_action == "allow"
    assert not gate_state.motion_confirmed
    assert not gate_state.stall_active
