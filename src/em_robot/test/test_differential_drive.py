import math

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


def test_convert_to_signed_supports_16_bit_and_32_bit_values():
    assert convert_to_signed(0xFFFFFFFF) == -1
    assert convert_to_signed(0xFFFF, bits=16) == -1
    assert convert_to_signed(1000, bits=16) == 1000


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


def test_startup_motion_gate_enters_hold_and_then_stall_without_imu_confirmation():
    gate_state = StartupMotionGateState()

    first_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )
    second_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.25,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )

    assert first_action == "hold"
    assert second_action == "hold"
    assert gate_state.stall_active
    assert not gate_state.motion_confirmed


def test_startup_motion_gate_ignores_short_linear_accel_spike_without_enough_delta_v():
    gate_state = StartupMotionGateState()

    first_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )
    second_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.4,
        imu_angular_velocity_z=0.0,
        now_s=1.05,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )
    third_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.12,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )

    assert first_action == "hold"
    assert second_action == "hold"
    assert third_action == "hold"
    assert not gate_state.motion_confirmed
    assert gate_state.forward_delta_v < 0.03


def test_startup_motion_gate_allows_linear_motion_after_enough_signed_delta_v():
    gate_state = StartupMotionGateState()

    hold_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )
    allow_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.15,
        wheel_vth=0.0,
        imu_linear_accel_x=0.8,
        imu_angular_velocity_z=0.0,
        now_s=1.05,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )
    reset_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.0,
        wheel_vth=0.0,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.4,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )

    assert hold_action == "hold"
    assert allow_action == "allow"
    assert reset_action == "allow"
    assert not gate_state.motion_confirmed
    assert not gate_state.stall_active


def test_startup_motion_gate_allows_rotation_from_gyro_confirmation():
    gate_state = StartupMotionGateState()

    hold_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.0,
        wheel_vth=0.4,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.0,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )
    allow_action = update_startup_motion_gate(
        gate_state,
        wheel_vx=0.0,
        wheel_vth=0.4,
        imu_linear_accel_x=0.0,
        imu_angular_velocity_z=0.15,
        now_s=1.05,
        linear_velocity_threshold=0.05,
        angular_velocity_threshold=0.2,
        linear_accel_deadband=0.15,
        linear_delta_v_threshold=0.03,
        angular_velocity_confirmation_threshold=0.12,
        confirmation_time_s=0.2,
    )

    assert hold_action == "hold"
    assert allow_action == "allow"


def test_hardware_overload_detected_checks_overload_bit():
    assert hardware_overload_detected(0x20)
    assert hardware_overload_detected(0x24)
    assert not hardware_overload_detected(0x10)


def test_load_stall_gate_allows_low_load_motion():
    gate_state = LoadStallGateState()

    action = update_load_stall_gate(
        gate_state,
        wheel_vx=0.15,
        motor_loads=[120, 180],
        overload_detected=False,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        load_threshold=450.0,
        confirmation_time_s=0.2,
    )

    assert action == "allow"
    assert not gate_state.stall_active
    assert gate_state.high_load_started_at is None


def test_load_stall_gate_holds_after_sustained_high_load():
    gate_state = LoadStallGateState()

    first_action = update_load_stall_gate(
        gate_state,
        wheel_vx=0.15,
        motor_loads=[520, 540],
        overload_detected=False,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        load_threshold=450.0,
        confirmation_time_s=0.2,
    )
    second_action = update_load_stall_gate(
        gate_state,
        wheel_vx=0.15,
        motor_loads=[530, 560],
        overload_detected=False,
        now_s=1.25,
        linear_velocity_threshold=0.05,
        load_threshold=450.0,
        confirmation_time_s=0.2,
    )

    assert first_action == "allow"
    assert second_action == "hold"
    assert gate_state.stall_active
    assert not gate_state.overload_active


def test_load_stall_gate_holds_immediately_on_overload():
    gate_state = LoadStallGateState()

    action = update_load_stall_gate(
        gate_state,
        wheel_vx=0.15,
        motor_loads=[100, 140],
        overload_detected=True,
        now_s=1.0,
        linear_velocity_threshold=0.05,
        load_threshold=450.0,
        confirmation_time_s=0.2,
    )

    assert action == "hold"
    assert gate_state.stall_active
    assert gate_state.overload_active


def test_load_stall_gate_resets_after_motion_stops():
    gate_state = LoadStallGateState(
        high_load_started_at=1.0,
        stall_active=True,
        overload_active=True,
    )

    action = update_load_stall_gate(
        gate_state,
        wheel_vx=0.0,
        motor_loads=[0, 0],
        overload_detected=False,
        now_s=1.4,
        linear_velocity_threshold=0.05,
        load_threshold=450.0,
        confirmation_time_s=0.2,
    )

    assert action == "allow"
    assert gate_state.high_load_started_at is None
    assert not gate_state.stall_active
    assert not gate_state.overload_active
