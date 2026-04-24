from em_robot.robot_state_modes import (
    HEALTH_DEGRADED,
    HEALTH_FAULT,
    HEALTH_OK,
    MOBILITY_BASE_STALE,
    MOBILITY_BOOTING,
    MOBILITY_COMMAND_TIMEOUT,
    MOBILITY_IDLE,
    MOBILITY_MOVING,
    OVERALL_DEGRADED,
    OVERALL_FAULT,
    OVERALL_MOVING,
    OVERALL_READY,
    RobotSignalSnapshot,
    compute_health_mode,
    compute_mobility_mode,
    compute_overall_mode,
)


def test_compute_mobility_mode_is_booting_before_first_odom():
    snapshot = RobotSignalSnapshot()

    assert compute_mobility_mode(snapshot) == MOBILITY_BOOTING


def test_compute_mobility_mode_detects_motion_from_fresh_odom():
    snapshot = RobotSignalSnapshot(
        odom_age_s=0.05,
        odom_linear=0.15,
        expected_odom_rate=30.0,
    )

    assert compute_mobility_mode(snapshot) == MOBILITY_MOVING


def test_compute_mobility_mode_reports_recent_command_timeout():
    snapshot = RobotSignalSnapshot(
        odom_age_s=0.05,
        odom_linear=0.0,
        cmd_vel_age_s=0.6,
        last_nonzero_cmd_age_s=0.6,
        cmd_linear=0.2,
        cmd_vel_timeout=0.25,
        expected_odom_rate=30.0,
    )

    assert compute_mobility_mode(snapshot) == MOBILITY_COMMAND_TIMEOUT


def test_compute_mobility_mode_reports_stale_base_when_odom_times_out():
    snapshot = RobotSignalSnapshot(
        odom_age_s=1.0,
        expected_odom_rate=30.0,
    )

    assert compute_mobility_mode(snapshot) == MOBILITY_BASE_STALE


def test_compute_mobility_mode_returns_idle_when_robot_is_still_and_fresh():
    snapshot = RobotSignalSnapshot(
        odom_age_s=0.05,
        expected_odom_rate=30.0,
    )

    assert compute_mobility_mode(snapshot) == MOBILITY_IDLE


def test_compute_health_mode_is_ok_when_signals_are_fresh():
    snapshot = RobotSignalSnapshot(
        imu_age_s=0.05,
        filtered_odom_age_s=0.05,
        expected_imu_rate=100.0,
        expected_filtered_odom_rate=30.0,
        filtered_odom_expected=True,
        max_non_motion_diagnostic_level=0,
    )

    assert compute_health_mode(snapshot) == HEALTH_OK


def test_compute_health_mode_degrades_when_filtered_odom_goes_stale():
    snapshot = RobotSignalSnapshot(
        imu_age_s=0.05,
        filtered_odom_age_s=1.0,
        expected_imu_rate=100.0,
        expected_filtered_odom_rate=30.0,
        filtered_odom_expected=True,
    )

    assert compute_health_mode(snapshot) == HEALTH_DEGRADED


def test_compute_health_mode_faults_on_error_diagnostics():
    snapshot = RobotSignalSnapshot(
        imu_age_s=0.05,
        filtered_odom_age_s=0.05,
        expected_imu_rate=100.0,
        expected_filtered_odom_rate=30.0,
        filtered_odom_expected=True,
        max_non_motion_diagnostic_level=2,
    )

    assert compute_health_mode(snapshot) == HEALTH_FAULT


def test_compute_overall_mode_prioritizes_faults_and_degraded_states():
    assert compute_overall_mode(MOBILITY_IDLE, HEALTH_OK) == OVERALL_READY
    assert compute_overall_mode(MOBILITY_MOVING, HEALTH_OK) == OVERALL_MOVING
    assert compute_overall_mode(MOBILITY_COMMAND_TIMEOUT, HEALTH_OK) == OVERALL_DEGRADED
    assert compute_overall_mode(MOBILITY_IDLE, HEALTH_DEGRADED) == OVERALL_DEGRADED
    assert compute_overall_mode(MOBILITY_BASE_STALE, HEALTH_OK) == OVERALL_FAULT
