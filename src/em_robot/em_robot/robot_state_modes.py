# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

from dataclasses import dataclass


DIAGNOSTIC_LEVEL_WARN = 1
DIAGNOSTIC_LEVEL_ERROR = 2

MOBILITY_BOOTING = "booting"
MOBILITY_IDLE = "idle"
MOBILITY_MOVING = "moving"
MOBILITY_COMMAND_TIMEOUT = "command_timeout"
MOBILITY_BASE_STALE = "base_stale"

HEALTH_BOOTING = "booting"
HEALTH_OK = "ok"
HEALTH_DEGRADED = "degraded"
HEALTH_FAULT = "fault"

OVERALL_BOOTING = "booting"
OVERALL_READY = "ready"
OVERALL_MOVING = "moving"
OVERALL_DEGRADED = "degraded"
OVERALL_FAULT = "fault"

OFF = (0.0, 0.0, 0.0)
WHITE = (1.0, 1.0, 1.0)
CYAN = (0.0, 0.8, 1.0)
BLUE = (0.0, 0.2, 1.0)
GREEN = (0.0, 1.0, 0.0)
YELLOW = (1.0, 0.7, 0.0)
RED = (1.0, 0.0, 0.0)


@dataclass
class RobotSignalSnapshot:
    cmd_vel_age_s: float | None = None
    last_nonzero_cmd_age_s: float | None = None
    cmd_linear: float = 0.0
    cmd_angular: float = 0.0
    odom_age_s: float | None = None
    odom_linear: float = 0.0
    odom_angular: float = 0.0
    imu_age_s: float | None = None
    filtered_odom_age_s: float | None = None
    diagnostics_age_s: float | None = None
    max_non_motion_diagnostic_level: int | None = None
    cmd_vel_timeout: float = 0.25
    expected_odom_rate: float = 30.0
    expected_imu_rate: float = 30.0
    expected_filtered_odom_rate: float = 30.0
    moving_linear_threshold: float = 0.02
    moving_angular_threshold: float = 0.1
    filtered_odom_expected: bool = True


def freshness_timeout(expected_rate_hz: float) -> float:
    if expected_rate_hz <= 0.0:
        return 1.0
    return max(0.5, 3.0 / expected_rate_hz)


def _has_motion(snapshot_linear: float, snapshot_angular: float, snapshot: RobotSignalSnapshot) -> bool:
    return (
        abs(snapshot_linear) >= snapshot.moving_linear_threshold
        or abs(snapshot_angular) >= snapshot.moving_angular_threshold
    )


def compute_mobility_mode(snapshot: RobotSignalSnapshot) -> str:
    odom_timeout = freshness_timeout(snapshot.expected_odom_rate)

    if snapshot.odom_age_s is None:
        return MOBILITY_BOOTING

    if snapshot.odom_age_s > odom_timeout:
        return MOBILITY_BASE_STALE

    if _has_motion(snapshot.odom_linear, snapshot.odom_angular, snapshot):
        return MOBILITY_MOVING

    if snapshot.cmd_vel_age_s is not None and snapshot.cmd_vel_age_s <= snapshot.cmd_vel_timeout:
        if _has_motion(snapshot.cmd_linear, snapshot.cmd_angular, snapshot):
            return MOBILITY_MOVING

    recent_nonzero_window = max(1.0, snapshot.cmd_vel_timeout * 4.0)
    if (
        snapshot.last_nonzero_cmd_age_s is not None
        and snapshot.last_nonzero_cmd_age_s <= recent_nonzero_window
        and (snapshot.cmd_vel_age_s is None or snapshot.cmd_vel_age_s > snapshot.cmd_vel_timeout)
    ):
        return MOBILITY_COMMAND_TIMEOUT

    return MOBILITY_IDLE


def compute_health_mode(snapshot: RobotSignalSnapshot) -> str:
    imu_timeout = freshness_timeout(snapshot.expected_imu_rate)
    filtered_odom_timeout = freshness_timeout(snapshot.expected_filtered_odom_rate)

    if snapshot.imu_age_s is None:
        return HEALTH_BOOTING

    if snapshot.imu_age_s > imu_timeout:
        return HEALTH_FAULT

    if snapshot.filtered_odom_expected:
        if snapshot.filtered_odom_age_s is None:
            return HEALTH_BOOTING

        if snapshot.filtered_odom_age_s > filtered_odom_timeout:
            return HEALTH_DEGRADED

    if snapshot.max_non_motion_diagnostic_level is None:
        return HEALTH_OK

    if snapshot.max_non_motion_diagnostic_level >= DIAGNOSTIC_LEVEL_ERROR:
        return HEALTH_FAULT

    if snapshot.max_non_motion_diagnostic_level >= DIAGNOSTIC_LEVEL_WARN:
        return HEALTH_DEGRADED

    return HEALTH_OK


def compute_overall_mode(mobility_mode: str, health_mode: str) -> str:
    if mobility_mode == MOBILITY_BOOTING or health_mode == HEALTH_BOOTING:
        return OVERALL_BOOTING

    if mobility_mode == MOBILITY_BASE_STALE or health_mode == HEALTH_FAULT:
        return OVERALL_FAULT

    if mobility_mode == MOBILITY_COMMAND_TIMEOUT or health_mode == HEALTH_DEGRADED:
        return OVERALL_DEGRADED

    if mobility_mode == MOBILITY_MOVING:
        return OVERALL_MOVING

    return OVERALL_READY


def mobility_led_color(mode: str, blink_on: bool) -> tuple[float, float, float]:
    if mode == MOBILITY_BOOTING:
        return WHITE if blink_on else OFF

    if mode == MOBILITY_IDLE:
        return CYAN

    if mode == MOBILITY_MOVING:
        return BLUE

    if mode == MOBILITY_COMMAND_TIMEOUT:
        return YELLOW if blink_on else OFF

    if mode == MOBILITY_BASE_STALE:
        return RED if blink_on else OFF

    return OFF


def health_led_color(mode: str, blink_on: bool) -> tuple[float, float, float]:
    if mode == HEALTH_BOOTING:
        return WHITE if blink_on else OFF

    if mode == HEALTH_OK:
        return GREEN

    if mode == HEALTH_DEGRADED:
        return YELLOW if blink_on else OFF

    if mode == HEALTH_FAULT:
        return RED if blink_on else OFF

    return OFF
