#!/usr/bin/env python3
from __future__ import annotations

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def _to_key_values(values: dict[str, object] | None) -> list[KeyValue]:
    if not values:
        return []

    return [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]


def build_diagnostic_status(
    name: str,
    level: int,
    message: str,
    *,
    values: dict[str, object] | None = None,
    hardware_id: str = "em_robot",
) -> DiagnosticStatus:
    status = DiagnosticStatus()
    status.name = name
    status.level = level
    status.message = message
    status.hardware_id = hardware_id
    status.values = _to_key_values(values)
    return status


def build_diagnostic_array(statuses: list[DiagnosticStatus], stamp) -> DiagnosticArray:
    diagnostic_array = DiagnosticArray()
    diagnostic_array.header.stamp = stamp
    diagnostic_array.status = statuses
    return diagnostic_array
