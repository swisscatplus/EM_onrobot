#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Any

import yaml


DEFAULT_MAP_FRAME = "map"
DEFAULT_MARKER_PREFIX = "aruco_"
DEFAULT_CEILING_MARKER_ROLL = math.pi
DEFAULT_CEILING_MARKER_PITCH = 0.0
DEFAULT_MARKER_Z = 0.0


def _normalize_marker_entry(marker: dict[str, Any]) -> dict[str, Any]:
    pose = marker.get("pose", marker)
    return {
        "id": int(marker["id"]),
        "x": float(pose.get("x", 0.0)),
        "y": float(pose.get("y", 0.0)),
        "z": float(pose.get("z", 0.0)),
        "roll": float(pose.get("roll", 0.0)),
        "pitch": float(pose.get("pitch", 0.0)),
        "yaw": float(pose.get("yaw", 0.0)),
    }


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _flatten_track_marker_entries(config: dict[str, Any]) -> list[dict[str, Any]]:
    tracks = (
        config.get("mob_rob_loca", {})
        .get("ros__parameters", {})
        .get("tracks", {})
    )
    if not isinstance(tracks, dict):
        return []

    markers: list[dict[str, Any]] = []
    for track in tracks.values():
        track_x = float(track.get("t_x", 0.0))
        track_y = float(track.get("t_y", 0.0))
        track_yaw = float(track.get("yaw", 0.0))

        for marker_id, marker in (track.get("markers") or {}).items():
            marker_tx = float(marker.get("t_x", 0.0))
            marker_ty = float(marker.get("t_y", 0.0))
            marker_yaw = float(marker.get("yaw", 0.0))

            # Legacy track maps define markers in the local track frame.
            # Ceiling markers are treated as downward-facing by default, so only
            # x/y/yaw (and optional height) need to be maintained in the config.
            markers.append(
                {
                    "id": int(marker_id),
                    "x": track_x + marker_tx * math.cos(track_yaw) - marker_ty * math.sin(track_yaw),
                    "y": track_y + marker_tx * math.sin(track_yaw) + marker_ty * math.cos(track_yaw),
                    "z": float(marker.get("t_z", DEFAULT_MARKER_Z)),
                    "roll": float(marker.get("roll", DEFAULT_CEILING_MARKER_ROLL)),
                    "pitch": float(marker.get("pitch", DEFAULT_CEILING_MARKER_PITCH)),
                    "yaw": _wrap_angle(track_yaw + marker_yaw),
                }
            )

    return markers


def load_marker_map_config(config_file: str) -> dict[str, Any]:
    with open(config_file, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    raw_markers = config.get("markers", [])
    if not raw_markers:
        raw_markers = _flatten_track_marker_entries(config)

    markers = []
    seen_ids: set[int] = set()
    for marker in raw_markers:
        normalized = _normalize_marker_entry(marker)
        marker_id = normalized["id"]
        if marker_id in seen_ids:
            raise ValueError(f"Duplicate marker id {marker_id} in {config_file}")
        seen_ids.add(marker_id)
        markers.append(normalized)

    return {
        "map_frame": str(config.get("map_frame", DEFAULT_MAP_FRAME)),
        "marker_prefix": str(config.get("marker_prefix", DEFAULT_MARKER_PREFIX)),
        "markers": markers,
    }
