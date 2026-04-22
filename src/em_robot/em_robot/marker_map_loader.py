#!/usr/bin/env python3
from __future__ import annotations

from typing import Any

import yaml


DEFAULT_MAP_FRAME = "map"
DEFAULT_MARKER_PREFIX = "aruco_"


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


def load_marker_map_config(config_file: str) -> dict[str, Any]:
    with open(config_file, "r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}

    markers = []
    seen_ids: set[int] = set()
    for marker in config.get("markers", []):
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
