#!/usr/bin/env python3
# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from em_robot.marker_map_loader import (
    DEFAULT_MAP_FRAME,
    DEFAULT_MARKER_PREFIX,
    _normalize_marker_entry,
)


def load_raw_marker_map_file(config_file: str) -> dict[str, Any]:
    path = Path(config_file)
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def merge_marker_entries(
    existing_markers: list[dict[str, Any]],
    updated_markers: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    merged_by_id = {
        int(marker["id"]): _normalize_marker_entry(marker)
        for marker in existing_markers
    }
    for marker in updated_markers:
        normalized = _normalize_marker_entry(marker)
        merged_by_id[int(normalized["id"])] = normalized

    return [merged_by_id[marker_id] for marker_id in sorted(merged_by_id)]


def save_marker_map_config(
    config_file: str,
    markers: list[dict[str, Any]],
    map_frame: str = DEFAULT_MAP_FRAME,
    marker_prefix: str = DEFAULT_MARKER_PREFIX,
) -> None:
    path = Path(config_file)
    path.parent.mkdir(parents=True, exist_ok=True)

    normalized_markers = [_normalize_marker_entry(marker) for marker in markers]
    normalized_markers.sort(key=lambda marker: int(marker["id"]))

    output = {
        "map_frame": str(map_frame or DEFAULT_MAP_FRAME),
        "marker_prefix": str(marker_prefix or DEFAULT_MARKER_PREFIX),
        "markers": [
            {
                "id": int(marker["id"]),
                "pose": {
                    "x": float(marker["x"]),
                    "y": float(marker["y"]),
                    "z": float(marker["z"]),
                    "roll": float(marker["roll"]),
                    "pitch": float(marker["pitch"]),
                    "yaw": float(marker["yaw"]),
                },
            }
            for marker in normalized_markers
        ],
    }

    with path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(output, handle, sort_keys=False)
