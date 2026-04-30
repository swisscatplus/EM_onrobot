# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from em_robot.marker_map_io import merge_marker_entries, save_marker_map_config
from em_robot.marker_map_loader import load_marker_map_config


def test_merge_marker_entries_updates_existing_marker_and_sorts_ids():
    merged = merge_marker_entries(
        existing_markers=[
            {"id": 5, "x": 5.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            {"id": 2, "x": 2.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        ],
        updated_markers=[
            {"id": 5, "x": 5.5, "y": 0.1, "z": 0.2, "roll": 0.0, "pitch": 0.1, "yaw": 0.2},
            {"id": 7, "x": 7.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        ],
    )

    assert [marker["id"] for marker in merged] == [2, 5, 7]
    assert merged[1]["x"] == 5.5
    assert merged[1]["pitch"] == 0.1


def test_save_marker_map_config_writes_loader_compatible_yaml(tmp_path):
    config_path = tmp_path / "marker_map.yaml"

    save_marker_map_config(
        str(config_path),
        markers=[
            {"id": 3, "x": 1.0, "y": 2.0, "z": 0.3, "roll": 3.14, "pitch": 0.0, "yaw": 1.57}
        ],
        map_frame="map",
        marker_prefix="aruco_",
    )

    loaded = load_marker_map_config(str(config_path))

    assert loaded["map_frame"] == "map"
    assert loaded["marker_prefix"] == "aruco_"
    assert loaded["markers"][0]["id"] == 3
    assert loaded["markers"][0]["z"] == 0.3
