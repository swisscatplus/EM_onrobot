import math
from pathlib import Path

from em_robot.marker_map_loader import load_marker_map_config


def test_load_marker_map_supports_flat_entries(tmp_path):
    config_path = tmp_path / "marker_map.yaml"
    config_path.write_text(
        "\n".join(
            [
                "map_frame: world",
                "marker_prefix: marker_",
                "markers:",
                "  - id: 4",
                "    x: 1.0",
                "    y: 2.0",
                "    z: 0.5",
                "    yaw: 1.57",
            ]
        ),
        encoding="utf-8",
    )

    marker_map = load_marker_map_config(str(config_path))

    assert marker_map["map_frame"] == "world"
    assert marker_map["marker_prefix"] == "marker_"
    assert marker_map["markers"][0]["id"] == 4
    assert marker_map["markers"][0]["x"] == 1.0
    assert marker_map["markers"][0]["yaw"] == 1.57


def test_load_marker_map_supports_nested_pose_entries(tmp_path):
    config_path = tmp_path / "marker_map.yaml"
    config_path.write_text(
        "\n".join(
            [
                "markers:",
                "  - id: 7",
                "    pose:",
                "      x: -0.3",
                "      y: 0.4",
                "      roll: 0.1",
                "      pitch: 0.2",
                "      yaw: 0.3",
            ]
        ),
        encoding="utf-8",
    )

    marker_map = load_marker_map_config(str(config_path))

    assert marker_map["map_frame"] == "map"
    assert marker_map["marker_prefix"] == "aruco_"
    assert marker_map["markers"][0]["id"] == 7
    assert marker_map["markers"][0]["roll"] == 0.1
    assert marker_map["markers"][0]["pitch"] == 0.2
    assert marker_map["markers"][0]["yaw"] == 0.3


def test_load_marker_map_rejects_duplicate_ids(tmp_path):
    config_path = tmp_path / "marker_map.yaml"
    config_path.write_text(
        "\n".join(
            [
                "markers:",
                "  - id: 1",
                "  - id: 1",
            ]
        ),
        encoding="utf-8",
    )

    try:
        load_marker_map_config(str(config_path))
    except ValueError as exc:
        assert "Duplicate marker id 1" in str(exc)
    else:
        raise AssertionError(f"Expected duplicate marker id error for {Path(config_path)}")


def test_load_marker_map_supports_legacy_track_hierarchy_with_fixed_ceiling_orientation(tmp_path):
    config_path = tmp_path / "marker_tracks.yaml"
    config_path.write_text(
        "\n".join(
            [
                "mob_rob_loca:",
                "  ros__parameters:",
                "    tracks:",
                '      "18":',
                "        t_x: 0.65",
                "        t_y: 0.0",
                "        yaw: -1.5708",
                "        markers:",
                '          "19":',
                "            t_x: 0.177",
                "            t_y: 0.900",
                "            yaw: 3.1415",
                "            t_z: 0.374",
            ]
        ),
        encoding="utf-8",
    )

    marker_map = load_marker_map_config(str(config_path))
    marker = marker_map["markers"][0]

    assert marker["id"] == 19
    assert math.isclose(marker["x"], 1.55, abs_tol=1e-3)
    assert math.isclose(marker["y"], -0.177, abs_tol=1e-3)
    assert math.isclose(marker["z"], 0.374, abs_tol=1e-9)
    assert math.isclose(marker["roll"], math.pi, abs_tol=1e-9)
    assert math.isclose(marker["pitch"], 0.0, abs_tol=1e-9)
    assert math.isclose(marker["yaw"], 1.5707, abs_tol=1e-4)
