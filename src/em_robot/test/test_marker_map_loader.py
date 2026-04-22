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
