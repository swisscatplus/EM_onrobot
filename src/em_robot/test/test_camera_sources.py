import numpy as np
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from em_robot.camera_sources import monotonic_to_system_time_ns, normalize_to_bgr8


def test_normalize_to_bgr8_converts_bgra_frames_to_bgr():
    frame = np.array([[[10, 20, 30, 255], [40, 50, 60, 255]]], dtype=np.uint8)

    normalized = normalize_to_bgr8(frame)

    assert normalized.shape == (1, 2, 3)
    assert normalized.dtype == np.uint8
    assert normalized.tolist() == [[[10, 20, 30], [40, 50, 60]]]


def test_normalize_to_bgr8_converts_grayscale_frames_to_bgr():
    frame = np.array([[10, 20]], dtype=np.uint8)

    normalized = normalize_to_bgr8(frame)

    assert normalized.shape == (1, 2, 3)
    assert normalized.tolist() == [[[10, 10, 10], [20, 20, 20]]]


def test_monotonic_to_system_time_ns_uses_current_clock_offset(monkeypatch):
    monkeypatch.setattr("time.time_ns", lambda: 1_000_000_000)
    monkeypatch.setattr("time.monotonic_ns", lambda: 100_000_000)

    assert monotonic_to_system_time_ns(90_000_000) == 990_000_000
