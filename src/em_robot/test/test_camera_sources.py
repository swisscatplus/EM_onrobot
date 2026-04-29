import numpy as np

from em_robot.camera_sources import normalize_to_bgr8


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
