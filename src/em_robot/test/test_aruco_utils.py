# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import cv2 as cv
import numpy as np

from em_robot.aruco_utils import (
    build_marker_object_points,
    detect_aruco_markers,
    estimate_marker_pose,
)


class FakeDetector:
    def detectMarkers(self, gray):
        del gray
        return [
            np.array(
                [[[10.0, 20.0], [30.0, 20.0], [30.0, 40.0], [10.0, 40.0]]],
                dtype=np.float32,
            )
        ], np.array([[0]], dtype=np.int32), []


def test_detect_aruco_markers_normalizes_detector_output():
    frame = np.zeros((80, 80, 3), dtype=np.uint8)

    detections = detect_aruco_markers(frame, detector=FakeDetector())

    assert len(detections) == 1
    assert detections[0]["id"] == 0
    assert detections[0]["corners"].shape == (4, 2)


def test_estimate_marker_pose_recovers_synthetic_projection():
    marker_object_points = build_marker_object_points(0.038)
    camera_matrix = np.array(
        [
            [840.0, 0.0, 640.0],
            [0.0, 840.0, 360.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    expected_rvec = np.array([[0.0], [0.0], [0.0]], dtype=np.float32)
    expected_tvec = np.array([[0.0], [0.0], [0.6]], dtype=np.float32)

    projected_points, _ = cv.projectPoints(
        marker_object_points,
        expected_rvec,
        expected_tvec,
        camera_matrix,
        dist_coeffs,
    )
    corners = projected_points.reshape(-1, 2).astype(np.float32)

    camera_to_marker, reprojection_error = estimate_marker_pose(
        corners,
        marker_object_points,
        camera_matrix,
        dist_coeffs,
        max_reprojection_error_px=1.0,
    )

    assert camera_to_marker is not None
    assert reprojection_error is not None
    assert abs(camera_to_marker[0, 3]) < 1e-3
    assert abs(camera_to_marker[1, 3]) < 1e-3
    assert abs(camera_to_marker[2, 3] - 0.6) < 1e-2
