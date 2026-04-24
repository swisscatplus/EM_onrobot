from __future__ import annotations

import cv2 as cv
import numpy as np


ARUCO_DICTIONARY_ID = cv.aruco.DICT_ARUCO_ORIGINAL


def create_aruco_detector():
    dictionary = cv.aruco.getPredefinedDictionary(ARUCO_DICTIONARY_ID)
    if hasattr(cv.aruco, "DetectorParameters"):
        detector_params = cv.aruco.DetectorParameters()
    else:  # pragma: no cover - compatibility with older OpenCV
        detector_params = cv.aruco.DetectorParameters_create()

    if hasattr(cv.aruco, "ArucoDetector"):
        return cv.aruco.ArucoDetector(dictionary, detector_params)

    return {
        "dictionary": dictionary,
        "parameters": detector_params,
    }


def detect_aruco_markers(frame, detector=None):
    active_detector = detector if detector is not None else create_aruco_detector()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
    if hasattr(active_detector, "detectMarkers"):
        corners_list, ids, _ = active_detector.detectMarkers(gray)
    else:  # pragma: no cover - compatibility with older OpenCV
        corners_list, ids, _ = cv.aruco.detectMarkers(
            gray,
            active_detector["dictionary"],
            parameters=active_detector["parameters"],
        )
    if ids is None or len(ids) == 0:
        return []

    return [
        {"id": int(marker_id), "corners": corners[0].astype(np.float32)}
        for corners, marker_id in zip(corners_list, ids.flatten())
    ]


def build_marker_object_points(marker_size: float):
    half_marker = float(marker_size) / 2.0
    return np.array(
        [
            [-half_marker, half_marker, 0.0],
            [half_marker, half_marker, 0.0],
            [half_marker, -half_marker, 0.0],
            [-half_marker, -half_marker, 0.0],
        ],
        dtype=np.float32,
    )


def estimate_marker_pose(
    corners,
    marker_object_points,
    camera_matrix,
    dist_coeffs,
    max_reprojection_error_px: float,
):
    success, rvec, tvec = cv.solvePnP(
        marker_object_points,
        corners,
        camera_matrix,
        dist_coeffs,
        flags=cv.SOLVEPNP_IPPE_SQUARE,
    )
    if not success:
        return None, None

    projected_points, _ = cv.projectPoints(
        marker_object_points,
        rvec,
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    reprojection_error = float(
        np.mean(np.linalg.norm(projected_points.reshape(-1, 2) - corners, axis=1))
    )
    if reprojection_error > max_reprojection_error_px:
        return None, reprojection_error

    rotation_matrix, _ = cv.Rodrigues(rvec)
    camera_to_marker = np.eye(4, dtype=np.float64)
    camera_to_marker[:3, :3] = rotation_matrix
    camera_to_marker[:3, 3] = tvec.reshape(3)
    return camera_to_marker, reprojection_error
