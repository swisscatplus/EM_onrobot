#!/usr/bin/env python3
import cv2 as cv
import numpy as np

# Create a dictionary + detector
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
detector_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, detector_params)

def detect_aruco_corners(frame):
    """
    Detect ArUco markers in 'frame' (BGR or grayscale).
    Return a list of dicts, each with:
        {
            'id': <marker ID>,
            'corners': numpy array of shape (4,2) with pixel coords
        }
    """
    if len(frame.shape) == 3:
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    else:
        gray_frame = frame

    corners_list, ids, _ = detector.detectMarkers(gray_frame)

    if ids is None or len(ids) == 0:
        return []

    results = []
    for corners, id_ in zip(corners_list, ids.flatten()):
        # corners is shape (1,4,2), so reshape to (4,2)
        c = corners[0]
        results.append({
            'id': int(id_),
            'corners': c
        })
    return results
