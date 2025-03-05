#!/usr/bin/env python3

import cv2 as cv
import numpy as np

# Basic ArUco configuration
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

def detect_markers(frame):
    """
    Detects ArUco markers in 'frame' (BGR or grayscale).
    Returns a list of dictionaries, each with:
      {
        'id':   <marker ID>,
        'x':    <X in pixel coords (or some camera 2D system)>,
        'y':    <Y in pixel coords (or some camera 2D system)>,
        'yaw':  <orientation in radians, in the camera plane>
      }
    """

    # 1) Convert to grayscale if needed
    if len(frame.shape) == 3:
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    else:
        gray_frame = frame

    # 2) Detect markers
    markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)

    results = []
    if markerIds is None or len(markerIds) == 0:
        return results

    # 3) Compute a simple 2D (x,y,yaw) for each marker.
    #    - x, y: just the pixel center
    #    - yaw: from center to the marker's "top side"
    #      (completely arbitrary, just to illustrate we can define an orientation.)
    for corners, id_ in zip(markerCorners, markerIds.flatten()):
        pts = corners[0]  # shape (4,2)

        # Marker center in pixel coords
        center_x = np.mean(pts[:,0])
        center_y = np.mean(pts[:,1])

        # "Top" side: midpoint between corners[0] & corners[1]
        top_x = np.mean(pts[0:2,0])
        top_y = np.mean(pts[0:2,1])

        dx = top_x - center_x
        dy = top_y - center_y
        yaw = np.arctan2(dy, dx)  # angle in radians from +x axis (pixel coords)

        results.append({
            'id':   int(id_),
            'x':    center_x,
            'y':    center_y,
            'yaw':  yaw
        })

    return results
