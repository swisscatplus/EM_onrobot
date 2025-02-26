import cv2 as cv
import numpy as np
import logging

# ArUco marker detection setup
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

CONV_RAD2DEG = 180 / np.pi  # Conversion factor from radians to degrees

class CameraVisionStation:
    """
    Handles ArUco marker detection and position estimation.
    Can operate with or without calibration data.
    """

    def __init__(self, cam_params=None, aruco_params=None, cam_frame=(640, 480)):
        # Use provided parameters or empty dictionaries if None
        self.cam_config = cam_params if cam_params is not None else {}
        self.aruco_params = aruco_params if aruco_params is not None else {}

        # Physical marker parameters from config
        self.aruco_square_size = self.cam_config.get('aruco_square_size', 0.036)
        self.dist_cam_robot_center = self.cam_config.get('dist_cam_robot_center', 0)

        self.size = cam_frame  # (width, height) of the image

        self.configure_logger()
        self.logger.info("CameraVisionStation initialized.")
        self.logger.info(f"AruCo Square Size: {self.aruco_square_size} m, Camera-Robot Center Distance: {self.dist_cam_robot_center} m")

    def configure_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        # Avoid adding multiple handlers if already configured
        if not self.logger.hasHandlers():
            self.logger.addHandler(ch)

    def get_robot_pose(self, frame, markerCorners, markerIds):
        """
        Computes the camera's (robot's) position in the map and the marker's orientation.

        It performs the following:
          1. For the first valid marker detected, compute its center (pixel coordinates).
          2. Convert the pixel offset (from the optical center) into meters using the pinhole model.
             Defaults are used if calibration data is not provided.
          3. Retrieve the marker's known map position from configuration.
          4. Compute the camera (robot) position on the map.
          5. Compute the marker's orientation based on its top and bottom edges.

        Returns:
          (camera_map_position, rad_angle)
            - camera_map_position: [x, y] in meters on the map.
            - rad_angle: marker orientation in radians.
          If no valid marker is found, returns (None, None).
        """
        if markerIds is None or len(markerIds) == 0:
            self.logger.info("No ArUco markers detected.")
            return None, None

        self.logger.info(f"Detected {len(markerIds)} marker(s): {markerIds.flatten().tolist()}")

        # Process the first valid marker from config
        for i in range(len(markerIds)):
            marker_id = markerIds[i, 0]
            if marker_id not in self.aruco_params:
                self.logger.info(f"Marker ID {marker_id} not in configuration. Skipping...")
                continue

            # Extract corners (assumed shape: (1, 4, 2))
            corners = markerCorners[i][0]
            marker_center = np.mean(corners, axis=0)

            # Retrieve calibration parameters or use defaults:
            camera_height = self.cam_config.get('camera_height', 1.0)
            fx = self.cam_config.get('fx', 800.0)
            fy = self.cam_config.get('fy', 800.0)
            cx = self.cam_config.get('principal_x', self.size[0] / 2)
            cy = self.cam_config.get('principal_y', self.size[1] / 2)

            # Compute pixel offset from optical center
            pixel_offset = marker_center - np.array([cx, cy])
            # Convert offset to meters using the pinhole camera model:
            offset_x = pixel_offset[0] * (camera_height / fx)
            offset_y = pixel_offset[1] * (camera_height / fy)
            offset_meters = np.array([offset_x, offset_y])

            # Retrieve the marker's known map position from configuration
            marker_map_position = np.array([
                self.aruco_params[marker_id]['t_x'],
                self.aruco_params[marker_id]['t_y']
            ])

            # Compute the camera (robot) position on the map:
            # If the marker appears offset, the camera is at the marker's position minus that offset.
            camera_map_position = marker_map_position - offset_meters

            # Compute marker orientation using its top and bottom edges
            top_center = np.mean(corners[0:2], axis=0)
            bottom_center = np.mean(corners[2:4], axis=0)
            dx, dy = top_center - bottom_center
            angle_deg = np.arctan2(dy, dx) * CONV_RAD2DEG
            rad_angle = np.deg2rad(angle_deg)

            return camera_map_position, rad_angle

        self.logger.info("No valid markers processed for pose estimation.")
        return None, None
