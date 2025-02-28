import cv2 as cv
import numpy as np
import logging

# Global variable to activate/deactivate logs
ENABLE_LOGS = True

# Dummy logger that does nothing when logs are disabled
class DummyLogger:
    def info(self, msg): pass
    def debug(self, msg): pass
    def warn(self, msg): pass
    def error(self, msg): pass

# ArUco marker detection setup
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

CONV_RAD2DEG = 180 / np.pi  # Conversion factor from radians to degrees


class CameraVisionStation:
    """
    Handles ArUco marker detection, image undistortion/resizing,
    and position estimation.
    Can operate with or without calibration data.
    """

    def __init__(self, cam_params=None, aruco_params=None, cam_frame=(4608, 2592)):
        # Use provided parameters or empty dictionaries if None.
        self.cam_config = cam_params if cam_params is not None else {}
        self.aruco_params = aruco_params if aruco_params is not None else {}

        # Physical marker parameters from config.
        self.aruco_square_size = self.cam_config.get('aruco_square_size', 0.036)
        self.dist_cam_robot_center = self.cam_config.get('dist_cam_robot_center', 0)

        self.size = cam_frame  # Original frame size (e.g., used during calibration).
        self.img_scale = self.cam_config.get("img_scale", 1.0)  # Scaling factor for resizing.

        self.configure_logger()
        # Override logger with dummy if logs are disabled
        if not ENABLE_LOGS:
            self.logger = DummyLogger()

        self.logger.info("CameraVisionStation initialized.")
        self.logger.info(
            f"AruCo Square Size: {self.aruco_square_size} m, Camera-Robot Center Distance: {self.dist_cam_robot_center} m")
        self.logger.info(f"Image scale factor: {self.img_scale}")

    def configure_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        # Avoid adding multiple handlers if already configured.
        if not self.logger.hasHandlers():
            self.logger.addHandler(ch)

    def undistort_and_resize(self, frame):
        """
        Undistorts the input frame using calibration data if available,
        then resizes it according to the img_scale factor.
        """
        if 'camera_matrix' in self.cam_config and 'dist_coeff' in self.cam_config:
            camera_matrix = np.array(self.cam_config['camera_matrix'])
            dist_coeff = np.array(self.cam_config['dist_coeff'])
            h, w = frame.shape[:2]
            new_camera_matrix, roi = cv.getOptimalNewCameraMatrix(camera_matrix, dist_coeff, (w, h), 1, (w, h))
            undistorted = cv.undistort(frame, camera_matrix, dist_coeff, None, new_camera_matrix)
            self.logger.info("Frame undistorted using calibration data.")
        else:
            undistorted = frame
            self.logger.info("No calibration data provided; using original frame.")

        if self.img_scale != 1.0:
            new_size = (int(undistorted.shape[1] * self.img_scale), int(undistorted.shape[0] * self.img_scale))
            undistorted = cv.resize(undistorted, new_size, interpolation=cv.INTER_AREA)
            self.logger.info(f"Frame resized to: {new_size}")
        return undistorted

    def get_robot_pose(self, frame, markerCorners, markerIds):
        """
        Computes the camera's (robot's) position on the map and the marker's orientation.
        The provided frame is first undistorted and resized.

        Returns:
          (camera_map_position, rad_angle_corrected)
            - camera_map_position: [x, y] in meters on the map.
            - rad_angle_corrected: marker orientation (in radians) after applying the offset.
          If no valid marker is found, returns (None, None).
        """
        # Undistort and resize the frame before processing.
        frame = self.undistort_and_resize(frame)

        if markerIds is None or len(markerIds) == 0:
            self.logger.info("No ArUco markers detected.")
            return None, None

        self.logger.info(f"Detected {len(markerIds)} marker(s): {markerIds.flatten().tolist()}")

        # Process the first valid marker found.
        for i in range(len(markerIds)):
            marker_id = markerIds[i, 0]
            if marker_id not in self.aruco_params:
                self.logger.info(f"Marker ID {marker_id} not in configuration. Skipping...")
                continue

            # Extract marker corners (assumed shape: (1, 4, 2)).
            corners = markerCorners[i][0]
            marker_center = np.mean(corners, axis=0)

            # Retrieve calibration parameters or use defaults.
            camera_height = self.cam_config.get('camera_height', 1.0)
            fx = self.cam_config.get('fx', 800.0)
            fy = self.cam_config.get('fy', 800.0)
            cx = self.cam_config.get('principal_x', self.size[0] / 2)
            cy = self.cam_config.get('principal_y', self.size[1] / 2)

            # Adjust intrinsic parameters for the resized image.
            fx *= self.img_scale
            fy *= self.img_scale
            cx *= self.img_scale
            cy *= self.img_scale

            # Compute pixel offset from the optical center.
            pixel_offset = marker_center - np.array([cx, cy])
            # Convert pixel offset to meters using the pinhole model.
            offset_x = pixel_offset[0] * (camera_height / fx)
            offset_y = pixel_offset[1] * (camera_height / fy)
            offset_meters = np.array([offset_x, offset_y])
            self.logger.info(f"Offset in meters: {offset_meters}")

            # Retrieve the marker's known map position from configuration.
            marker_map_position = np.array([
                self.aruco_params[marker_id]['t_x'],
                self.aruco_params[marker_id]['t_y']
            ])
            self.logger.info(f"Marker {marker_id} map position: {marker_map_position}")

            # Compute the camera's position on the map.
            camera_map_position = marker_map_position - offset_meters
            self.logger.info(f"Computed camera position on map: {camera_map_position}")

            # Compute marker orientation using its top and bottom edges.
            top_center = np.mean(corners[0:2], axis=0)
            bottom_center = np.mean(corners[2:4], axis=0)
            dx, dy = top_center - bottom_center
            angle_deg = np.arctan2(dy, dx) * CONV_RAD2DEG
            rad_angle = np.deg2rad(angle_deg)
            self.logger.info(f"Marker {marker_id} raw angle: {angle_deg:.2f}° ({rad_angle:.4f} rad)")

            # Retrieve the angle offset from the configuration (in radians).
            angle_offset = self.cam_config.get('angle_offset', 0.0)
            rad_angle_corrected = rad_angle + angle_offset
            self.logger.info(
                f"Marker {marker_id} corrected angle: {np.rad2deg(rad_angle_corrected):.2f}° ({rad_angle_corrected:.4f} rad)")

            return camera_map_position, rad_angle_corrected

        self.logger.info("No valid markers processed for pose estimation.")
        return None, None
