import cv2 as cv
import numpy as np
import logging

# ArUco marker detection setup
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

CONV_RAD2DEG = 180 / np.pi  # Conversion from radians to degrees


class CameraVisionStation:
    """
    Handles ArUco marker detection and position estimation.
    """

    def __init__(self, cam_params=None, aruco_params=None, cam_frame=(640, 480)):
        self.cam_config = cam_params
        self.aruco_params = aruco_params
        self.aruco_square_size = self.cam_config.get('aruco_square_size', 0.038)  # ArUco marker size in meters
        self.dist_cam_robot_center = self.cam_config.get('dist_cam_robot_center', 0.098)

        self.size = cam_frame
        self.pxl2meter = None  # Conversion factor from pixels to meters
        self.configure_logger()

        self.logger.info("CameraVisionStation initialized.")
        self.logger.info(f"AruCo Square Size: {self.aruco_square_size} meters")
        self.logger.info(f"Distance from Camera to Robot Center: {self.dist_cam_robot_center} meters")

    def configure_logger(self):
        """Set up the logger for debugging."""
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)  # Set to DEBUG for detailed logging
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        self.logger.addHandler(ch)

    def pixels_to_meters(self, markerCorners):
        """
        Computes pixel-to-meter conversion factor based on the size of detected ArUco markers.
        """
        if not markerCorners:
            self.logger.warning("No marker corners received for pixel-to-meter conversion.")
            return None

        distances = [np.linalg.norm(corners[0][i % 4] - corners[0][(i + 1) % 4]) for corners in markerCorners for i in range(4)]
        avg_distance_pixels = np.mean(distances)
        if avg_distance_pixels == 0:
            self.logger.error("Average pixel distance is zero! Possible detection issue.")
            return None

        self.pxl2meter = self.aruco_square_size / avg_distance_pixels
        self.logger.debug(f"Pixel-to-meter conversion factor: {self.pxl2meter:.6f}")
        return self.pxl2meter

    def get_robot_pose(self, frame, markerCorners, markerIds):
        """
        Computes the robot’s position and orientation based on detected ArUco markers.
        """
        if markerIds is None or len(markerIds) == 0:
            self.logger.warning("No ArUco markers detected.")
            return None, None

        self.logger.info(f"Detected {len(markerIds)} ArUco markers: {markerIds.flatten().tolist()}")

        self.pxl2meter = self.pixels_to_meters(markerCorners)
        if self.pxl2meter is None:
            self.logger.error("Pixel-to-meter conversion failed.")
            return None, None

        robot_positions = []
        robot_angles = []

        for i in range(len(markerIds)):
            marker_id = markerIds[i, 0]
            if marker_id not in self.aruco_params:
                self.logger.warning(f"AruCo ID {marker_id} not found in parameters. Skipping...")
                continue

            self.logger.debug(f"Processing marker ID {marker_id}...")

            # Extract marker corners
            corners = markerCorners[i][0]
            bottom_center = np.mean(corners[2:4], axis=0).astype(int)
            top_center = np.mean(corners[0:2], axis=0).astype(int)

            # Compute orientation
            dx, dy = top_center - bottom_center
            angle = np.arctan2(dy, dx) * CONV_RAD2DEG
            rad_angle = np.deg2rad(angle + 180 + 90)
            #self.logger.debug(f"Marker {marker_id} Angle: {angle:.2f} degrees ({rad_angle:.4f} rad)")

            # Compute robot position based on known ArUco map positions
            aruco_x, aruco_y = self.aruco_params[marker_id]['t_x'], self.aruco_params[marker_id]['t_y']
            robot_x = aruco_x - np.cos(-rad_angle) * self.dist_cam_robot_center
            robot_y = aruco_y - np.sin(-rad_angle) * self.dist_cam_robot_center

            robot_positions.append([robot_x, robot_y])
            robot_angles.append(-rad_angle)

            self.logger.info(f"Computed Robot Position from Marker {marker_id}: X={robot_x:.4f}, Y={robot_y:.4f}")

        if robot_positions:
            mean_position = np.mean(robot_positions, axis=0)
            mean_angle = np.mean(robot_angles)
            self.logger.info(f"Final Estimated Robot Pose: X={mean_position[0]:.4f}, Y={mean_position[1]:.4f}, Angle={mean_angle:.4f} rad")
            return mean_position, mean_angle

        self.logger.error("No valid ArUco markers used for robot pose estimation.")
        return None, None
