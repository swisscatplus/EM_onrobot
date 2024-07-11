#!/usr/bin/python3
import cv2 as cv
import numpy as np
import logging

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

CONV_RAD2DEG = 180 / np.pi

class CameraVisionStation:
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self, cam_params=None, aruco_params=None, cam_frame=(447, 569)):
        """
        Class constructor to set up the right camera
        """

        self.cam_config = cam_params
        self.aruco_params = aruco_params
        self.pxl2meter = None
        self.size = cam_frame
        self.pixels_to_m = None
        self.aruco_square_size = self.cam_config.get('aruco_square_size', 0.038)
        self.dist_cam_robot_center = self.cam_config.get('dist_cam_robot_center', 0.098)
        
        self.configure_logger()

        self.logger.info(f"Initialised with camera parameters: {self.cam_config}")
        ids = list(self.aruco_params.keys())
        self.logger.info(f"Aruco ids: {ids}")

        self.conv_list = []

    def pixels_to_meters(self, markerCorners):
        """
        Computes distances of all sides in pixels, should be replaced by a constant computed in the constructor.
        """
        # tried setting a fixed pixels_to_m value, but since every ArUco code is located at a slightly different height, would need to compute it for each code
        distances = []
        for corners in markerCorners:
            for i in range(4):
                p1 = corners[0][i%4]
                p2 = corners[0][(i + 1) % 4]
                distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                distances.append(distance)

        # Compute average (or median) distance in pixels
        avg_distance_pixels = np.mean(distances)
        pixels_to_m = self.aruco_square_size / avg_distance_pixels

        return pixels_to_m
    
    def configure_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()

        ch.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

    def compute_camera_center(self, aruco_pxl_c, id, theta):
        # Gets the pose of the aruco in the circuit
        t_x, t_y, yaw = self.aruco_params[id]['t_x'], self.aruco_params[id]['t_y'], self.aruco_params[id]['yaw']
        theta = theta + yaw

        # Compute the offset in the camera frame
        delta_x = aruco_pxl_c[0] - self.size[0] / 2
        delta_y = self.size[1] / 2 - aruco_pxl_c[1]

        # Convert pixels to meters
        delta_x = delta_x * self.pxl2meter 
        delta_y = delta_y * self.pxl2meter

        # Rotation matrix empirically defined
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0.0],
            [-np.sin(theta), -np.cos(theta), 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        delta_XY = R @ np.array([delta_x, delta_y, 0.0])
        # self.logger.debug(f'delta_XY: {delta_XY}')
        
        X_camera = t_x - delta_XY[0]
        Y_camera = t_y - delta_XY[1]

        return X_camera, Y_camera

    # Function to process the frame and detect ArUco markers
    def get_robot_pose(self, frame, markerCorners, markerIds, set_visual_interface=False):
        self.pxl2meter = self.pixels_to_meters(markerCorners)  # Constant conversion factor
        self.logger.debug(f'pixels_to_m: {self.pixels_to_m}')
        aruco_poses = []
        robot_angles = []

        for i in range(len(markerIds)):
            marker_id = markerIds[i, 0]
            if marker_id in self.aruco_params:
                # Gets center pixel of the aruco code detected
                corners = markerCorners[i][0]
                bottom_center = tuple(map(int, np.mean(corners[2:4], axis=0)))
                top_center = tuple(map(int, np.mean(corners[0:2], axis=0)))
                center_code = tuple(map(int, np.mean(corners, axis=0)))

                # Gets its angle
                dx = top_center[0] - bottom_center[0]
                dy = top_center[1] - bottom_center[1]
                angle = np.arctan2(dy, dx) * CONV_RAD2DEG
                rad_angle = np.deg2rad(angle + 180)

                # Computes the camera center in the circuit frame
                coord_cam_circuit = self.compute_camera_center(aruco_pxl_c=center_code, id=markerIds[i, 0], theta=rad_angle)

                # Computes robot center relative to the camera center
                robot_center = coord_cam_circuit[:2]- np.array([
                    np.cos(-rad_angle), np.sin(-rad_angle)
                ]) * self.dist_cam_robot_center

                aruco_poses.append(robot_center)
                robot_angles.append(-rad_angle) # needed to make the robot rotate in the good orientation
                self.logger.debug(f'robot_center: {robot_center}, robot_angle: {-rad_angle}')

                if set_visual_interface:
                    cv.arrowedLine(frame, bottom_center, top_center, (0, 0, 255), 4)
                    cv.circle(frame, center_code, 3, (255, 0, 0), -1)
                    text = (
                        f"ID {marker_id}: ang: {angle:.1f} deg, "
                        f"robot_pose: {robot_center}, cam_pose: {coord_cam_circuit[:2]}"
                    )
                    cv.putText(frame, text, (10, frame.shape[0] - 20 - i * 25), 
                            cv.FONT_HERSHEY_COMPLEX, 0.3, (255, 255, 255), 1)
                    cv.circle(frame, (int(robot_center[0]), int(robot_center[1])), 3, (255, 0, 0), -1)
            # not using else statemenent, the log was smh introducing noise in the position of the robot
            # uncomment following two lines if you want to see the aruco id
            # else:
            #     self.logger.warn(f"ArUco ID {marker_id} not in the list of known IDs.")
        if aruco_poses:
            robot_center = np.mean(aruco_poses, axis=0)
            robot_angle = np.mean(robot_angles)
        else:
            robot_center, robot_angle = None, None

        return robot_center, robot_angle