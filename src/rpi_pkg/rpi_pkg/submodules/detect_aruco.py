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
    def __init__(self, config=None, cam_frame=(447, 569)):
        """
        Class constructor to set up the right camera
        """

        self.cam_config = config['cam_params']
        self.focal_length = self.cam_config['focal_length']
        self.aruco_ids = config['aruco_params']
        self.pxl2meter = None #self.cam_config['conv_pxl2met']
        self.size = cam_frame
        self.pixels_to_m = None
        
        self.configure_logger()
        self.logger.info(f'CameraVisionStation initialized with {self.pxl2meter} pxl2meter conv factor and {self.size} frame size.')

        self.conv_list = []

    def pixels_to_meters(self, markerCorners):
        """
        Computes distances of all sides in pixels, should be replaced by a constant computed in the constructor.
        """
        distances = []
        for corners in markerCorners:
            for i in range(4):
                p1 = corners[0][i%4]
                p2 = corners[0][(i + 1) % 4]
                distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                distances.append(distance)

        # Compute average (or median) distance in pixels
        avg_distance_pixels = np.mean(distances)
        pixels_to_m = self.cam_config['aruco_square_size'] / avg_distance_pixels
        # self.conv_list.append(pixels_to_m)
        # if len(self.conv_list) == 300:
        #     mean = np.mean(self.conv_list)
        #     median = np.median(self.conv_list)
        #     std = np.std(self.conv_list)
        #     self.logger.info(f'pixels_to_meters: {mean}, {median}. {std}')
        #     self.conv_list = []
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
        t_x, t_y, yaw = self.aruco_ids[id]['t_x'], self.aruco_ids[id]['t_y'], self.aruco_ids[id]['yaw']
        theta = theta + yaw
        # self.logger.debug(f"aruco_pxl_c: {aruco_pxl_c}, id: {id}, theta: {theta}")

        # Compute the offset in the camera frame
        delta_x = aruco_pxl_c[0] - self.size[0] / 2
        delta_y = self.size[1] / 2 - aruco_pxl_c[1]

        # Convert pixels to meters
        delta_x = delta_x * self.pxl2meter 
        delta_y = delta_y * self.pxl2meter
        # self.logger.debug(f'delta_x: {delta_x}, delta_y: {delta_y}')

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
            if marker_id in self.aruco_ids:
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
                ]) * self.cam_config['dist_cam_robot_center']

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

        if aruco_poses:
            robot_center = np.mean(aruco_poses, axis=0)
            robot_angle = np.mean(robot_angles)
        else:
            robot_center, robot_angle = None, None

        return robot_center, robot_angle