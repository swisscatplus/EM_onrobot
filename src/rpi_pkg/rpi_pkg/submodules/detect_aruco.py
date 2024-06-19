#!/usr/bin/python3
import cv2 as cv
import numpy as np
import logging
import yaml
import os

################# CONFIG #################
# local config to run the code independently
camera_port = 0 # worked for me, use 0 if you want to use the embedded wembcam of the computer, or try other numbers
window_name = 'Robot poses'
verbose = False
# station_id = 'station_1'

# directory_path = os.path.dirname(os.path.abspath(__file__))
# config_file_path = os.path.join(directory_path, '../..', 'config', 'rpi_cam.yaml')

def get_cam_config(config_file_path=None):
    with open(config_file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

# config = get_cam_config(config_file_path=config_file_path)
# ########################################

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

class CameraVisionStation:
    """
    Create an ImagePublisher class, which is a subclass of the Node class.
    """
    def __init__(self, config=None):
        """
        Class constructor to set up the right camera
        """

        self.cam_config = config['cam_params']
        self.focal_length = self.cam_config['focal_length']
        self.aruco_ids = config['aruco_params']

        self.pixels_to_m = None
        self.pxl_max = None

        self.configure_logger()

    def pixels_to_meters(self, markerCorners):
        # Compute distances of all sides in pixels, should be replaced by a conostant computed in the init
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

        return pixels_to_m
    
    def configure_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        ch = logging.StreamHandler()

        ch.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

    def compute_camera_center(self, aruco_pxl_c, id, theta, pxl_max_x, pxl_max_y):
        t_x, t_y, yaw = self.aruco_ids[id]['t_x'], self.aruco_ids[id]['t_y'], self.aruco_ids[id]['yaw']
        theta = theta + yaw
        self.logger.debug(f"aruco_pxl_c: {aruco_pxl_c}, id: {id}, theta: {theta}")
        # Compute the offset in the camera frame
        delta_x = aruco_pxl_c[0] - pxl_max_x / 2
        delta_y = pxl_max_y / 2 - aruco_pxl_c[1]
        delta_x = delta_x * self.pixels_to_m  # Convert pixels to meters
        delta_y = delta_y * self.pixels_to_m  # Convert pixels to meters
        self.logger.debug(f'delta_x: {delta_x}, delta_y: {delta_y}')
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0.0],
            [-np.sin(theta), -np.cos(theta), 0.0],
            [0.0, 0.0, 1.0]
        ])
        
        delta_XY = R @ np.array([delta_x, delta_y, 0.0])
        self.logger.debug(f'delta_XY: {delta_XY}')
        X_camera = t_x - delta_XY[0]
        Y_camera = t_y - delta_XY[1]

        return X_camera, Y_camera

    # Function to process the frame and detect ArUco markers
    def get_robot_pose(self, frame, markerCorners, markerIds, set_visual_interface=False):
        pxl_max_y, pxl_max_x, _ = frame.shape
        self.pixels_to_m = self.pixels_to_meters(markerCorners)  # Constant conversion factor
        self.logger.debug(f'pixels_to_m: {self.pixels_to_m}')
        aruco_poses = []
        robot_angles = []

        for i in range(len(markerIds)):
            marker_id = markerIds[i, 0]
            if marker_id == 0:
                continue
            
            if marker_id in self.aruco_ids:
                corners = markerCorners[i][0]
                bottom_center = tuple(map(int, np.mean(corners[2:4], axis=0)))
                top_center = tuple(map(int, np.mean(corners[0:2], axis=0)))
                center_code = tuple(map(int, np.mean(corners, axis=0)))

                dx = top_center[0] - bottom_center[0]
                dy = top_center[1] - bottom_center[1]
                angle = np.arctan2(dy, dx) * 180 / np.pi
                rad_angle = np.deg2rad(angle + 180)
                coord_cam_circuit = self.compute_camera_center(aruco_pxl_c=center_code, id=markerIds[i, 0], theta=rad_angle, pxl_max_x=pxl_max_x, pxl_max_y=pxl_max_y)

                robot_center = coord_cam_circuit[:2]- np.array([
                    np.cos(-rad_angle), np.sin(-rad_angle)
                ]) * self.cam_config['dist_cam_robot_center']
                aruco_poses.append(robot_center)
                robot_angles.append(-rad_angle)

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
            else:
                self.logger.warn(f'Unknown marker ID detected: {marker_id}, ignoring')

        if aruco_poses:
            robot_center = np.mean(aruco_poses, axis=0)
            robot_angle = np.mean(robot_angles)
        else:
            robot_center, robot_angle = None, None

        return robot_center, robot_angle

def main():


    directory_path = os.path.dirname(os.path.abspath(__file__))
    config_file_path = '/home/coderey/Desktop/SwissCat-on_robot/src/rpi_pkg/config/cam.yaml'

    config = get_cam_config(config_file_path=config_file_path)

    cap = cv.VideoCapture(camera_port)
    cap.set(cv.CAP_PROP_AUTOFOCUS, 0) #remove autofocus
    cam = CameraVisionStation(config=config)

    while True:
        ret, frame = cap.read()
        
        if not ret:
            raise ("Can't receive frame (stream end?). Exiting ...")
        
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert frame to grayscale
        
        markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)  # Detect markers in grayscale frame

        if markerIds is not None:
            robot_poses = cam.get_robot_pose(frame, markerCorners, markerIds, set_visual_interface=True)
            if verbose:
                cam.logger.info('robot_poses: ' + str(robot_poses))
                cam.logger.info('len(robot_poses): ' + str(len(robot_poses)))
                cam.logger.info('---')
                for robot_pose in robot_poses:
                    cam.logger.info('robot_pose: ' + str(robot_pose))
                    cam.logger.info('ID:' +  str(robot_pose[0]))
                    cam.logger.info('-------------------------------')
        cv.imshow(window_name, frame)

        if cv.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    main()