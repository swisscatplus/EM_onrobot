#!/usr/bin/python3

import time
import cv2 as cv
import numpy as np
import yaml
import logging
from picamera2 import Picamera2
from libcamera import controls

window_name = 'Robot poses'
verbose = False
# ##########################################

# ARUCO_DICT = {
# 	"DICT_ARUCO_ORIGINAL": cv.aruco.DICT_ARUCO_ORIGINAL
# }

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
        # self.station_id = station_id
        # self.station_config = config['rpi_cam_parameters'][self.station_id] # structure of camera_config: {'height': 100, 't_x': 10, 't_y': 20, 'r_z': 30}
        self.cam_config = config['cam_params']
        self.focal_length = self.cam_config['focal_length']
        # self.mat_config = self.station_config['transformation_matrix']
        self.aruco_ids = config['aruco_params'] # gets an array of known ids on circuit, to avoid detecting random ids due to noise

        # Create a logger
        self.configure_logger()
        
    def get_cam_config(self):
      with open(self.config_file_path, 'r') as file:
          data = yaml.safe_load(file)
      return data['rpi_cam_parameters'], data['robot_params']
    
    def pixels_to_meters(self, markerCorners):
        # Compute distances of all sides in pixels, should be replaced by constant but since we don't have yet the real value, we compute it
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
        
    def transfo_cam2circuit(self, t_vec):
        coord_cam_frame = np.array([0.0, 0.0, 0.0, 1.0], dtype=object)
        translation_vector = np.array([t_vec[0], t_vec[1], 0.0, 1.0], dtype=object) # assuming no translation in z

        # theta = self.mat_config['theta']
        theta=0.0
        rotation_matrix = np.array([[1, 0, 0], 
                                    [0, 1, 0], 
                                    [0, 0, 1],
                                    [0, 0, 0]])
        
        homogeneous_matrix = np.concatenate((rotation_matrix, translation_vector.reshape(-1,1)), axis=1)
        # print('hm :', homogeneous_matrix)
        self.logger.debug('homogeneous_matrix {}'.format(homogeneous_matrix))
        coord_circuit_frame= homogeneous_matrix @ coord_cam_frame
        return coord_circuit_frame[:2]
    
    def configure_logger(self):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        ch = logging.StreamHandler()

        ch.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)

    def get_robot_poses(self, frame, markerCorners, markerIds=None, set_visual_interface=False):
        pxl_max_y, pxl_max_x, _ = frame.shape
        pxl_center_cam = (pxl_max_x // 2, pxl_max_y // 2)
        pixels_to_m = self.pixels_to_meters(markerCorners)  # Constant conversion factor
        robot_pose = None
        aruco_infos = []

        # Draw and process detected markers
        for i in range(len(markerIds)):
            if markerIds[i, 0] == 0:
                continue
            if markerIds[i, 0] in self.aruco_ids:
                # Calculate the centers of the bottom and top edges of the marker
                bottom_center = tuple(map(int, np.mean(markerCorners[i][0][2:4], axis=0)))
                top_center = tuple(map(int, np.mean(markerCorners[i][0][0:2], axis=0)))
                mean_center = np.mean(markerCorners[i][0], axis=0)
                center_code = tuple(map(int, mean_center))

                dx = top_center[0] - bottom_center[0]
                dy = top_center[1] - bottom_center[1]
                angle = np.arctan2(dy, dx) * 180 / np.pi

                # Calculate the displacement of the ArUco center from the camera center in meters
                delta_center_x = (center_code[0] - pxl_center_cam[0]) * pixels_to_m
                delta_center_y = (center_code[1] - pxl_center_cam[1]) * pixels_to_m
                delta_center = (delta_center_x, -delta_center_y)

                rad_angle = np.deg2rad(angle)
                t_cam = [
                    self.aruco_ids[markerIds[i, 0]]['t_x'] - delta_center[0],
                    self.aruco_ids[markerIds[i, 0]]['t_y'] - delta_center[1]
                ]

                # Transform the camera frame coordinates to the circuit frame
                # coord_circuit_frame = self.transfo_cam2circuit(t_cam)

                # print('ccf :', coord_circuit_frame)
                print('t_cam :', t_cam)
                # Calculate the robot center position in the circuit frame, should be done by urdf once everything working
                robot_center = t_cam[:2] + np.array(
                    [np.cos(rad_angle), np.sin(rad_angle)], dtype=object
                ) * self.cam_config['dist_cam_robot_center']
                aruco_infos.append((robot_center, -rad_angle))

                # print('cam_center', coord_circuit_frame[:2])
                # print('robot_center: {0}, robot_angle {1}'.format(robot_center, angle))
                if set_visual_interface:
                    # Draw arrow from bottom to top center
                    cv.arrowedLine(frame, bottom_center, top_center, (0, 0, 255), 4)
                    # Draw marker ID and center
                    cv.circle(frame, center_code, 3, (255, 0, 0), -1)
                    # Display orientation and position at the bottom of the screen
                    text = f"ID {markerIds[i][0]}: pxl pos: {bottom_center}, robot_pose: {robot_center}, ang: {angle:.1f} deg"
                    cv.putText(frame, text, (10, frame.shape[0] - 20 - i * 25), cv.FONT_HERSHEY_COMPLEX, 0.43, (255, 255, 255), 1)
                    cv.circle(frame, (int(robot_center[0]), int(robot_center[1])), 3, (255, 0, 0), -1)

            else:
                self.logger.warn('Unknown marker ID detected: {0}, ignoring'.format(markerIds[i,0]))
        if len(aruco_infos) > 0:        
            robot_pose = np.mean(aruco_infos, axis=0)

        return robot_pose

    
def main():
    # Grab images as numpy arrays and leave everything else to OpenCV.
    cv.startWindowThread()

    cam = CameraVisionStation(config=config)
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
    picam2.start()
    # setting focal pint at 10cm (TO BE FINE TUNED)
    picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": 1.3}) 

    while True:
        frame = picam2.capture_array()
        
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert frame to grayscale
        
        markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)  # Detect markers in grayscale frame

        if markerIds is not None:
            robot_poses = cam.get_robot_poses(frame, markerCorners, markerIds, set_visual_interface=True)
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
