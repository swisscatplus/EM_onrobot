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

        self.configure_logger()

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

        # theta=0.0
        # rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0], 
        #                             [np.sin(theta), np.cos(theta), 0], 
        #                             [0, 0, 1],
        #                             [0, 0, 0]])
        rotation_matrix = np.array([[1, 0, 0], 
                                    [0, 1, 0], 
                                    [0, 0, 1],
                                    [0, 0, 0]])
        
        homogeneous_matrix = np.concatenate((rotation_matrix, translation_vector.reshape(-1,1)), axis=1)
        
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

    def transform_to_circuit_frame(self, camera_center, aruco_center, aruco_tx, aruco_ty, aruco_angle, camera_angle=0.0):
        # Translate camera center to the origin of the ArUco marker in circuit frame
        dx = self.pixels_to_m*(camera_center[0] - aruco_center[0])* np.cos(aruco_angle)
        dy = self.pixels_to_m*(camera_center[1] - aruco_center[1]) * np.sin(aruco_angle)
        print('dx_pxl: ', camera_center[0] - aruco_center[0])
        print('dy_pxl: ', camera_center[1] - aruco_center[1])
        print('pxl_to_m: ', self.pixels_to_m)
        print('dx: ', dx)
        print('dy: ', dy)
        # Rotate around the ArUco marker and add the marker position in the circuit frame
        robot_tx = aruco_tx + dx
        robot_ty = aruco_ty + dy
        
        # Calculate the robot's orientation in the circuit frame
        robot_angle = camera_angle + aruco_angle
        
        return robot_tx, robot_ty, robot_angle

    def compute_camera_center(self, aruco_pxl_c, id, theta, pxl_max_x, pxl_max_y):
            t_x, t_y = self.aruco_ids[id]['t_x'], self.aruco_ids[id]['t_y']
            # Convert camera coordinates to Cartesian coordinates
            y_cart = pxl_max_y - aruco_pxl_c[1]

            # Compute the offset in the camera frame
            delta_x = aruco_pxl_c[0] - pxl_max_x / 2
            delta_y = y_cart - pxl_max_y / 2
            delta_x = delta_x * self.pixels_to_m  # Convert pixels to meters
            delta_y = delta_y * self.pixels_to_m  # Convert pixels to meters

            # Rotation matrix for -theta
            R = np.array([
                [np.cos(theta), -np.sin(theta), 0.0],
                [-np.sin(theta), -np.cos(theta), 0.0],
                [0.0, 0.0, 1.0]
            ])
            # Apply the rotation
            delta_XY = R @ np.array([delta_x, -delta_y, 0.0])
            
            # Compute the camera center in the circuit frame
            X_camera = t_x - delta_XY[0]
            Y_camera = t_y - delta_XY[1]

            return X_camera, Y_camera


    # Function to process the frame and detect ArUco markers
    def get_robot_pose(self, frame, markerCorners, markerIds, set_visual_interface=False):
        pxl_max_y, pxl_max_x, _ = frame.shape
        # pxl_center_cam = (pxl_max_x // 2, pxl_max_y // 2)
        self.pixels_to_m = self.pixels_to_meters(markerCorners)  # Constant conversion factor
        # robot_pose = None
        aruco_poses = []
        robot_angle = []
        robot_center = []

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

                # Calculate the angle of the marker
                dx = top_center[0] - bottom_center[0]
                dy = top_center[1] - bottom_center[1]
                angle = np.arctan2(dy, dx) * 180 / np.pi
                rad_angle = np.deg2rad(angle+90)

                coord_cam_circuit = self.compute_camera_center(aruco_pxl_c=center_code, id=markerIds[i, 0], theta=rad_angle, pxl_max_x=pxl_max_x, pxl_max_y=pxl_max_y)

                # Calculate the robot center position in the circuit frame, should be done by urdf once everything working
                robot_center = coord_cam_circuit[:2] - np.array([np.cos(-rad_angle), np.sin(-rad_angle)]) * self.cam_config['dist_cam_robot_center']
                aruco_poses.append(robot_center)
                robot_angle.append(-rad_angle)
                self.logger.debug('robot_center: {0}, robot_angle {1}'.format(robot_center, -rad_angle))
                
                if set_visual_interface:
                    # Draw arrow from bottom to top center
                    cv.arrowedLine(frame, bottom_center, top_center, (0, 0, 255), 4)
                    # Draw marker ID and center
                    cv.circle(frame, center_code, 3, (255, 0, 0), -1)
                    # Display orientation and position at the bottom of the screen
                    text = f"ID {markerIds[i][0]}: ang: {angle:.1f} deg, robot_pose: {robot_center}, cam_pose: {coord_cam_circuit[:2]}"
                    cv.putText(frame, text, (10, frame.shape[0] - 20 - i * 25), cv.FONT_HERSHEY_COMPLEX, 0.3, (255, 255, 255), 1)
                    cv.circle(frame, (int(robot_center[0]), int(robot_center[1])), 3, (255, 0, 0), -1)

            else:
                self.logger.warn('Unknown marker ID detected: {0}, ignoring'.format(markerIds[i,0]))
        if len(robot_center) > 0:  
            robot_center = np.mean(aruco_poses, axis=0)
            robot_angle = np.mean(robot_angle)
            
        return robot_center, robot_angle

def main():


    directory_path = os.path.dirname(os.path.abspath(__file__))
    config_file_path = os.path.join(directory_path, '../..', 'config', 'rpi_cam_on_robot.yaml')

    config = get_cam_config(config_file_path=config_file_path)

    cap = cv.VideoCapture(camera_port)
    cap.set(cv.CAP_PROP_AUTOFOCUS, 0) #remove autofocus
    cam = CameraRobot(config=config)

    while True:
        ret, frame = cap.read()
        
        if not ret:
            raise ("Can't receive frame (stream end?). Exiting ...")
        # print(frame.shape) #(480, 640, 3)
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
