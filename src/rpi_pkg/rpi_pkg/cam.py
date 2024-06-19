import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped # Message type for publishing robot pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv # OpenCV library
from tf_transformations import quaternion_from_euler
from rpi_pkg.submodules.detect_aruco import CameraVisionStation, detector
from launch_ros.substitutions import FindPackageShare
from picamera2 import Picamera2
from libcamera import controls
import yaml
import os
import pickle

# ################# CONFIG #################
# camera_id = 'station_1'
package_name = 'rpi_pkg'
params_path = 'config/cam.yaml'
timer_period = 0.1  # seconds

# ##########################################

pkg_share = FindPackageShare(package=package_name).find(package_name)
config_file_path = os.path.join(pkg_share, params_path)
print(pkg_share)
calib_mat_file_path = os.path.join(pkg_share, 'config', 'cameraMatrix.pkl')
calib_dist_file_path = os.path.join(pkg_share, 'config', 'dist.pkl')

class RobotCamPublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    super().__init__('robot_cam_publisher')
    
    self.declare_parameter('config_file_path', config_file_path)
    config_path = self.get_parameter('config_file_path').get_parameter_value().string_value
    self.config = self.get_cam_config(config_path)

    self.cam = CameraVisionStation(config=self.config)
    self.size = (640, 480) # size of the window
    self.picam2 = Picamera2()
    self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (self.size[0], self.size[1])}))
    self.picam2.start()
    self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": self.config['cam_params']['focal_length']}) 

    self.cam_publisher = self.create_publisher(PoseWithCovarianceStamped, 'edi/cam', 10)
    
    self.timer = self.create_timer(timer_period, self.publish_frame)
  
  def get_cam_config(self, config_file_path=None):
      with open(config_file_path, 'r') as file:
          data = yaml.safe_load(file)
      return data
  
  def publish_frame(self):
    frame = self.picam2.capture_array()
      
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert frame to grayscale
    cameraMatrix = pickle.load(open(calib_mat_file_path, 'rb'))
    dist = pickle.load(open(calib_dist_file_path, 'rb'))

    #Calibration process
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, self.size, 1, self.size)
    cal_frame = cv.undistort(gray_frame, cameraMatrix, dist, None, newCameraMatrix)
    # x, y, w, h = roi
    # cal_frame = cal_frame[y:y+h, x:x+w]
    markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)  # Detect markers in grayscale frame

    if markerIds is not None:
          robot_pose, robot_angle = self.cam.get_robot_pose(frame, markerCorners, markerIds)
          # print('rob pose, rob angle: ', robot_pose, robot_angle)
          if robot_pose is not None:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position.x = robot_pose[0]
                pose.pose.pose.position.y = robot_pose[1]
                pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w  = quaternion_from_euler(0, 0, robot_angle)
                pose.pose.covariance = [0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.001]
                self.get_logger().debug('ID:' +  str(pose.pose.pose.orientation))
                self.get_logger().debug('robot_pose:' +  str(pose))
                self.cam_publisher.publish(pose)
   
def main(args=None):
  rclpy.init()
  robot_cam_publisher = RobotCamPublisher()
  rclpy.spin(robot_cam_publisher)
  robot_cam_publisher.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
