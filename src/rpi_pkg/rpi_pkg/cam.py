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

# ################# CONFIG #################
# camera_id = 'station_1'
package_name = 'rpi_pkg'
params_path = 'config/cam.yaml'
timer_period = 0.1  # seconds

# ##########################################

pkg_share = FindPackageShare(package=package_name).find(package_name)
config_file_path = os.path.join(pkg_share, params_path)

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

    #self.station_id = self.config['station_id']
    self.cam = CameraVisionStation(config=self.config)

    self.picam2 = Picamera2()
    self.picam2.configure(self.picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
    self.picam2.start()
    self.picam2.set_controls({"AfMode": controls.AfModeEnum.Manual, "LensPosition": self.config['cam_params']['focal_length']}) 

    self.cam_publisher = self.create_publisher(PoseWithCovarianceStamped, '/cam_robot_pose', 10)
    
    self.timer = self.create_timer(timer_period, self.publish_frame)
  
  def get_cam_config(self, config_file_path=None):
      with open(config_file_path, 'r') as file:
          data = yaml.safe_load(file)
      return data
  
  def publish_frame(self):
    frame = self.picam2.capture_array()
      
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convert frame to grayscale
        
    markerCorners, markerIds, _ = detector.detectMarkers(gray_frame)  # Detect markers in grayscale frame

    if markerIds is not None:
          robot_pose = self.cam.get_robot_poses(frame, markerCorners, markerIds)
          if robot_pose is not None:
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'odom'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position.x = robot_pose[0][0]
                pose.pose.pose.position.y = robot_pose[0][1]
                pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w  = quaternion_from_euler(0, 0, robot_pose[1])
                self.get_logger().debug('ID:' +  str(pose.pose.pose.orientation)) #
                self.get_logger().debug('robot_pose:' +  str(pose))
                self.cam_publisher.publish(pose)
   
def main(args=None):
  rclpy.init()
  
  robot_cam_publisher = RobotCamPublisher()
   
  rclpy.spin(robot_cam_publisher)
   
  robot_cam_publisher.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()
