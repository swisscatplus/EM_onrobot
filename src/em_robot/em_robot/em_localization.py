#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

import cv2 as cv
from picamera2 import Picamera2
from libcamera import controls

# Import the detection function from the aruco_detection script
from .aruco_detection import detect_markers


class MarkerLocalizationNode(Node):
    """
    ROS Node:
      - Continuously captures images from the PiCamera2
      - Detects ArUco markers in each frame
      - For each marker ID, publishes a TF transform that places
        the camera in the *marker’s* coordinate frame.

      i.e. parent: aruco_<markerID>
           child:  camera_frame
    """

    def __init__(self):
        super().__init__('marker_localization_node')
        self.get_logger().info("MarkerLocalizationNode started.")

        # 1) Initialize camera
        self.size = (4608, 2592)  # Or your real camera resolution
        self.picam2 = Picamera2()
        preview_config = self.picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": self.size}
        )
        self.picam2.configure(preview_config)
        self.picam2.start()

        # Example: manual focus
        self.camera_height = 0.63
        self.lens_position = 1.0 / self.camera_height
        self.picam2.set_controls({
            "AfMode": controls.AfModeEnum.Manual,
            "LensPosition": self.lens_position
        })

        # 2) Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 3) Create a timer to process frames
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.process_frame)

    def process_frame(self):
        # A) Capture an image
        frame = self.picam2.capture_array()

        # B) Detect markers in the image
        markers = detect_markers(frame)
        if not markers:
            self.get_logger().info("No markers detected this frame.")
            return

        # C) For each detected marker, publish "camera_frame" in marker's frame
        for marker_info in markers:
            marker_id = marker_info['id']
            mx = marker_info['x']
            my = marker_info['y']
            marker_yaw = marker_info['yaw']

            # In the camera's coordinate system, we found
            #   "Marker is at (mx, my) with heading marker_yaw"
            # We want: "Camera is at ??? in the marker's frame."
            #
            # The simplest approach: invert that 2D transform:
            #   camera_in_marker = - (marker_in_camera)
            # So translation = (-mx, -my) [plus zero in Z]
            # Orientation = -marker_yaw around z

            tx = -mx
            ty = -my
            tz = 0.0
            # Use roll=0, pitch=0, yaw=-marker_yaw
            roll = 0.0
            pitch = 0.0
            yaw = -marker_yaw
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

            # Build and publish the transform
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            # The marker frame is the parent, camera_frame is the child
            t.header.frame_id = f"aruco_{marker_id}"
            t.child_frame_id = "camera_frame"

            t.transform.translation.x = tx
            t.transform.translation.y = ty
            t.transform.translation.z = tz
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

            self.get_logger().info(
                f"Detected marker {marker_id} -> Published camera_frame in aruco_{marker_id} (tx={tx:.2f}, ty={ty:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down MarkerLocalizationNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
