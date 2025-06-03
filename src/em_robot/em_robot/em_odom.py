import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from builtin_interfaces.msg import Time as BuiltinTime

class TfToPoseXYYawPublisher(Node):
    def __init__(self):
        super().__init__('tf_to_pose_xy_yaw_publisher')

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/camera_pose',
            10
        )
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_tf_stamp = None

        # Timer at 2Hz
        self.timer = self.create_timer(1.0 / 2.0, self.publish_pose)

    def publish_pose(self):
        try:
            now = self.get_clock().now()
            trans = self.tf_buffer.lookup_transform(
                'map',
                'cam_base_link',
                rclpy.time.Time(),  # latest available
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Check how old the transform is
            tf_time = rclpy.time.Time.from_msg(trans.header.stamp)
            age = now - tf_time
            age_sec = age.nanoseconds / 1e9

            if age_sec > 0.5:
                self.get_logger().warn(
                    f"Stale transform ({age_sec:.2f}s old), skipping publish."
                )
                return

            # Extract yaw only from quaternion
            quat = trans.transform.rotation
            (_, _, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            quat_yaw = quaternion_from_euler(0, 0, yaw)

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = trans.header.stamp
            pose_msg.header.frame_id = 'map'


            pose_msg.pose.pose.position.x = trans.transform.translation.x
            pose_msg.pose.pose.position.y = trans.transform.translation.y
            pose_msg.pose.pose.position.z = 0.0

            pose_msg.pose.pose.orientation.x = quat_yaw[0]
            pose_msg.pose.pose.orientation.y = quat_yaw[1]
            pose_msg.pose.pose.orientation.z = quat_yaw[2]
            pose_msg.pose.pose.orientation.w = quat_yaw[3]

            pose_msg.pose.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 9999.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 9999.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 9999.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01
            ]

            self.publisher_.publish(pose_msg)

        except Exception as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = TfToPoseXYYawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
