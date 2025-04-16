import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_multiply, quaternion_from_euler, quaternion_matrix
from copy import deepcopy
import numpy as np

class ImuTransformer(Node):
    def __init__(self):
        super().__init__('imu_transformer')

        self.subscription = self.create_subscription(
            Imu,
            '/bno055/imu',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(
            Imu,
            '/bno055/imu_rotated',
            10)

        # Quaternion représentant la rotation Y 90° puis Z 90°
        q_y_90 = quaternion_from_euler(0, 1.5708, 0)
        q_z_90 = quaternion_from_euler(0, 0, 1.5708)
        self.rotation_quaternion = quaternion_multiply(q_z_90, q_y_90)

        # Matrice de rotation 4x4 -> on extrait la 3x3
        self.rotation_matrix = quaternion_matrix(self.rotation_quaternion)[:3, :3]

    def rotate_vector(self, vec):
        return np.dot(self.rotation_matrix, vec)

    def listener_callback(self, msg: Imu):
        transformed_msg = deepcopy(msg)

        # === Orientation ===
        original_q = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        rotated_q = quaternion_multiply(self.rotation_quaternion, original_q)
        transformed_msg.orientation.x = rotated_q[0]
        transformed_msg.orientation.y = rotated_q[1]
        transformed_msg.orientation.z = rotated_q[2]
        transformed_msg.orientation.w = rotated_q[3]

        # === Angular velocity ===
        orig_ang_vel = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        rotated_ang_vel = self.rotate_vector(orig_ang_vel)
        transformed_msg.angular_velocity.x = rotated_ang_vel[0]
        transformed_msg.angular_velocity.y = rotated_ang_vel[1]
        transformed_msg.angular_velocity.z = rotated_ang_vel[2]

        # === Linear acceleration ===
        orig_lin_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        rotated_lin_acc = self.rotate_vector(orig_lin_acc)
        transformed_msg.linear_acceleration.x = rotated_lin_acc[0]
        transformed_msg.linear_acceleration.y = rotated_lin_acc[1]
        transformed_msg.linear_acceleration.z = rotated_lin_acc[2]

        self.publisher_.publish(transformed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
