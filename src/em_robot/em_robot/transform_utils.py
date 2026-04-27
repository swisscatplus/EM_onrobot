#!/usr/bin/env python3
import math

from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


def transform_to_matrix(transform):
    return concatenate_matrices(
        translation_matrix(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        ),
        quaternion_matrix(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        ),
    )


def build_transform(parent_frame, child_frame, transform_matrix, stamp):
    from geometry_msgs.msg import TransformStamped

    translation = translation_from_matrix(transform_matrix)
    quaternion = quaternion_from_matrix(transform_matrix)

    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    transform.transform.rotation.x = float(quaternion[0])
    transform.transform.rotation.y = float(quaternion[1])
    transform.transform.rotation.z = float(quaternion[2])
    transform.transform.rotation.w = float(quaternion[3])
    return transform


def build_planar_transform(x, y, yaw):
    return concatenate_matrices(
        translation_matrix([x, y, 0.0]),
        quaternion_matrix(quaternion_from_euler(0.0, 0.0, yaw)),
    )


def compute_map_to_base_from_marker(map_to_marker, camera_to_marker, camera_to_base):
    return map_to_marker @ inverse_matrix(camera_to_marker) @ camera_to_base


def compute_map_to_odom_from_map_to_base(map_to_base, odom_to_base):
    return map_to_base @ inverse_matrix(odom_to_base)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_matrix(transform_matrix):
    quaternion = quaternion_from_matrix(transform_matrix)
    _, _, yaw = euler_from_quaternion(quaternion)
    return yaw


def blend_angles(previous_yaw, new_yaw, alpha):
    return math.atan2(
        (1.0 - alpha) * math.sin(previous_yaw) + alpha * math.sin(new_yaw),
        (1.0 - alpha) * math.cos(previous_yaw) + alpha * math.cos(new_yaw),
    )
