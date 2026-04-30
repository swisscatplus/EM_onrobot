# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

from __future__ import annotations

import math

import numpy as np


def concatenate_matrices(*matrices):
    result = np.identity(4, dtype=np.float64)
    for matrix in matrices:
        result = np.dot(result, np.array(matrix, dtype=np.float64))
    return result


def translation_matrix(direction):
    matrix = np.identity(4, dtype=np.float64)
    matrix[:3, 3] = np.array(direction[:3], dtype=np.float64)
    return matrix


def translation_from_matrix(matrix):
    return np.array(matrix, dtype=np.float64)[:3, 3].copy()


def inverse_matrix(matrix):
    return np.linalg.inv(np.array(matrix, dtype=np.float64))


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = float(roll) * 0.5
    half_pitch = float(pitch) * 0.5
    half_yaw = float(yaw) * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


def quaternion_multiply(quaternion1, quaternion0):
    x1, y1, z1, w1 = np.array(quaternion1, dtype=np.float64)
    x0, y0, z0, w0 = np.array(quaternion0, dtype=np.float64)

    return np.array(
        [
            w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
            w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
            w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0,
            w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
        ],
        dtype=np.float64,
    )


def quaternion_matrix(quaternion):
    q = np.array(quaternion, dtype=np.float64, copy=True)
    norm = np.dot(q, q)
    if norm < np.finfo(float).eps:
        return np.identity(4, dtype=np.float64)

    q /= math.sqrt(norm)
    x, y, z, w = q

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w), 0.0],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w), 0.0],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y), 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def quaternion_from_matrix(matrix):
    m = np.array(matrix, dtype=np.float64, copy=False)
    if m.shape == (4, 4):
        m = m[:3, :3]
    elif m.shape != (3, 3):
        raise ValueError(f"Expected a 3x3 or 4x4 matrix, got shape {m.shape}")

    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < np.finfo(float).eps:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def euler_from_quaternion(quaternion):
    x, y, z, w = np.array(quaternion, dtype=np.float64)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
