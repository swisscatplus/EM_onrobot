# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math

import numpy as np

from tf_transformations import (
    concatenate_matrices,
    euler_from_quaternion,
    inverse_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_multiply,
    translation_from_matrix,
    translation_matrix,
)


def test_translation_and_inverse_matrix_round_trip():
    transform = concatenate_matrices(
        translation_matrix([1.0, -2.0, 0.5]),
        quaternion_matrix(quaternion_from_euler(0.1, -0.2, 0.3)),
    )

    restored = concatenate_matrices(transform, inverse_matrix(transform))

    assert np.allclose(restored, np.identity(4), atol=1e-9)
    assert np.allclose(translation_from_matrix(transform), [1.0, -2.0, 0.5])


def test_quaternion_helpers_round_trip_rotation():
    quaternion = quaternion_from_euler(0.2, -0.1, 0.4)
    recovered = quaternion_from_matrix(quaternion_matrix(quaternion))
    roll, pitch, yaw = euler_from_quaternion(recovered)

    assert np.allclose(np.abs(np.dot(quaternion, recovered)), 1.0, atol=1e-6)
    assert math.isclose(roll, 0.2, abs_tol=1e-6)
    assert math.isclose(pitch, -0.1, abs_tol=1e-6)
    assert math.isclose(yaw, 0.4, abs_tol=1e-6)


def test_quaternion_multiply_composes_rotations():
    q_yaw = quaternion_from_euler(0.0, 0.0, 0.4)
    q_pitch = quaternion_from_euler(0.0, 0.2, 0.0)
    combined = quaternion_multiply(q_yaw, q_pitch)

    combined_matrix = quaternion_matrix(combined)
    expected_matrix = np.dot(quaternion_matrix(q_yaw), quaternion_matrix(q_pitch))

    assert np.allclose(combined_matrix, expected_matrix, atol=1e-6)
