# Copyright 2026 SwissCAT+
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import math

import numpy as np

from em_robot.transform_utils import (
    build_planar_transform,
    compute_map_to_base_from_marker,
    compute_map_to_odom_from_map_to_base,
    yaw_from_matrix,
)


def test_compute_map_to_base_from_marker_preserves_camera_to_base_offset():
    map_to_marker = build_planar_transform(2.0, 3.0, math.pi / 2.0)
    camera_to_marker = build_planar_transform(0.8, 0.0, 0.0)
    camera_to_base = build_planar_transform(-0.2, 0.0, 0.0)

    map_to_base = compute_map_to_base_from_marker(
        map_to_marker,
        camera_to_marker,
        camera_to_base,
    )

    assert math.isclose(map_to_base[0, 3], 2.0, abs_tol=1e-9)
    assert math.isclose(map_to_base[1, 3], 2.0, abs_tol=1e-9)
    assert math.isclose(yaw_from_matrix(map_to_base), math.pi / 2.0, abs_tol=1e-9)


def test_compute_map_to_odom_reconstructs_the_same_map_to_base_pose():
    map_to_base = build_planar_transform(1.4, -0.6, 0.35)
    odom_to_base = build_planar_transform(0.3, 0.2, -0.15)

    map_to_odom = compute_map_to_odom_from_map_to_base(map_to_base, odom_to_base)

    reconstructed_map_to_base = map_to_odom @ odom_to_base

    assert np.allclose(reconstructed_map_to_base, map_to_base, atol=1e-9)
