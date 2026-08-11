#!/usr/bin/env python3
"""Unit tests for the dependency-free depth-to-point-cloud projection."""

from pathlib import Path
import sys

import numpy as np
from sensor_msgs.msg import Image


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from depth_to_pointcloud import depth_image_metres, project_depth, xyz_point_cloud  # noqa: E402


def test_project_depth_uses_camera_intrinsics_and_filters_invalid_values():
    depth = np.array(
        [[1.0, np.nan, 2.0], [0.0, 3.0, 11.0]],
        dtype=np.float32,
    )
    camera_matrix = [2.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0]

    points = project_depth(depth, camera_matrix, stride=1, max_depth_m=10.0)

    np.testing.assert_allclose(
        points,
        np.array([[-0.5, 0.0, 1.0], [1.0, 0.0, 2.0], [0.0, 1.5, 3.0]]),
    )


def test_projection_stride_samples_expected_pixels():
    depth = np.full((4, 4), 2.0, dtype=np.float32)
    camera_matrix = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 1.0]

    points = project_depth(depth, camera_matrix, stride=2)

    np.testing.assert_allclose(
        points,
        np.array([[0.0, 0.0, 2.0], [2.0, 0.0, 2.0], [0.0, 2.0, 2.0], [2.0, 2.0, 2.0]]),
    )


def test_16_bit_depth_with_row_padding_is_converted_from_mm_to_metres():
    message = Image()
    message.height = 2
    message.width = 2
    message.encoding = "16UC1"
    message.is_bigendian = False
    message.step = 6
    message.data = (
        np.array([1000, 2000, 99, 3000, 4000, 99], dtype="<u2").tobytes()
    )

    depth = depth_image_metres(message)

    np.testing.assert_allclose(depth, [[1.0, 2.0], [3.0, 4.0]])


def test_point_cloud_message_has_standard_xyz_layout():
    image = Image()
    image.header.frame_id = "camera_optical_frame"
    points = np.array([[1.0, 2.0, 3.0], [-1.0, 0.5, 4.0]], dtype=np.float32)

    cloud = xyz_point_cloud(image.header, points)

    assert cloud.header.frame_id == "camera_optical_frame"
    assert cloud.height == 1
    assert cloud.width == 2
    assert cloud.point_step == 12
    assert cloud.row_step == 24
    assert [field.name for field in cloud.fields] == ["x", "y", "z"]
    np.testing.assert_allclose(np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3), points)
