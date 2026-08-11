#!/usr/bin/env python3
"""Unit tests for ground-frame point-cloud filtering."""

from pathlib import Path
import sys

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from point_cloud_filter import (  # noqa: E402
    FilterBounds,
    downsample_points,
    extract_xyz,
    filter_obstacle_points,
    transform_points,
    xyz_point_cloud,
)


def test_downsample_points_uses_a_valid_positive_stride():
    points = np.arange(18, dtype=np.float32).reshape(6, 3)

    np.testing.assert_allclose(downsample_points(points, 2), points[[0, 2, 4]])
    np.testing.assert_allclose(downsample_points(points, 1), points)

    for invalid_stride in (0, -1, 1.5, True):
        try:
            downsample_points(points, invalid_stride)
        except ValueError:
            pass
        else:
            raise AssertionError(f"stride {invalid_stride!r} should be rejected")


def test_observed_floor_transforms_to_ground_and_is_removed():
    camera_points = np.array(
        [
            [0.0, 0.182, 0.293],
            [0.0, 0.000, 0.196],
        ],
        dtype=np.float32,
    )

    base_points = transform_points(
        camera_points,
        translation=(0.192, 0.0, 0.182),
        quaternion_xyzw=(-0.5, 0.5, -0.5, 0.5),
    )
    filtered = filter_obstacle_points(base_points, FilterBounds())

    np.testing.assert_allclose(base_points[0], [0.485, 0.0, 0.0], atol=1e-6)
    np.testing.assert_allclose(base_points[1], [0.388, 0.0, 0.182], atol=1e-6)
    np.testing.assert_allclose(filtered, [[0.388, 0.0, 0.182]], atol=1e-6)


def test_crop_rejects_self_far_lateral_tall_and_nonfinite_points():
    points = np.array(
        [
            [1.0, 0.0, 0.2],
            [0.2, 0.0, 0.2],
            [3.1, 0.0, 0.2],
            [1.0, 2.1, 0.2],
            [1.0, 0.0, 1.6],
            [1.0, 0.0, np.nan],
        ],
        dtype=np.float32,
    )

    filtered = filter_obstacle_points(points, FilterBounds())

    np.testing.assert_allclose(filtered, [[1.0, 0.0, 0.2]])


def test_extract_xyz_supports_row_padding():
    cloud = PointCloud2()
    cloud.height = 2
    cloud.width = 1
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.point_step = 12
    cloud.row_step = 16
    cloud.data = (
        np.array([1.0, 2.0, 3.0], dtype="<f4").tobytes()
        + b"pad!"
        + np.array([4.0, 5.0, 6.0], dtype="<f4").tobytes()
        + b"pad!"
    )

    np.testing.assert_allclose(extract_xyz(cloud), [[1, 2, 3], [4, 5, 6]])


def test_filtered_cloud_has_target_frame_and_standard_layout():
    points = np.array([[1.0, 0.0, 0.2]], dtype=np.float32)
    source = PointCloud2()
    source.header.stamp.sec = 42

    cloud = xyz_point_cloud("base_link", source.header.stamp, points)

    assert cloud.header.frame_id == "base_link"
    assert cloud.header.stamp.sec == 42
    assert cloud.width == 1
    assert cloud.point_step == 12
    assert [field.name for field in cloud.fields] == ["x", "y", "z"]
