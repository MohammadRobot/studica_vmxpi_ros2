#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Transform an XYZ PointCloud2 into base_link and remove floor/self returns."""

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class FilterBounds:
    """Ground-referenced crop and robot-body exclusion limits in metres."""

    min_height_m: float = 0.04
    max_height_m: float = 1.50
    min_forward_m: float = 0.15
    max_forward_m: float = 3.00
    max_lateral_m: float = 2.00
    robot_min_x_m: float = -0.20
    robot_max_x_m: float = 0.25
    robot_half_width_m: float = 0.21
    robot_max_height_m: float = 0.35

    def validate(self) -> None:
        """Raise ValueError when the crop cannot describe a valid volume."""
        values = tuple(self.__dict__.values())
        if not all(np.isfinite(values)):
            raise ValueError("all point-cloud filter bounds must be finite")
        if self.min_height_m < 0.0 or self.max_height_m <= self.min_height_m:
            raise ValueError("height bounds must satisfy 0 <= min < max")
        if self.max_forward_m <= self.min_forward_m:
            raise ValueError("forward bounds must satisfy min < max")
        if self.max_lateral_m <= 0.0:
            raise ValueError("max_lateral_m must be greater than zero")
        if self.robot_max_x_m <= self.robot_min_x_m:
            raise ValueError("robot x bounds must satisfy min < max")
        if self.robot_half_width_m <= 0.0 or self.robot_max_height_m <= 0.0:
            raise ValueError("robot exclusion width and height must be positive")


def extract_xyz(message: PointCloud2) -> np.ndarray:
    """Extract float32 XYZ values while respecting field and row padding."""
    fields = {field.name: field for field in message.fields}
    missing = [name for name in ("x", "y", "z") if name not in fields]
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {', '.join(missing)}")
    if message.point_step <= 0:
        raise ValueError("PointCloud2 point_step must be greater than zero")

    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype != PointField.FLOAT32 or field.count < 1:
            raise ValueError(f"PointCloud2 field {name!r} must be FLOAT32")
        if field.offset + 4 > message.point_step:
            raise ValueError(f"PointCloud2 field {name!r} exceeds point_step")

    minimum_row_step = int(message.width) * int(message.point_step)
    if int(message.row_step) < minimum_row_step:
        raise ValueError("PointCloud2 row_step is smaller than width * point_step")
    required_bytes = int(message.height) * int(message.row_step)
    if len(message.data) < required_bytes:
        raise ValueError("PointCloud2 data is shorter than height * row_step")

    byte_order = ">" if message.is_bigendian else "<"
    data_type = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [byte_order + "f4"] * 3,
            "offsets": [fields[name].offset for name in ("x", "y", "z")],
            "itemsize": int(message.point_step),
        }
    )
    rows = []
    for row_index in range(int(message.height)):
        row = np.frombuffer(
            message.data,
            dtype=data_type,
            count=int(message.width),
            offset=row_index * int(message.row_step),
        )
        rows.append(np.column_stack((row["x"], row["y"], row["z"])))
    if not rows:
        return np.empty((0, 3), dtype=np.float32)
    return np.concatenate(rows).astype(np.float32, copy=False)


def transform_points(
    points: np.ndarray,
    translation: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Apply a rigid transform to an N-by-3 point array."""
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if len(translation) != 3 or len(quaternion_xyzw) != 4:
        raise ValueError("transform requires xyz translation and xyzw quaternion")

    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if not np.isfinite(norm) or norm <= 1e-12:
        raise ValueError("transform quaternion must be finite and nonzero")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    rotation = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    offset = np.asarray(translation, dtype=np.float32)
    if not np.all(np.isfinite(offset)):
        raise ValueError("transform translation must be finite")
    return xyz @ rotation.T + offset


def downsample_points(points: np.ndarray, stride: int) -> np.ndarray:
    """Keep every ``stride``-th point without copying the input array."""
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if isinstance(stride, bool) or not isinstance(stride, (int, np.integer)):
        raise ValueError("stride must be an integer")
    if int(stride) < 1:
        raise ValueError("stride must be at least one")
    return xyz[:: int(stride)]


def filter_obstacle_points(points: np.ndarray, bounds: FilterBounds) -> np.ndarray:
    """Keep finite points in the useful volume and outside the robot body."""
    bounds.validate()
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")

    finite = np.all(np.isfinite(xyz), axis=1)
    useful_volume = (
        finite
        & (xyz[:, 0] >= bounds.min_forward_m)
        & (xyz[:, 0] <= bounds.max_forward_m)
        & (np.abs(xyz[:, 1]) <= bounds.max_lateral_m)
        & (xyz[:, 2] >= bounds.min_height_m)
        & (xyz[:, 2] <= bounds.max_height_m)
    )
    inside_robot = (
        (xyz[:, 0] >= bounds.robot_min_x_m)
        & (xyz[:, 0] <= bounds.robot_max_x_m)
        & (np.abs(xyz[:, 1]) <= bounds.robot_half_width_m)
        & (xyz[:, 2] >= 0.0)
        & (xyz[:, 2] <= bounds.robot_max_height_m)
    )
    return xyz[useful_volume & ~inside_robot]


def xyz_point_cloud(frame_id: str, stamp, points: np.ndarray) -> PointCloud2:
    """Create a compact unorganized PointCloud2 with float32 XYZ fields."""
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.header.stamp = stamp
    cloud.height = 1
    cloud.width = int(xyz.shape[0])
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = xyz.astype("<f4", copy=False).tobytes()
    cloud.is_dense = True
    return cloud


class PointCloudFilter(Node):
    """Transform raw points into a ground frame and publish obstacle returns."""

    def __init__(self) -> None:
        super().__init__("point_cloud_filter")
        self.declare_parameter("input_topic", "/camera/depth/points")
        self.declare_parameter("output_topic", "/camera/depth/points_filtered")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("stride", 1)
        defaults = FilterBounds()
        for name, value in defaults.__dict__.items():
            self.declare_parameter(name, value)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._target_frame = str(self.get_parameter("target_frame").value)
        self._stride = int(self.get_parameter("stride").value)
        downsample_points(np.empty((0, 3), dtype=np.float32), self._stride)
        self._bounds = FilterBounds(
            **{
                name: float(self.get_parameter(name).value)
                for name in defaults.__dict__
            }
        )
        self._bounds.validate()
        if not self._input_topic or not self._output_topic or not self._target_frame:
            raise ValueError("point-cloud topics and target_frame must not be empty")

        self._transform_warning_logged = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            PointCloud2,
            self._output_topic,
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            self._input_topic,
            self._on_point_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Point-cloud filter started: {self._input_topic} -> {self._output_topic}, "
            f"frame={self._target_frame}, height={self._bounds.min_height_m:.2f}-"
            f"{self._bounds.max_height_m:.2f} m, stride={self._stride}"
        )

    def _on_point_cloud(self, message: PointCloud2) -> None:
        if not message.header.frame_id:
            self.get_logger().error("Cannot filter a point cloud with an empty frame_id")
            return
        try:
            points = downsample_points(extract_xyz(message), self._stride)
            if message.header.frame_id != self._target_frame:
                transform = self._tf_buffer.lookup_transform(
                    self._target_frame,
                    message.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.05),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                points = transform_points(
                    points,
                    (translation.x, translation.y, translation.z),
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                )
            filtered = filter_obstacle_points(points, self._bounds)
            self._publisher.publish(
                xyz_point_cloud(self._target_frame, message.header.stamp, filtered)
            )
            self._transform_warning_logged = False
        except TransformException as error:
            if not self._transform_warning_logged:
                self.get_logger().warning(
                    f"Waiting for transform {self._target_frame} <- "
                    f"{message.header.frame_id}: {error}"
                )
                self._transform_warning_logged = True
        except ValueError as error:
            self.get_logger().error(f"Cannot filter point cloud: {error}")


def main(args=None) -> None:
    """Run the base-frame point-cloud filter."""
    rclpy.init(args=args)
    node = PointCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
