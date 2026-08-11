#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Convert a depth image and camera calibration into an XYZ PointCloud2."""

from typing import Sequence

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


def depth_image_metres(message: Image) -> np.ndarray:
    """Return a zero-copy depth view converted to metres when necessary."""
    encoding = message.encoding.lower()
    if encoding == "32fc1":
        format_code = "f4"
        scale = 1.0
    elif encoding in {"16uc1", "mono16"}:
        format_code = "u2"
        scale = 0.001
    else:
        raise ValueError(
            f"unsupported depth encoding {message.encoding!r}; "
            "expected 32FC1, 16UC1, or mono16"
        )

    byte_order = ">" if message.is_bigendian else "<"
    data_type = np.dtype(byte_order + format_code)
    minimum_step = int(message.width) * data_type.itemsize
    if int(message.step) < minimum_step:
        raise ValueError("depth image step is smaller than one image row")
    if len(message.data) < int(message.step) * int(message.height):
        raise ValueError("depth image data is shorter than height * step")

    depth = np.ndarray(
        shape=(int(message.height), int(message.width)),
        dtype=data_type,
        buffer=message.data,
        strides=(int(message.step), data_type.itemsize),
    )
    if scale == 1.0:
        return depth
    return depth.astype(np.float32) * scale


def project_depth(
    depth_metres: np.ndarray,
    camera_matrix: Sequence[float],
    stride: int = 4,
    min_depth_m: float = 0.1,
    max_depth_m: float = 10.0,
) -> np.ndarray:
    """Project sampled valid pixels into optical-frame XYZ coordinates."""
    if depth_metres.ndim != 2:
        raise ValueError("depth image must be a two-dimensional array")
    if len(camera_matrix) != 9:
        raise ValueError("camera matrix must contain nine values")
    if stride < 1:
        raise ValueError("stride must be at least one")
    if min_depth_m < 0.0 or max_depth_m <= min_depth_m:
        raise ValueError("depth limits must satisfy 0 <= min_depth_m < max_depth_m")

    focal_x = float(camera_matrix[0])
    focal_y = float(camera_matrix[4])
    centre_x = float(camera_matrix[2])
    centre_y = float(camera_matrix[5])
    if focal_x <= 0.0 or focal_y <= 0.0:
        raise ValueError("camera focal lengths must be greater than zero")

    sampled = np.asarray(depth_metres[::stride, ::stride], dtype=np.float32)
    pixel_y, pixel_x = np.indices(sampled.shape, dtype=np.float32)
    pixel_x *= float(stride)
    pixel_y *= float(stride)

    valid = (
        np.isfinite(sampled)
        & (sampled >= float(min_depth_m))
        & (sampled <= float(max_depth_m))
    )
    depth = sampled[valid]
    if depth.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    x = (pixel_x[valid] - centre_x) * depth / focal_x
    y = (pixel_y[valid] - centre_y) * depth / focal_y
    return np.column_stack((x, y, depth)).astype(np.float32, copy=False)


def xyz_point_cloud(header, points: np.ndarray) -> PointCloud2:
    """Create a compact, unorganized PointCloud2 containing float32 XYZ."""
    xyz = np.asarray(points, dtype=np.float32)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")

    cloud = PointCloud2()
    cloud.header = header
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


class DepthToPointCloud(Node):
    """Publish a sampled XYZ cloud from the standard simulated depth topics."""

    def __init__(self) -> None:
        super().__init__("depth_to_pointcloud")
        self.declare_parameter("stride", 4)
        self.declare_parameter("min_depth_m", 0.1)
        self.declare_parameter("max_depth_m", 10.0)

        self._stride = int(self.get_parameter("stride").value)
        self._min_depth_m = float(self.get_parameter("min_depth_m").value)
        self._max_depth_m = float(self.get_parameter("max_depth_m").value)
        if self._stride < 1:
            raise ValueError("stride must be at least one")
        if self._min_depth_m < 0.0 or self._max_depth_m <= self._min_depth_m:
            raise ValueError("invalid min_depth_m and max_depth_m parameters")

        self._camera_matrix = None
        self._waiting_logged = False
        self._publisher = self.create_publisher(
            PointCloud2,
            "camera/depth/points",
            qos_profile_sensor_data,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            "camera/depth/camera_info",
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._depth_subscription = self.create_subscription(
            Image,
            "camera/depth/image_raw",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Depth-to-point-cloud converter started: "
            f"stride={self._stride}, range={self._min_depth_m:.2f}-"
            f"{self._max_depth_m:.2f} m"
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        if len(message.k) == 9 and message.k[0] > 0.0 and message.k[4] > 0.0:
            self._camera_matrix = tuple(message.k)

    def _on_depth(self, message: Image) -> None:
        if self._camera_matrix is None:
            if not self._waiting_logged:
                self.get_logger().info("Waiting for /camera/depth/camera_info")
                self._waiting_logged = True
            return

        try:
            depth = depth_image_metres(message)
            points = project_depth(
                depth,
                self._camera_matrix,
                stride=self._stride,
                min_depth_m=self._min_depth_m,
                max_depth_m=self._max_depth_m,
            )
            self._publisher.publish(xyz_point_cloud(message.header, points))
        except ValueError as error:
            self.get_logger().error(f"Cannot convert depth image: {error}")


def main(args=None) -> None:
    """Run the depth-to-point-cloud node."""
    rclpy.init(args=args)
    node = DepthToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
