#!/usr/bin/env python3
"""Bounded depth/CameraInfo/LiDAR observation; no command publishing."""

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan


def main():
    rclpy.init()
    node = Node("stationary_camera_acceptance")
    images, scans, infos = [], [], []
    node.create_subscription(Image, "/camera/depth/image_raw",
                             lambda msg: images.append((time.monotonic(), msg)), qos_profile_sensor_data)
    node.create_subscription(LaserScan, "/scan",
                             lambda msg: scans.append(time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/camera/depth/camera_info",
                             lambda msg: infos.append(msg), qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for topic in ("/cmd_vel", "/robot/platform/cmd_vel", "/robot_base_controller/cmd_vel"):
                if node.count_publishers(topic):
                    raise RuntimeError(f"unexpected motion publisher: {topic}")
        if len(images) < 20 or len(scans) < 40 or not infos:
            raise RuntimeError(f"insufficient data: {len(images)} images, {len(scans)} scans, {len(infos)} CameraInfo")
        previous = -1
        for _, msg in images:
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if (msg.width, msg.height, msg.encoding, msg.step, len(msg.data)) != (320, 240, "16UC1", 640, 153600):
                raise RuntimeError("unexpected image format")
            if stamp <= previous:
                raise RuntimeError("non-increasing camera timestamps")
            previous = stamp
        msg = images[-1][1]
        pixels = np.frombuffer(msg.data, dtype=">u2" if msg.is_bigendian else "<u2")
        valid = pixels[(pixels > 0) & (pixels < 65535)]
        if not len(valid):
            raise RuntimeError("no valid depth pixels")
        info = infos[-1]
        if ((info.width, info.height, info.header.frame_id) != (320, 240, msg.header.frame_id)
                or info.k[0] <= 0 or info.k[4] <= 0):
            raise RuntimeError("invalid or mismatched camera calibration")
        image_rate = (len(images) - 1) / (images[-1][0] - images[0][0])
        scan_rate = (len(scans) - 1) / (scans[-1] - scans[0])
        if not 4 <= image_rate <= 6 or scan_rate < 5:
            raise RuntimeError("unexpected camera or LiDAR rate")
        print(json.dumps({
            "passed": True, "images": len(images), "image_rate_hz": round(image_rate, 3),
            "scan_rate_hz": round(scan_rate, 3), "camera_info_messages": len(infos),
            "width": msg.width, "height": msg.height, "encoding": msg.encoding,
            "frame_id": msg.header.frame_id, "valid_depth_pixels": len(valid),
            "depth_raw_min": int(valid.min()), "depth_raw_max": int(valid.max()),
            "motion_publishers_in_test_domain": 0,
        }, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
