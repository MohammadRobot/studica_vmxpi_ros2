#!/usr/bin/env python3
"""Bounded sensor-only acceptance probe; never publishes drive commands."""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def main():
    rclpy.init()
    node = Node("stationary_lidar_acceptance")
    samples = []
    service_times = {}

    def receive(message):
        samples.append((time.monotonic(), message))

    subscription = node.create_subscription(
        LaserScan, "/scan", receive, qos_profile_sensor_data
    )
    del subscription  # Node owns the subscription until destroy_node().

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            for topic in (
                "/cmd_vel", "/robot/platform/cmd_vel",
                "/robot_base_controller/cmd_vel",
            ):
                if node.count_publishers(topic):
                    raise RuntimeError(f"unexpected motion publisher: {topic}")

    def call(service):
        client = node.create_client(Empty, service)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(f"service unavailable: {service}")
            future = client.call_async(Empty.Request())
            started = time.monotonic()
            # X2 turnOn performs intensity detection (~7–8 s on this robot).
            # This bound is for scanner startup only, never a motion lease.
            deadline = started + (15 if service == "/start_scan" else 3)
            while not future.done() and time.monotonic() < deadline:
                spin_for(0.1)
            if not future.done() or future.result() is None:
                raise RuntimeError(f"service failed or timed out: {service}")
            service_times[service] = round(time.monotonic() - started, 3)
        finally:
            node.destroy_client(client)

    def summarize():
        if len(samples) < 10:
            raise RuntimeError(f"insufficient scans: {len(samples)}")
        rate = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
        if not 5 <= rate <= 20:
            raise RuntimeError(f"unexpected scan rate: {rate}")
        previous_stamp = -1
        for _, message in samples:
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            valid = [
                value for value in message.ranges
                if math.isfinite(value) and message.range_min <= value <= message.range_max
            ]
            if (
                message.header.frame_id != "laser_scan_frame"
                or stamp <= previous_stamp
                or not valid
                or message.angle_increment <= 0
                or message.range_max <= message.range_min
            ):
                raise RuntimeError("invalid scan metadata, timestamps or ranges")
            previous_stamp = stamp
        message = samples[-1][1]
        return {
            "scans": len(samples), "rate_hz": round(rate, 3),
            "frame_id": message.header.frame_id, "points": len(message.ranges),
            "valid_ranges_last_scan": len(valid),
        }

    try:
        spin_for(6)
        if node.count_publishers("/scan") != 1:
            raise RuntimeError("expected exactly one scan publisher")
        before = summarize()
        call("/stop_scan")
        spin_for(0.7)  # Drain any in-flight scan before assessing stopped output.
        samples.clear()
        spin_for(1.5)
        stopped_count = len(samples)
        call("/start_scan")
        samples.clear()
        spin_for(6)
        after = summarize()
        if stopped_count:
            raise RuntimeError(f"scanner still published while stopped: {stopped_count}")
        print(json.dumps({
            "passed": True, "before": before, "stopped_scans": stopped_count,
            "after": after, "motion_publishers": 0,
            "service_response_seconds": service_times,
            "nodes": sorted(node.get_node_names()),
        }, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
