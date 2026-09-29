#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Prove three concurrent local DDS sessions cannot exchange telemetry or clocks."""

import json
import os
from pathlib import Path
import subprocess
import sys

from test_session_cli import CLI


PROBE = '''
import json, os, time
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
rclpy.init()
node = rclpy.create_node('session_isolation_probe')
identity = os.environ['ROS_DOMAIN_ID']
seen, clocks = set(), []
node.create_subscription(String, '/studica_session_probe', lambda m: seen.add(m.data), 10)
node.create_subscription(Clock, '/clock', lambda m: clocks.append(True), 10)
publisher = node.create_publisher(String, '/studica_session_probe', 10)
clock = node.create_publisher(Clock, '/clock', 10) if os.environ['STUDICA_USE_SIM_TIME'] == 'true' else None
deadline = time.monotonic() + 4
while time.monotonic() < deadline:
    publisher.publish(String(data=identity))
    if clock:
        clock.publish(Clock())
    rclpy.spin_once(node, timeout_sec=0.03)
print(json.dumps({'seen': sorted(seen), 'clock': bool(clocks)}))
node.destroy_node()
rclpy.shutdown()
'''


def main():
    config = Path(__file__).resolve().parents[1] / "bringup/config/network/cyclonedds_sim.xml"
    first = 30 + os.getpid() % 30
    sessions = [CLI.environment({"kind": kind, "domain_id": first + offset,
                                 "dds_config": str(config)})
                for offset, kind in enumerate(("robot", "robot", "sim"))]
    processes = []
    try:
        for env in sessions:
            processes.append(subprocess.Popen([sys.executable, "-c", PROBE], env=env,
                                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                              text=True))
        for index, process in enumerate(processes):
            output, error = process.communicate(timeout=15)
            assert process.returncode == 0, error
            result = json.loads(output)
            assert result["seen"] == [str(first + index)], result
            assert result["clock"] == (index == 2), result
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
                process.wait()
    print("PASS: two robot sessions and one simulation isolate telemetry and /clock")


if __name__ == "__main__":
    main()
