#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Black-box mode transition and final autonomy-disarm contract test."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from studica_robot_platform import API_VERSION
from studica_vmxpi_ros2.msg import CompanionHeartbeat, PlatformStatus
from studica_vmxpi_ros2.srv import SetMode


class Probe(Node):

    def __init__(self):
        super().__init__("mode_manager_contract_probe")
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status = None
        self.output = Twist()
        self.disarm_count = 0
        self.safety_state = "READY_DISARMED"
        self.publish_scan = True
        self.companion = CompanionHeartbeat()
        self.companion.api_version = API_VERSION
        self.companion.companion_id = "test-pc"
        self.companion.connected = True
        self.companion.active_mode = 0
        self.companion.ready = True
        self.safety_publisher = self.create_publisher(
            String, "/robot/state", 10
        )
        self.scan_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.companion_publisher = self.create_publisher(
            CompanionHeartbeat, "/robot/companion/heartbeat", 10
        )
        self.web_publisher = self.create_publisher(
            Twist, "/robot/control/web", 1
        )
        self.create_subscription(
            PlatformStatus,
            "/robot/platform/status",
            self._status,
            status_qos,
        )
        self.create_subscription(
            Twist, "/robot/platform/cmd_vel", self._output, 10
        )
        self.create_service(Trigger, "/robot/disarm", self._disarm)
        self.mode_client = self.create_client(SetMode, "/robot/platform/set_mode")

    def _status(self, message):
        self.status = message

    def _output(self, message):
        self.output = message

    def _disarm(self, request, response):
        del request
        self.disarm_count += 1
        self.safety_state = "READY_DISARMED"
        response.success = True
        response.message = "test disarm acknowledged"
        return response

    def publish_inputs(self, command=None):
        state = String()
        state.data = self.safety_state
        self.safety_publisher.publish(state)
        if self.publish_scan:
            scan = LaserScan()
            scan.angle_min = -3.14
            scan.angle_max = 3.14
            scan.angle_increment = 0.01
            scan.range_min = 0.1
            scan.range_max = 12.0
            scan.ranges = [1.0] * 20
            self.scan_publisher.publish(scan)
        self.companion_publisher.publish(self.companion)
        if command is not None:
            self.web_publisher.publish(command)


def spin_until(node, predicate, timeout, command=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        node.publish_inputs(command)
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    status = node.status
    detail = "no status" if status is None else (
        f"{status.mode_name}/{status.transition}/{status.safety_state}"
    )
    raise AssertionError(f"timeout waiting for mode manager: {detail}")


def request_mode(node, mode, map_id=""):
    request = SetMode.Request()
    request.mode = mode
    request.map_id = map_id
    future = node.mode_client.call_async(request)
    spin_until(node, future.done, 3.0)
    assert future.result().accepted, future.result().message


def main():
    os.environ["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 20)
    prefix = Path(get_package_prefix("studica_vmxpi_ros2"))
    executable = (
        prefix / "lib/studica_vmxpi_ros2/studica_mode_manager.py"
    )
    snapshot = Path(f"/tmp/studica-mode-manager-test-{os.getpid()}.json")
    maintenance = snapshot.with_suffix(".maintenance")
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "orchestrator_required:=false",
            "-p",
            f"status_snapshot_path:={snapshot}",
            "-p",
            "companion_timeout_sec:=3.0",
            "-p",
            f"maintenance_lock_file:={maintenance}",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    node = None
    try:
        rclpy.init()
        node = Probe()
        spin_until(
            node,
            lambda: node.mode_client.service_is_ready()
            and node.status is not None
            and node.status.ready,
            6.0,
        )
        assert node.count_publishers("/robot/platform/cmd_vel") == 1

        request_mode(node, 2)
        spin_until(
            node,
            lambda: node.status.mode_name == "MANUAL_WEB"
            and node.status.transition == "READY",
            3.0,
        )
        assert node.disarm_count == 1

        node.safety_state = "ARMED"
        command = Twist()
        command.linear.x = 0.2
        spin_until(node, lambda: node.output.linear.x > 0.01, 3.0, command)

        request_mode(node, 3)
        spin_until(
            node,
            lambda: node.status.requested_mode_name == "SLAM"
            and node.status.transition == "WAITING_FOR_COMPANION",
            3.0,
        )
        assert node.disarm_count == 2
        assert abs(node.output.linear.x) < 1.0e-9

        # Simulate a Start press during companion startup. Dependency readiness
        # must trigger another acknowledged disarm before SLAM becomes ready.
        node.safety_state = "ARMED"
        spin_until(node, lambda: node.status.safety_state == "ARMED", 2.0)
        node.companion.active_mode = 3
        node.companion.ready = True
        spin_until(
            node,
            lambda: node.status.mode_name == "SLAM"
            and node.status.transition == "READY"
            and node.status.safety_state == "READY_DISARMED",
            3.0,
        )
        assert node.disarm_count == 3

        node.publish_scan = False
        spin_until(
            node,
            lambda: node.status.mode_name == "IDLE"
            and node.status.last_error == "LIDAR_LOST",
            3.0,
        )
        spin_until(node, lambda: node.disarm_count >= 4, 2.0)
        assert node.disarm_count >= 4
        assert abs(node.output.linear.x) < 1.0e-9
        # A new manual session must not resume when a lost source reconnects.
        request_mode(node, 2)
        spin_until(node, lambda: node.status.mode_name == "MANUAL_WEB"
                   and node.status.transition == "READY", 3.0)
        node.safety_state = "ARMED"
        spin_until(node, lambda: node.output.linear.x > 0.01, 3.0, command)
        count_before_loss = node.disarm_count
        spin_until(node, lambda: node.disarm_count > count_before_loss, 3.0)
        spin_until(node, lambda: node.status.safety_state == "READY_DISARMED", 2.0, command)
        assert abs(node.output.linear.x) < 1.0e-9

        node.publish_scan = True
        node.companion.active_mode = 0
        spin_until(
            node,
            lambda: node.status.lidar_healthy
            and node.status.companion_connected,
            3.0,
        )
        request_mode(node, 3)
        spin_until(
            node,
            lambda: node.status.transition == "WAITING_FOR_COMPANION",
            3.0,
        )
        node.companion.active_mode = 3
        spin_until(
            node,
            lambda: node.status.mode_name == "SLAM"
            and node.status.transition == "READY",
            3.0,
        )
        node.companion.connected = False
        spin_until(
            node,
            lambda: node.status.mode_name == "IDLE"
            and node.status.last_error == "COMPANION_LOST",
            3.0,
        )
        assert node.disarm_count >= 4
        assert abs(node.output.linear.x) < 1.0e-9
        maintenance.touch()
        request = SetMode.Request()
        request.mode = 2
        future = node.mode_client.call_async(request)
        spin_until(node, future.done, 3.0)
        assert not future.result().accepted
        assert "activation" in future.result().message
        node.safety_state = "ARMED"
        before_maintenance = node.disarm_count
        spin_until(node, lambda: node.disarm_count > before_maintenance, 3.0, command)
        assert abs(node.output.linear.x) < 1.0e-9
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        process.terminate()
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate(timeout=5.0)
        snapshot.unlink(missing_ok=True)
        maintenance.unlink(missing_ok=True)
        if process.returncode != 0:
            print(output, file=sys.stderr)
            raise RuntimeError(
                f"mode manager exited with {process.returncode}"
            )


if __name__ == "__main__":
    main()
