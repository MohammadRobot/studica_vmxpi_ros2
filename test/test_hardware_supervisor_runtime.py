#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Black-box hardware-state mirror test with no VMX or motor process."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


SAFETY_NAMES = [
    "input_valid",
    "estop_ok",
    "enable_active",
    "drive_healthy",
    "motion_enabled",
    "gate_state",
    "fault_reason",
]


def safety_message(values=None):
    resolved = [1.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    if values is not None:
        resolved = values
    message = DynamicJointState()
    message.joint_names = ["hardware_safety"]
    interfaces = InterfaceValue()
    interfaces.interface_names = SAFETY_NAMES[: len(resolved)]
    interfaces.values = resolved
    message.interface_values = [interfaces]
    return message


class Probe(Node):

    def __init__(self):
        super().__init__("hardware_supervisor_contract_probe")
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state = ""
        self.reason = ""
        self.output = TwistStamped()
        self.safety_publisher = self.create_publisher(
            DynamicJointState, "/dynamic_joint_states", 10
        )
        self.command_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/robot/state", self._state, status_qos)
        self.create_subscription(
            String, "/robot/safety_reason", self._reason, status_qos
        )
        self.create_subscription(
            TwistStamped,
            "/robot_base_controller/cmd_vel",
            self._output,
            10,
        )
        self.disarm_client = self.create_client(Trigger, "/robot/disarm")

    def _state(self, message):
        self.state = message.data

    def _reason(self, message):
        self.reason = message.data

    def _output(self, message):
        self.output = message


def spin_until(node, predicate, timeout, publish=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if publish is not None:
            publish()
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError(
        f"timeout: state={node.state!r} reason={node.reason!r} "
        f"output={node.output.twist.linear.x:.3f}"
    )


def spin_for(node, duration, publish=None):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        if publish is not None:
            publish()
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    os.environ["ROS_DOMAIN_ID"] = str(140 + os.getpid() % 40)
    prefix = Path(get_package_prefix("studica_vmxpi_ros2"))
    executable = prefix / "lib" / "studica_vmxpi_ros2" / "safety_supervisor_node"
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "hardware_mode:=true",
            "-p",
            "publish_rate_hz:=50.0",
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
            lambda: node.safety_publisher.get_subscription_count() == 1,
            5.0,
        )

        enabled = safety_message([1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 0.0])
        spin_until(
            node,
            lambda: node.state == "READY_DISARMED"
            and node.reason == "HARDWARE_DISARMED_WAITING_FOR_LOCAL_RELEASE",
            2.0,
            lambda: node.safety_publisher.publish(enabled),
        )

        ready = safety_message()
        spin_until(
            node,
            lambda: node.reason == "HARDWARE_READY_LOCAL_ENABLE_OFF",
            2.0,
            lambda: node.safety_publisher.publish(ready),
        )

        spin_until(
            node,
            lambda: node.state == "ARMED",
            2.0,
            lambda: node.safety_publisher.publish(enabled),
        )
        command = Twist()
        command.linear.x = 0.2

        def publish_enabled_command():
            node.safety_publisher.publish(enabled)
            node.command_publisher.publish(command)

        spin_until(
            node,
            lambda: node.output.twist.linear.x > 0.01,
            2.0,
            publish_enabled_command,
        )

        spin_until(node, node.disarm_client.service_is_ready, 2.0)
        future = node.disarm_client.call_async(Trigger.Request())
        spin_until(
            node,
            future.done,
            2.0,
            lambda: node.safety_publisher.publish(enabled),
        )
        assert future.result().success
        spin_until(
            node,
            lambda: node.state == "READY_DISARMED"
            and abs(node.output.twist.linear.x) < 1.0e-9,
            2.0,
            lambda: node.safety_publisher.publish(enabled),
        )
        assert node.reason == "HARDWARE_DISARMED_WAITING_FOR_LOCAL_RELEASE"

        node.safety_publisher.publish(ready)
        spin_until(
            node,
            lambda: node.reason == "HARDWARE_READY_LOCAL_ENABLE_OFF",
            2.0,
            lambda: node.safety_publisher.publish(ready),
        )
        spin_until(
            node,
            lambda: node.state == "ARMED",
            2.0,
            lambda: node.safety_publisher.publish(enabled),
        )

        spin_until(
            node,
            lambda: node.state == "FAULT"
            and node.reason == "HARDWARE_SAFETY_STALE",
            2.0,
        )
        spin_for(node, 0.3, lambda: node.safety_publisher.publish(enabled))
        assert node.state == "FAULT"
        spin_until(
            node,
            lambda: node.state == "READY_DISARMED",
            2.0,
            lambda: node.safety_publisher.publish(ready),
        )

        malformed = safety_message([1.0, 1.0])
        spin_until(
            node,
            lambda: node.state == "FAULT"
            and node.reason == "HARDWARE_SAFETY_MALFORMED",
            2.0,
            lambda: node.safety_publisher.publish(malformed),
        )

        spin_until(
            node,
            lambda: node.state == "READY_DISARMED",
            2.0,
            lambda: node.safety_publisher.publish(ready),
        )
        competing_publisher = node.create_publisher(
            DynamicJointState, "/dynamic_joint_states", 10
        )
        spin_until(
            node,
            lambda: node.count_publishers("/dynamic_joint_states") == 2,
            3.0,
        )
        spin_until(
            node,
            lambda: node.state == "FAULT"
            and node.reason == "HARDWARE_SAFETY_SOURCE_CONFLICT",
            2.0,
            lambda: competing_publisher.publish(ready),
        )
        node.destroy_publisher(competing_publisher)
        spin_until(
            node,
            lambda: node.count_publishers("/dynamic_joint_states") == 1,
            5.0,
        )
        spin_until(
            node,
            lambda: node.state == "READY_DISARMED",
            3.0,
            lambda: node.safety_publisher.publish(ready),
        )
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
        if process.returncode not in (0, -15):
            print(output, file=sys.stderr)
            raise RuntimeError(f"safety supervisor exited with {process.returncode}")


if __name__ == "__main__":
    main()
