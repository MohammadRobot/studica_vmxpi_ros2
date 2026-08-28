#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Black-box joystick source and deadman contract test."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def joy_message(*, deadman=False, turbo=False, button_count=6):
    message = Joy()
    message.buttons = [0] * button_count
    if button_count > 4:
        message.buttons[4] = int(deadman)
    if button_count > 5:
        message.buttons[5] = int(turbo)
    return message


class Probe(Node):
    def __init__(self):
        super().__init__("joystick_supervisor_contract_probe")
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state = ""
        self.reason = ""
        self.reasons = []
        self.output = TwistStamped()
        self.joy_publisher = self.create_publisher(Joy, "/joy", 10)
        self.joystick_command_publisher = self.create_publisher(
            Twist, "/cmd_vel/joy", 10
        )
        self.application_command_publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )
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
        self.arm_client = self.create_client(Trigger, "/robot/arm")

    def _state(self, message):
        self.state = message.data

    def _reason(self, message):
        self.reason = message.data
        self.reasons.append(message.data)

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
    os.environ["ROS_DOMAIN_ID"] = str(180 + os.getpid() % 40)
    prefix = Path(get_package_prefix("studica_vmxpi_ros2"))
    executable = prefix / "lib" / "studica_vmxpi_ros2" / "safety_supervisor_node"
    process = subprocess.Popen(
        [
            str(executable),
            "--ros-args",
            "-p",
            "allow_software_arm:=true",
            "-p",
            "control_source:=joystick",
            "-p",
            "command_timeout_sec:=0.2",
            "-p",
            "joystick_state_timeout_sec:=0.5",
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
            lambda: node.joy_publisher.get_subscription_count() == 1
            and node.joystick_command_publisher.get_subscription_count() == 1,
            5.0,
        )
        assert node.application_command_publisher.get_subscription_count() == 0

        spin_until(node, lambda: node.state == "READY_DISARMED", 2.0)
        spin_until(node, node.arm_client.service_is_ready, 2.0)
        future = node.arm_client.call_async(Trigger.Request())
        spin_until(node, future.done, 2.0)
        assert future.result().success

        command = Twist()
        command.linear.x = 0.2
        held = joy_message(deadman=True)

        def publish_held_command():
            node.joy_publisher.publish(held)
            node.joystick_command_publisher.publish(command)

        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_DEADMAN_RELEASE_REQUIRED"
            and abs(node.output.twist.linear.x) < 1.0e-9,
            2.0,
            publish_held_command,
        )

        turbo_only = joy_message(turbo=True)

        def publish_turbo_without_deadman():
            node.joy_publisher.publish(turbo_only)
            node.joystick_command_publisher.publish(command)

        spin_for(node, 0.3, publish_turbo_without_deadman)
        assert abs(node.output.twist.linear.x) < 1.0e-9
        assert node.reason == "JOYSTICK_DEADMAN_RELEASED"

        both_buttons = joy_message(deadman=True, turbo=True)

        def publish_enabled_turbo():
            node.joy_publisher.publish(both_buttons)
            node.joystick_command_publisher.publish(command)

        spin_until(
            node,
            lambda: node.output.twist.linear.x > 0.01,
            2.0,
            publish_enabled_turbo,
        )

        node.joy_publisher.publish(joy_message())
        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_DEADMAN_RELEASED"
            and abs(node.output.twist.linear.x) < 1.0e-9,
            2.0,
        )
        spin_until(
            node,
            lambda: node.output.twist.linear.x > 0.01,
            2.0,
            publish_held_command,
        )

        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_STATE_STALE"
            and abs(node.output.twist.linear.x) < 1.0e-9,
            2.0,
        )
        spin_for(node, 0.3, publish_held_command)
        assert abs(node.output.twist.linear.x) < 1.0e-9
        assert node.reason == "JOYSTICK_DEADMAN_RELEASE_REQUIRED"
        node.joy_publisher.publish(joy_message())
        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_DEADMAN_RELEASED",
            2.0,
        )
        spin_until(
            node,
            lambda: node.output.twist.linear.x > 0.01,
            2.0,
            publish_held_command,
        )

        node.destroy_publisher(node.joystick_command_publisher)
        spin_until(
            node,
            lambda: node.count_publishers("/cmd_vel/joy") == 0,
            2.0,
        )
        spin_until(
            node,
            lambda: "COMMAND_SOURCE_LOST" in node.reasons
            and abs(node.output.twist.linear.x) < 1.0e-9,
            1.0,
        )
        node.joystick_command_publisher = node.create_publisher(
            Twist, "/cmd_vel/joy", 10
        )
        spin_until(
            node,
            lambda: node.joystick_command_publisher.get_subscription_count() == 1,
            2.0,
        )
        spin_for(node, 0.3, publish_held_command)
        assert abs(node.output.twist.linear.x) < 1.0e-9
        node.joy_publisher.publish(joy_message())
        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_DEADMAN_RELEASED",
            2.0,
        )
        spin_until(
            node,
            lambda: node.output.twist.linear.x > 0.01,
            2.0,
            publish_held_command,
        )

        competing_joy = node.create_publisher(Joy, "/joy", 10)
        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_STATE_SOURCE_CONFLICT"
            and abs(node.output.twist.linear.x) < 1.0e-9,
            2.0,
            lambda: competing_joy.publish(held),
        )
        node.destroy_publisher(competing_joy)
        spin_until(
            node,
            lambda: node.joy_publisher.get_subscription_count() == 1,
            2.0,
        )

        malformed = joy_message(button_count=2)
        spin_until(
            node,
            lambda: node.reason == "JOYSTICK_STATE_MALFORMED",
            2.0,
            lambda: node.joy_publisher.publish(malformed),
        )
        assert abs(node.output.twist.linear.x) < 1.0e-9
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
