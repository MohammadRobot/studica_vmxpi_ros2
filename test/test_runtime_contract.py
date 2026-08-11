#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Black-box contract test for the mock and optional headless Gazebo runtimes."""

import argparse
import os
from pathlib import Path
import random
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import yaml


TOPIC_TYPES = {
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/imu": "sensor_msgs/msg/Imu",
    "/joint_states": "sensor_msgs/msg/JointState",
    "/odom": "nav_msgs/msg/Odometry",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
}


def run_cli(arguments, env, timeout=12, check=True):
    result = subprocess.run(
        arguments,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"Command failed ({result.returncode}): {' '.join(arguments)}\n"
            f"stdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=3)


def parse_first_yaml(output):
    lines = output.splitlines()
    try:
        first_header = lines.index("header:")
    except ValueError as error:
        raise RuntimeError(f"No ROS message found in output:\n{output}") from error
    document = "\n".join(lines[first_header:]).split("---", maxsplit=1)[0].strip()
    result = yaml.safe_load(document)
    if not isinstance(result, dict):
        raise RuntimeError(f"Expected a YAML message, received:\n{output}")
    return result


def echo_once(topic, env, timeout=12):
    return parse_first_yaml(
        run_cli(["ros2", "topic", "echo", topic, "--once"], env, timeout=timeout).stdout
    )


def wait_until_ready(process, env, timeout=40):
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Launch exited early with code {process.returncode}")
        try:
            result = run_cli(
                ["ros2", "control", "list_controllers"], env, timeout=5, check=False
            )
        except subprocess.TimeoutExpired:
            time.sleep(1.0)
            continue
        last_output = result.stdout
        if (
            "joint_state_broadcaster" in last_output
            and "robot_base_controller" in last_output
            and last_output.count("active") >= 2
        ):
            return
        time.sleep(1.0)
    raise RuntimeError(f"Controllers did not become active:\n{last_output}")


def verify_topic_contract(env, timeout=20):
    """Wait for DDS discovery to expose the complete classroom topic graph."""
    deadline = time.monotonic() + timeout
    output = ""
    missing = []
    while time.monotonic() < deadline:
        result = run_cli(
            ["ros2", "topic", "list", "--no-daemon", "--spin-time", "2", "-t"],
            env,
            timeout=10,
            check=False,
        )
        output = result.stdout
        missing = [
            f"{topic} [{message_type}]"
            for topic, message_type in TOPIC_TYPES.items()
            if f"{topic} [{message_type}]" not in output
        ]
        if not missing:
            return
        time.sleep(0.5)
    raise RuntimeError(
        "DDS discovery did not expose the complete topic contract; missing "
        f"{missing}:\n{output}"
    )


def verify_twist_adapter(env):
    subscriber = subprocess.Popen(
        ["ros2", "topic", "echo", "/robot_base_controller/cmd_vel", "--once"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    try:
        time.sleep(1.0)
        run_cli(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/cmd_vel",
                "geometry_msgs/msg/Twist",
                "{linear: {x: 0.2}, angular: {z: -0.1}}",
            ],
            env,
            timeout=12,
        )
        output, _ = subscriber.communicate(timeout=12)
    finally:
        stop_process(subscriber)

    message = parse_first_yaml(output)
    if message["header"]["frame_id"] != "base_link":
        raise RuntimeError(f"Unexpected command frame: {message}")
    if abs(float(message["twist"]["linear"]["x"]) - 0.2) > 1e-6:
        raise RuntimeError(f"Twist adapter changed the command: {message}")


def verify_sensor_and_odom_aliases(env):
    if echo_once("/odom", env)["header"]["frame_id"] != "odom":
        raise RuntimeError("/odom does not use the odom frame")
    if echo_once("/imu", env)["header"]["frame_id"] != "imu_link":
        raise RuntimeError("/imu does not use the imu_link frame")
    if echo_once("/scan", env)["header"]["frame_id"] != "laser_scan_frame":
        raise RuntimeError("/scan does not use the laser_scan_frame frame")


def verify_command_timeout(env):
    publisher = subprocess.Popen(
        [
            "ros2",
            "topic",
            "pub",
            "-r",
            "10",
            "/cmd_vel",
            "geometry_msgs/msg/Twist",
            "{linear: {x: 0.2}}",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        time.sleep(1.5)
        moving = echo_once("/joint_states", env)
    finally:
        stop_process(publisher)

    moving_speeds = [abs(float(value)) for value in moving.get("velocity", [])]
    if not moving_speeds or max(moving_speeds) < 0.2:
        raise RuntimeError(f"Wheel feedback did not respond to /cmd_vel: {moving}")

    time.sleep(1.5)
    stopped = echo_once("/joint_states", env)
    stopped_speeds = [abs(float(value)) for value in stopped.get("velocity", [])]
    if not stopped_speeds or max(stopped_speeds) > 0.1:
        raise RuntimeError(f"Command timeout did not stop wheel feedback: {stopped}")


def verify_command_publisher_ownership(mode, env):
    output = run_cli(
        ["ros2", "topic", "info", "/cmd_vel", "--no-daemon", "-v"], env
    ).stdout
    match = re.search(r"Publisher count:\s*(\d+)", output)
    if match is None:
        raise RuntimeError(f"Could not inspect /cmd_vel ownership:\n{output}")
    publisher_count = int(match.group(1))
    lowered = output.lower()
    if "teleop" in lowered or "patrol" in lowered:
        raise RuntimeError(f"Unexpected embedded command publisher:\n{output}")
    if mode == "mapping" and publisher_count != 0:
        raise RuntimeError(f"Mapping must leave /cmd_vel to external teleop:\n{output}")
    if mode == "navigation" and publisher_count > 0:
        nav2_publishers = (
            "controller_server" in lowered or "velocity_smoother" in lowered
        )
        if not nav2_publishers:
            raise RuntimeError(f"Navigation command publisher is not owned by Nav2:\n{output}")


def dependencies_available(env):
    if shutil.which("gz") is None:
        return False
    for package in ("ros_gz_sim", "ros_gz_bridge", "gz_ros2_control"):
        if run_cli(["ros2", "pkg", "prefix", package], env, check=False).returncode != 0:
            return False
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode", choices=("mock", "gz", "mapping", "navigation"), required=True
    )
    parser.add_argument("--allow-skip", action="store_true")
    args = parser.parse_args()

    env = os.environ.copy()
    env.pop("ROS_DISCOVERY_SERVER", None)
    cyclone_config = (
        Path(__file__).resolve().parents[1]
        / "bringup"
        / "config"
        / "network"
        / "cyclonedds_sim.xml"
    )
    env.update(
        {
            "CYCLONEDDS_URI": cyclone_config.as_uri(),
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            "ROS_DOMAIN_ID": str(random.randint(101, 220)),
            # The checked-in Cyclone profile explicitly selects loopback. Keep
            # ROS_LOCALHOST_ONLY off so Cyclone does not select `lo` a second time.
            "ROS_LOCALHOST_ONLY": "0",
        }
    )

    if args.mode != "mock" and not dependencies_available(env):
        if args.allow_skip:
            print(
                f"[SKIP] {args.mode}: Gazebo Harmonic ROS packages or the gz executable "
                "are unavailable"
            )
            return 0
        raise RuntimeError("Gazebo Harmonic runtime dependencies are unavailable")

    if args.mode == "mock":
        launch_arguments = [
            "bringup.launch.py",
            "mode:=mock",
            "gui:=false",
            "use_lidar:=false",
            "use_camera:=false",
            "use_monitoring:=false",
            "use_foxglove:=false",
            "use_joystick:=false",
        ]
    elif args.mode == "gz":
        launch_arguments = [
            "sim.launch.py",
            "gui:=false",
            "gz_headless:=true",
            "use_camera:=false",
            "use_joystick:=false",
        ]
    elif args.mode == "mapping":
        launch_arguments = [
            "mapping.launch.py",
            "gui:=false",
            "gz_headless:=true",
            "use_joystick:=false",
        ]
    else:
        launch_arguments = [
            "navigation.launch.py",
            "gui:=false",
            "gz_headless:=true",
            "use_joystick:=false",
        ]

    with tempfile.TemporaryDirectory(prefix="studica_runtime_test_") as temp_dir:
        log_path = Path(temp_dir) / "launch.log"
        with log_path.open("w+", encoding="utf-8") as launch_log:
            process = subprocess.Popen(
                ["ros2", "launch", "studica_vmxpi_ros2", *launch_arguments],
                env=env,
                stdout=launch_log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                wait_until_ready(process, env, timeout=35 if args.mode == "mock" else 75)
                verify_topic_contract(env)
                if args.mode in ("mapping", "navigation"):
                    verify_command_publisher_ownership(args.mode, env)
                else:
                    verify_twist_adapter(env)
                    verify_sensor_and_odom_aliases(env)
                    verify_command_timeout(env)
            except Exception:  # pylint: disable=broad-except
                launch_log.flush()
                launch_log.seek(0)
                print(launch_log.read(), file=sys.stderr)
                raise
            finally:
                stop_process(process)

    print(f"[PASS] {args.mode} runtime classroom contract")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # pylint: disable=broad-except
        print(f"[FAIL] {error}", file=sys.stderr)
        sys.exit(1)
