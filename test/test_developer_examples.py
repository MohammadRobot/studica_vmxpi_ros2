#!/usr/bin/env python3
"""Exercise example processes against a telemetry-only loopback fixture."""
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def main():
    if {name for _, name in socket.if_nameindex()} != {'lo'}:
        raise RuntimeError('example tests require an isolated loopback network namespace')
    os.environ.update(ROS_DOMAIN_ID='95', ROS_LOCALHOST_ONLY='1',
                      RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    rclpy.init()
    node = Node('example_fixture')
    odom = node.create_publisher(Odometry, '/odom', 1)
    samples = []
    node.create_subscription(Twist, '/cmd_vel', lambda msg: samples.append(msg.linear.x), 1)
    binary = Path(get_package_prefix('studica_vmxpi_ros2')) / 'lib/studica_vmxpi_ros2'
    try:
        for executable, motion_args in [
                ('developer_example.py', ['--publish-motion']),
                ('developer_example_cpp', ['--ros-args', '-p', 'publish_motion:=true'])]:
            for args in ([], motion_args):
                samples.clear()
                log = tempfile.TemporaryFile()
                process = subprocess.Popen([str(binary / executable), *args],
                                           stdout=subprocess.DEVNULL, stderr=log)
                deadline = time.monotonic() + 7
                try:
                    while process.poll() is None and time.monotonic() < deadline:
                        odom.publish(Odometry())
                        rclpy.spin_once(node, timeout_sec=0.02)
                        time.sleep(0.02)
                    log.seek(0)
                    assert process.poll() == 0, log.read().decode()[-2000:]
                    if args:
                        assert any(value > 0 for value in samples), executable
                        assert all(0 <= value <= 0.05 for value in samples), samples
                        assert samples[-1] == 0, samples
                    else:
                        assert samples == [], executable
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.wait(timeout=2)
                    log.close()
        print('PASS: both examples default read-only; explicit pulses bounded and finish at zero')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
