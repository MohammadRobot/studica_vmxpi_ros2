# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Supervised class_4wd hardware launch with conservative sensor settings."""

from launch import LaunchDescription
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _classroom_launch import deferred_include  # noqa: E402


def generate_launch_description():
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    use_camera = LaunchConfiguration("use_camera")
    use_foxglove = LaunchConfiguration("use_foxglove")

    arguments = [
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Start the Orbbec color and depth camera.",
        ),
        DeclareLaunchArgument(
            "use_foxglove",
            default_value="true",
            description="Start the read-only Foxglove bridge.",
        ),
        DeclareLaunchArgument(
            "foxglove_address",
            default_value="127.0.0.1",
            description="Foxglove bind address; supply a trusted LAN address for remote access.",
        ),
        DeclareLaunchArgument(
            "foxglove_port",
            default_value="8765",
            description="Foxglove WebSocket port.",
        ),
    ]

    robot = deferred_include(
        "studica_vmxpi_ros2",
        "bringup.launch.py",
        {
            "mode": "hardware",
            "robot_profile": "class_4wd",
            "gui": "false",
            "use_lidar": "true",
            "use_camera": use_camera,
            "use_monitoring": "true",
            "use_foxglove": use_foxglove,
            "foxglove_address": foxglove_address,
            "foxglove_port": foxglove_port,
            "orbbec_enable_point_cloud": "false",
            "orbbec_enable_color": "true",
            "orbbec_enable_depth": "true",
            "orbbec_enable_ir": "false",
            "orbbec_color_width": "640",
            "orbbec_color_height": "480",
            "orbbec_color_fps": "15",
            "orbbec_depth_width": "640",
            "orbbec_depth_height": "480",
            "orbbec_depth_fps": "15",
        },
    )

    return LaunchDescription(arguments + [robot])
