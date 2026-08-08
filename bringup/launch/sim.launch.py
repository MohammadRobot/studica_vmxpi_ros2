# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Beginner simulation: one robot, one maze, and the standard ROS topic API."""

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
    gui = LaunchConfiguration("gui")
    gz_headless = LaunchConfiguration("gz_headless")
    use_camera = LaunchConfiguration("use_camera")
    world = LaunchConfiguration("world")

    arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2."),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without its graphical window.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="false",
            description="Enable simulated color and depth images.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value="maze",
            description="Bundled world name or absolute SDF path.",
        ),
    ]

    robot = deferred_include(
        "studica_vmxpi_ros2",
        "bringup.launch.py",
        {
            "mode": "gz_sim",
            "robot_profile": "class_4wd",
            "world": world,
            "gui": gui,
            "gz_headless": gz_headless,
            "use_ground_truth_odom_tf": "false",
            "use_lidar": "true",
            "sim_enable_camera": use_camera,
            "sim_camera_width": "640",
            "sim_camera_height": "480",
            "sim_camera_update_rate": "10.0",
            "sim_lidar_samples": "180",
            "sim_lidar_update_rate": "10.0",
            "sim_lidar_visualize": "false",
            "sim_imu_update_rate": "50.0",
            "use_monitoring": "true",
            "use_foxglove": "false",
        },
    )

    return LaunchDescription(arguments + [robot])
