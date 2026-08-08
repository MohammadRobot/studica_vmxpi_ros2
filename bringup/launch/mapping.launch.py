# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Beginner SLAM lesson in the bundled office world; teleop stays external."""

from launch import LaunchDescription
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _classroom_launch import deferred_include  # noqa: E402


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    gz_headless = LaunchConfiguration("gz_headless")
    slam_params_file = LaunchConfiguration("slam_params_file")

    arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2."),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without its graphical window.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "config", "slam_toolbox_mapper_params.yaml"]
            ),
            description="SLAM Toolbox parameter file.",
        ),
    ]

    robot = deferred_include(
        "studica_vmxpi_ros2",
        "bringup.launch.py",
        {
            "mode": "gz_sim",
            "robot_profile": "class_4wd",
            "world": "office_map",
            "gui": gui,
            "gz_headless": gz_headless,
            "use_ground_truth_odom_tf": "false",
            "use_lidar": "true",
            "sim_enable_camera": "false",
            "sim_lidar_samples": "180",
            "sim_lidar_update_rate": "10.0",
            "sim_lidar_visualize": "false",
            "sim_imu_update_rate": "50.0",
            "use_monitoring": "true",
            "use_foxglove": "false",
        },
    )
    slam = deferred_include(
        "slam_toolbox",
        "online_async_launch.py",
        {
            "use_sim_time": "true",
            "slam_params_file": slam_params_file,
        },
    )

    return LaunchDescription(arguments + [robot, slam])
