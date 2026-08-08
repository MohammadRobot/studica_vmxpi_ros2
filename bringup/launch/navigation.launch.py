# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Beginner Nav2 lesson in the bundled office world with no embedded teleop."""

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
    autostart = LaunchConfiguration("autostart")
    gui = LaunchConfiguration("gui")
    gz_headless = LaunchConfiguration("gz_headless")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2."),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without its graphical window.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "maps", "office_map.yaml"]
            ),
            description="Map YAML used by AMCL.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"]
            ),
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes.",
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
            "rviz_config_file": PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description", "robot", "rviz", "nav2_navigation.rviz"]
            ),
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
    nav2 = deferred_include(
        "nav2_bringup",
        "bringup_launch.py",
        {
            "slam": "False",
            "map": map_file,
            "params_file": nav2_params_file,
            "use_sim_time": "true",
            "autostart": autostart,
            "use_composition": "False",
        },
    )

    return LaunchDescription(arguments + [robot, nav2])
