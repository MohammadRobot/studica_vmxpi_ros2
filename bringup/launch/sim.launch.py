# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Beginner simulation: one robot, one maze, and the standard ROS topic API."""

from launch import LaunchDescription
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _classroom_launch import deferred_include  # noqa: E402


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    gz_headless = LaunchConfiguration("gz_headless")
    use_camera = LaunchConfiguration("use_camera")
    use_point_cloud = LaunchConfiguration("use_point_cloud")
    use_joystick = LaunchConfiguration("use_joystick")
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
            "use_point_cloud",
            default_value="false",
            description="Publish raw and floor-filtered PointCloud2 topics; enables the camera.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="true",
            description="Start the default DualShock joystick teleop nodes.",
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
            "sim_enable_camera": PythonExpression(
                [
                    "'",
                    use_camera,
                    "'.lower() == 'true' or '",
                    use_point_cloud,
                    "'.lower() == 'true'",
                ]
            ),
            "sim_camera_width": "640",
            "sim_camera_height": "480",
            "sim_camera_update_rate": "10.0",
            "sim_lidar_samples": "180",
            "sim_lidar_update_rate": "10.0",
            "sim_lidar_visualize": "false",
            "sim_imu_update_rate": "50.0",
            "use_monitoring": "true",
            "use_foxglove": "false",
            "use_joystick": use_joystick,
        },
    )

    point_cloud = Node(
        package="studica_vmxpi_ros2",
        executable="depth_to_pointcloud.py",
        name="depth_to_pointcloud",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "stride": 4,
                "min_depth_m": 0.1,
                "max_depth_m": 10.0,
            }
        ],
        condition=IfCondition(use_point_cloud),
    )

    point_cloud_filter = Node(
        package="studica_vmxpi_ros2",
        executable="point_cloud_filter.py",
        name="point_cloud_filter",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/camera/depth/points",
                "output_topic": "/camera/depth/points_filtered",
                "target_frame": "base_link",
                "min_height_m": 0.04,
                "max_height_m": 1.50,
                "min_forward_m": 0.15,
                "max_forward_m": 3.00,
                "max_lateral_m": 2.00,
                "robot_min_x_m": -0.20,
                "robot_max_x_m": 0.25,
                "robot_half_width_m": 0.21,
                "robot_max_height_m": 0.35,
            }
        ],
        condition=IfCondition(use_point_cloud),
    )

    return LaunchDescription(arguments + [robot, point_cloud, point_cloud_filter])
