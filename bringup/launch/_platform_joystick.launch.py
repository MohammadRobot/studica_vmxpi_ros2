# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Internal direct Bluetooth joystick input for the managed robot platform."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("joystick_config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joystick_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("studica_vmxpi_ros2"),
                        "config",
                        "dualshock4_teleop.yaml",
                    ]
                ),
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="platform_joy",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="platform_joystick_teleop",
                output="screen",
                parameters=[config],
                remappings=[("cmd_vel", "/robot/control/joystick")],
            ),
        ]
    )
