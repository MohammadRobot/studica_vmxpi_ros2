# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""SLAM mapping in local simulation or against a separately launched VMXPi."""

from launch import LaunchDescription
import sys
from pathlib import Path

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _classroom_launch import deferred_include  # noqa: E402


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    gui = LaunchConfiguration("gui")
    gz_headless = LaunchConfiguration("gz_headless")
    use_joystick = LaunchConfiguration("use_joystick")
    joystick_config_file = LaunchConfiguration("joystick_config_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = PythonExpression(
        ["'true' if '", mode, "' == 'gz_sim' else 'false'"]
    )

    simulation_slam_params_file = PathJoinSubstitution(
        [
            FindPackageShare("studica_vmxpi_ros2"),
            "config",
            "slam_toolbox_mapper_params.yaml",
        ]
    )
    hardware_slam_params_file = PathJoinSubstitution(
        [
            FindPackageShare("studica_vmxpi_ros2"),
            "config",
            "slam_toolbox_hardware_mapper_params.yaml",
        ]
    )
    default_slam_params_file = PythonExpression(
        [
            "'",
            hardware_slam_params_file,
            "' if '",
            mode,
            "' == 'hardware' else '",
            simulation_slam_params_file,
            "'",
        ]
    )

    arguments = [
        DeclareLaunchArgument(
            "mode",
            default_value="gz_sim",
            choices=["gz_sim", "hardware"],
            description=(
                "Use gz_sim for the local office world or hardware for the PC-side "
                "client of a separately launched VMXPi."
            ),
        ),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2."),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Run Gazebo without its graphical window.",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="true",
            description="Start the default DualShock joystick teleop nodes.",
        ),
        DeclareLaunchArgument(
            "joystick_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "config", "dualshock4_teleop.yaml"]
            ),
            description="Joystick and teleop_twist_joy parameter file.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("studica_vmxpi_ros2"),
                    "description/robot/rviz",
                    "robot.rviz",
                ]
            ),
            description="RViz configuration; hardware mode forces its fixed frame to map.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params_file,
            description=(
                "SLAM Toolbox parameter file; defaults to the simulation or "
                "hardware-tuned profile selected by mode."
            ),
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
            "use_joystick": use_joystick,
        },
    )
    simulation = GroupAction(
        actions=[robot],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'gz_sim'"])),
    )

    hardware_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file, "-f", "map"],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(gui),
    )
    hardware_joystick = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[joystick_config_file],
        condition=IfCondition(use_joystick),
    )
    hardware_teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            joystick_config_file,
            {
                "scale_linear.x": 0.08,
                "scale_linear_turbo.x": 0.12,
                "scale_angular.yaw": 0.25,
                "scale_angular_turbo.yaw": 0.40,
            },
        ],
        remappings=[("cmd_vel", "/cmd_vel/joy")],
        condition=IfCondition(use_joystick),
    )
    hardware_client = GroupAction(
        actions=[hardware_rviz, hardware_joystick, hardware_teleop],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'hardware'"])),
    )

    slam = deferred_include(
        "slam_toolbox",
        "online_async_launch.py",
        {
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        },
    )

    return LaunchDescription(arguments + [simulation, hardware_client, slam])
