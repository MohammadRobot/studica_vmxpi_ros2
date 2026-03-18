# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Gazebo Sim navigation wrapper (Nav2 localization + unified bringup)."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_nav2_params_file(context, nav2_share: str) -> str:
    # Use custom params when provided; otherwise keep nav2_bringup defaults.
    configured = LaunchConfiguration("nav2_params_file").perform(context).strip()
    if configured:
        return configured
    return os.path.join(nav2_share, "params", "nav2_params.yaml")


def _maybe_include_nav2(context, *args, **kwargs):
    map_path = LaunchConfiguration("map").perform(context).strip()
    if not map_path:
        return [LogInfo(msg="Map path is empty. Pass map:=/absolute/path/to/map.yaml to start Nav2.")]
    if not os.path.exists(map_path):
        return [LogInfo(msg=f"Map file not found: {map_path}")]

    try:
        nav2_share = get_package_share_directory("nav2_bringup")
    except PackageNotFoundError:
        return [LogInfo(msg="nav2_bringup not found; install ros-humble-nav2-bringup.")]

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    params_file = _resolve_nav2_params_file(context, nav2_share)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
            launch_arguments={
                "slam": "False",
                # Disable composition to keep node graph transparent for students.
                "use_composition": "False",
                "map": map_path,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 from robot launch.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/robot/rviz", "nav2_navigation.rviz"]
            ),
            description="Absolute path to RViz config file.",
        ),
        DeclareLaunchArgument(
            "robot_profile",
            default_value="class_4wd",
            description="Robot profile under config/profiles.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gz/worlds", "diff_drive_world.sdf"]
            ),
            description="Absolute path to Gazebo Sim world file.",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="default",
            description="World name to spawn into.",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="0.0",
            description="Initial robot spawn x (meters).",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="0.0",
            description="Initial robot spawn y (meters).",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="0.10",
            description="Initial robot spawn z (meters).",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="0.0",
            description="Initial robot spawn yaw (radians).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "use_ground_truth_odom_tf",
            default_value="false",
            description=(
                "In gz_sim, source /odom and /tf from Gazebo ground-truth odometry topics. "
                "Set false to use controller odometry TF."
            ),
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
        ),
        DeclareLaunchArgument(
            "joystick_cmd_vel_topic",
            default_value="",
            description="Joystick command velocity output topic (empty = auto from drive profile).",
        ),
        DeclareLaunchArgument(
            "joystick_publish_stamped",
            default_value="true",
            description="Publish TwistStamped joystick commands.",
        ),
        DeclareLaunchArgument(
            "bridge_drive_cmd_topic",
            default_value="",
            description="Override Nav2 bridge output command topic (empty = auto from drive profile).",
        ),
        DeclareLaunchArgument(
            "bridge_drive_odom_topic",
            default_value="",
            description="Override Nav2 bridge input odom topic (empty = auto from drive profile).",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Absolute path to map yaml file for Nav2 localization.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Autostart Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value="",
            description="Path to Nav2 parameter file (leave empty to use nav2_bringup default).",
        ),
    ]

    gui = LaunchConfiguration("gui")
    robot_profile = LaunchConfiguration("robot_profile")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    world = LaunchConfiguration("world")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf")
    use_joystick = LaunchConfiguration("use_joystick")
    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic")
    joystick_publish_stamped = LaunchConfiguration("joystick_publish_stamped")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "launch", "bringup.launch.py"])
        ),
        launch_arguments={
            "mode": "gz_sim",
            "gui": gui,
            "world": world,
            "world_name": world_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "use_sim_time": use_sim_time,
            "use_ground_truth_odom_tf": use_ground_truth_odom_tf,
            "use_joystick": use_joystick,
            "joystick_cmd_vel_topic": joystick_cmd_vel_topic,
            "joystick_publish_stamped": joystick_publish_stamped,
            "rviz_config_file": rviz_config_file,
            "robot_profile": robot_profile,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [robot, OpaqueFunction(function=_maybe_include_nav2)]
    )
