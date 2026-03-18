# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Legacy Gazebo Classic navigation wrapper (Nav2 + base bringup)."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


DEFAULT_DRIVE_CONTROLLER_NAME = "robot_base_controller"
DEFAULT_DRIVE_CONTROLLER_TYPE = "diff_drive_controller/DiffDriveController"
MECANUM_DRIVE_CONTROLLER_TYPE = "mecanum_drive_controller/MecanumDriveController"


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a mapping: {path}")
    return data


def _drive_topics(controller_name: str, controller_type: str):
    if controller_type == MECANUM_DRIVE_CONTROLLER_TYPE:
        return f"/{controller_name}/reference", f"/{controller_name}/odometry"
    return f"/{controller_name}/cmd_vel", f"/{controller_name}/odom"


def _resolve_profile_drive_topics(profile_name: str):
    controller_name = DEFAULT_DRIVE_CONTROLLER_NAME
    controller_type = DEFAULT_DRIVE_CONTROLLER_TYPE

    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    profile_file = os.path.join(pkg_share, "config", "profiles", profile_name, "robot_profile.yaml")
    if not os.path.exists(profile_file):
        return _drive_topics(controller_name, controller_type)

    try:
        profile = _load_yaml(profile_file)
    except Exception:
        return _drive_topics(controller_name, controller_type)

    drive_cfg = profile.get("drive")
    if isinstance(drive_cfg, dict):
        profile_controller_name = str(
            drive_cfg.get("controller_name", DEFAULT_DRIVE_CONTROLLER_NAME)
        ).strip()
        profile_controller_type = str(
            drive_cfg.get("controller_type", DEFAULT_DRIVE_CONTROLLER_TYPE)
        ).strip()
        if profile_controller_name:
            controller_name = profile_controller_name
        if profile_controller_type:
            controller_type = profile_controller_type

    return _drive_topics(controller_name, controller_type)


def _build_nav2_bridge(context, *args, **kwargs):
    use_gazebo_classic = _is_true(LaunchConfiguration("use_gazebo_classic").perform(context))
    use_hardware = _is_true(LaunchConfiguration("use_hardware").perform(context))
    bridge_needed = use_gazebo_classic or not use_hardware
    if not bridge_needed:
        return []

    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip() or "training_4wd"
    drive_cmd_topic = LaunchConfiguration("bridge_drive_cmd_topic").perform(context).strip()
    drive_odom_topic = LaunchConfiguration("bridge_drive_odom_topic").perform(context).strip()

    if not drive_cmd_topic or not drive_odom_topic:
        resolved_cmd_topic, resolved_odom_topic = _resolve_profile_drive_topics(robot_profile)
        if not drive_cmd_topic:
            drive_cmd_topic = resolved_cmd_topic
        if not drive_odom_topic:
            drive_odom_topic = resolved_odom_topic

    return [
        Node(
            package="studica_vmxpi_ros2",
            executable="topic_adapter_node",
            name="nav2_topic_bridge",
            output="screen",
            parameters=[
                {
                    "use_sim_time": _is_true(LaunchConfiguration("use_sim_time").perform(context)),
                    "enable_nav2_bridge": True,
                    "input_cmd_vel_topic": "/cmd_vel",
                    "output_cmd_vel_topic": drive_cmd_topic,
                    "input_odom_topic": drive_odom_topic,
                    "output_odom_topic": "/odom",
                    "cmd_vel_frame_id": "base_link",
                }
            ],
        )
    ]


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

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
            launch_arguments={
                "slam": "False",
                "use_composition": "False",
                "map": map_path,
                "params_file": (
                    LaunchConfiguration("nav2_params_file").perform(context).strip()
                    or os.path.join(nav2_share, "params", "nav2_params.yaml")
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
                "autostart": LaunchConfiguration("autostart").perform(context),
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
            "robot_profile",
            default_value="training_4wd",
            description="Robot profile under config/profiles.",
        ),
        DeclareLaunchArgument(
            "use_hardware",
            default_value="false",
            description="Use Titan hardware instead of mock system.",
        ),
        DeclareLaunchArgument(
            "use_gazebo_classic",
            default_value="true",
            description="Start Gazebo Classic simulation.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gazebo/worlds", "diff_drive_world.world"]
            ),
            description="Absolute path to Gazebo world file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=LaunchConfiguration("use_gazebo_classic"),
            description="Use simulation time (defaults to use_gazebo_classic).",
        ),
        DeclareLaunchArgument(
            "use_joystick",
            default_value="false",
            description="Launch joystick teleop from studica_ros2_control.",
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
    use_hardware = LaunchConfiguration("use_hardware")
    use_gazebo_classic = LaunchConfiguration("use_gazebo_classic")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joystick = LaunchConfiguration("use_joystick")
    mode = PythonExpression(
        [
            "'gazebo_classic' if ('",
            use_gazebo_classic,
            "').lower() in ['true','1','yes','on'] else "
            "('hardware' if ('",
            use_hardware,
            "').lower() in ['true','1','yes','on'] else 'mock')",
        ]
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "launch", "bringup.launch.py"]
            )
        ),
        launch_arguments={
            "mode": mode,
            "gui": gui,
            "world": world,
            "use_sim_time": use_sim_time,
            "use_joystick": use_joystick,
            "robot_profile": robot_profile,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [robot, OpaqueFunction(function=_build_nav2_bridge), OpaqueFunction(function=_maybe_include_nav2)]
    )
