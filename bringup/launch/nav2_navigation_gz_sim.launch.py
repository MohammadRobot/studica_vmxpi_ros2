# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Gazebo Sim navigation wrapper (Nav2 localization + unified bringup)."""

import os
import tempfile

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
import yaml


def _resolve_nav2_params_file(context, nav2_share: str) -> str:
    # Use custom params when provided; otherwise keep nav2_bringup defaults.
    configured = LaunchConfiguration("nav2_params_file").perform(context).strip()
    if configured:
        return configured
    return os.path.join(nav2_share, "params", "nav2_params.yaml")


def _is_true(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def _float_arg(context, name: str) -> float:
    value = LaunchConfiguration(name).perform(context).strip()
    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(f"Invalid {name} '{value}'. Expected a floating point value.") from exc


def _configured_nav2_params_file(context, params_file: str) -> str:
    with open(params_file, "r", encoding="utf-8") as stream:
        params = yaml.safe_load(stream) or {}

    amcl_params = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl_params["set_initial_pose"] = _is_true(
        LaunchConfiguration("set_initial_pose").perform(context)
    )
    amcl_params["initial_pose.x"] = _float_arg(context, "initial_pose_x")
    amcl_params["initial_pose.y"] = _float_arg(context, "initial_pose_y")
    amcl_params["initial_pose.z"] = _float_arg(context, "initial_pose_z")
    amcl_params["initial_pose.yaw"] = _float_arg(context, "initial_pose_yaw")

    configured = tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="studica_nav2_params_",
        suffix=".yaml",
        delete=False,
    )
    with configured:
        yaml.safe_dump(params, configured)
    return configured.name


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
    use_sim_time_bool = _is_true(use_sim_time)
    use_sim_time_value = "true" if use_sim_time_bool else "false"
    autostart = LaunchConfiguration("autostart").perform(context)
    params_file = _resolve_nav2_params_file(context, nav2_share)
    configured_params = _configured_nav2_params_file(context, params_file)
    return [
        GroupAction(
            actions=[
                # Force sim time in the Nav2 scope even if upstream defaults leak through.
                SetParameter(name="use_sim_time", value=use_sim_time_bool),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_share, "launch", "bringup_launch.py")
                    ),
                    launch_arguments={
                        "slam": "False",
                        # Disable composition to keep node graph transparent for students.
                        "use_composition": "False",
                        "map": map_path,
                        "params_file": configured_params,
                        "use_sim_time": use_sim_time_value,
                        "autostart": autostart,
                    }.items(),
                ),
            ]
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
            default_value="",
            description="Initial robot spawn x (meters). Empty uses world default.",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="",
            description="Initial robot spawn y (meters). Empty uses world default.",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="",
            description="Initial robot spawn z (meters). Empty uses world default.",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="",
            description="Initial robot spawn yaw (radians). Empty uses world default.",
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
        DeclareLaunchArgument(
            "set_initial_pose",
            default_value="true",
            description=(
                "Set AMCL initial pose from initial_pose_* args. Set false to use RViz 2D Pose Estimate."
            ),
        ),
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="0.0",
            description="AMCL initial pose x in the map frame.",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="0.0",
            description="AMCL initial pose y in the map frame.",
        ),
        DeclareLaunchArgument(
            "initial_pose_z",
            default_value="0.0",
            description="AMCL initial pose z in the map frame.",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="0.0",
            description="AMCL initial pose yaw in the map frame.",
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
