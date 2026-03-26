# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Hardware Orbbec camera launch with optional static TF configuration."""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _runtime_actions(context, *args, **kwargs):
    del args
    del kwargs

    orbbec_launch_file = LaunchConfiguration("orbbec_launch_file").perform(context).strip()
    if not orbbec_launch_file:
        orbbec_launch_file = "gemini_e.launch.py"

    camera_name = LaunchConfiguration("orbbec_camera_name").perform(context).strip() or "camera"
    serial_number = LaunchConfiguration("orbbec_serial_number").perform(context).strip()
    enable_point_cloud = (
        LaunchConfiguration("orbbec_enable_point_cloud").perform(context).strip() or "false"
    )

    camera_parent_frame = LaunchConfiguration("camera_parent_frame").perform(context).strip()
    camera_child_frame = LaunchConfiguration("camera_child_frame").perform(context).strip()
    if not camera_child_frame:
        camera_child_frame = f"{camera_name}_link"

    camera_tf_x = LaunchConfiguration("camera_tf_x").perform(context).strip()
    camera_tf_y = LaunchConfiguration("camera_tf_y").perform(context).strip()
    camera_tf_z = LaunchConfiguration("camera_tf_z").perform(context).strip()
    camera_tf_qx = LaunchConfiguration("camera_tf_qx").perform(context).strip()
    camera_tf_qy = LaunchConfiguration("camera_tf_qy").perform(context).strip()
    camera_tf_qz = LaunchConfiguration("camera_tf_qz").perform(context).strip()
    camera_tf_qw = LaunchConfiguration("camera_tf_qw").perform(context).strip()

    try:
        orbbec_pkg = get_package_share_directory("orbbec_camera")
    except PackageNotFoundError:
        return [LogInfo(msg="orbbec_camera not found; skipping Orbbec camera launch.")]

    launch_path = os.path.join(orbbec_pkg, "launch", orbbec_launch_file)
    if not os.path.exists(launch_path):
        return [LogInfo(msg=f"Orbbec launch file not found: {launch_path}")]

    actions = [
        LogInfo(msg=["Using Orbbec launch: ", launch_path]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                "camera_name": camera_name,
                "serial_number": serial_number,
                "enable_point_cloud": enable_point_cloud,
            }.items(),
        ),
    ]

    publish_camera_tf = LaunchConfiguration("publish_camera_tf").perform(context).strip()
    if _is_true(publish_camera_tf):
        actions.extend(
            [
                LogInfo(
                    msg=[
                        "Camera TF ",
                        camera_parent_frame,
                        " -> ",
                        camera_child_frame,
                        " quat=(",
                        camera_tf_qx,
                        ",",
                        camera_tf_qy,
                        ",",
                        camera_tf_qz,
                        ",",
                        camera_tf_qw,
                        ")",
                    ]
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_tf_pub_camera",
                    arguments=[
                        camera_tf_x,
                        camera_tf_y,
                        camera_tf_z,
                        camera_tf_qx,
                        camera_tf_qy,
                        camera_tf_qz,
                        camera_tf_qw,
                        camera_parent_frame,
                        camera_child_frame,
                    ],
                ),
            ]
        )

    return actions


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "orbbec_launch_file",
            default_value="gemini_e.launch.py",
            description="Orbbec launch file in orbbec_camera/launch.",
        ),
        DeclareLaunchArgument(
            "orbbec_camera_name",
            default_value="camera",
            description="Orbbec camera_name launch argument (also sets namespace).",
        ),
        DeclareLaunchArgument(
            "orbbec_serial_number",
            default_value="",
            description="Optional Orbbec serial number for selecting a specific device.",
        ),
        DeclareLaunchArgument(
            "orbbec_enable_point_cloud",
            default_value="false",
            description="Enable Orbbec point cloud output.",
        ),
        DeclareLaunchArgument(
            "publish_camera_tf",
            default_value="false",
            description="Publish additional static TF from robot base frame to camera frame.",
        ),
        DeclareLaunchArgument(
            "camera_parent_frame",
            default_value="base_link",
            description="Parent frame for camera static transform.",
        ),
        DeclareLaunchArgument(
            "camera_child_frame",
            default_value="",
            description="Child frame for camera static transform (empty => <orbbec_camera_name>_link).",
        ),
        DeclareLaunchArgument(
            "camera_tf_x",
            default_value="0.0",
            description="Static TF translation X from parent to camera frame (meters).",
        ),
        DeclareLaunchArgument(
            "camera_tf_y",
            default_value="0.0",
            description="Static TF translation Y from parent to camera frame (meters).",
        ),
        DeclareLaunchArgument(
            "camera_tf_z",
            default_value="0.0",
            description="Static TF translation Z from parent to camera frame (meters).",
        ),
        DeclareLaunchArgument(
            "camera_tf_qx",
            default_value="0.0",
            description="Static TF quaternion X.",
        ),
        DeclareLaunchArgument(
            "camera_tf_qy",
            default_value="0.0",
            description="Static TF quaternion Y.",
        ),
        DeclareLaunchArgument(
            "camera_tf_qz",
            default_value="0.0",
            description="Static TF quaternion Z.",
        ),
        DeclareLaunchArgument(
            "camera_tf_qw",
            default_value="1.0",
            description="Static TF quaternion W.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_runtime_actions)])
