# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Hardware YDLIDAR launch with optional static TF configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _resolve_params_file(context):
    params_file_arg = LaunchConfiguration("ydlidar_params_file").perform(context).strip()
    if params_file_arg:
        return params_file_arg

    lidar_type = LaunchConfiguration("lidar_type").perform(context).strip().lower()
    if not lidar_type:
        lidar_type = "tmini"

    lidar_to_yaml = {
        "default": "ydlidar.yaml",
        "ydlidar": "ydlidar.yaml",
        "g1": "G1.yaml",
        "g2": "G2.yaml",
        "g4": "G4.yaml",
        "g6": "G6.yaml",
        "x2": "X2.yaml",
        "x2l": "X2.yaml",
        "x3": "X3.yaml",
        "x4": "X4.yaml",
        "x4-pro": "X4-Pro.yaml",
        "x4_pro": "X4-Pro.yaml",
        "tg": "TG.yaml",
        "tg15": "TG.yaml",
        "tg30": "TG.yaml",
        "tg50": "TG.yaml",
        "tmini": "Tmini.yaml",
        "timini": "Tmini.yaml",
        "tmini-pro": "Tmini.yaml",
        "tmini_pro": "Tmini.yaml",
        "tmini-plus": "Tmini.yaml",
        "tmini_plus": "Tmini.yaml",
        "tmini-plus-sh": "Tmini-Plus-SH.yaml",
        "tmini_plus_sh": "Tmini-Plus-SH.yaml",
        "tea": "TEA.yaml",
        "gs2": "GS2.yaml",
        "gs5": "GS5.yaml",
        "sdm15": "sdm15.yaml",
    }
    if lidar_type not in lidar_to_yaml:
        supported = ", ".join(sorted(lidar_to_yaml.keys()))
        raise RuntimeError(
            f"Unsupported lidar_type '{lidar_type}'. "
            f"Use one of: {supported}, or pass ydlidar_params_file:=/abs/path/to/file.yaml"
        )

    return os.path.join(
        get_package_share_directory("ydlidar_ros2_driver"),
        "params",
        lidar_to_yaml[lidar_type],
    )


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    ydlidar_params_file = _resolve_params_file(context)
    lidar_parent_frame = LaunchConfiguration("lidar_parent_frame")
    lidar_child_frame = LaunchConfiguration("lidar_child_frame")
    lidar_tf_x = LaunchConfiguration("lidar_tf_x")
    lidar_tf_y = LaunchConfiguration("lidar_tf_y")
    lidar_tf_z = LaunchConfiguration("lidar_tf_z")
    lidar_tf_qx = LaunchConfiguration("lidar_tf_qx")
    lidar_tf_qy = LaunchConfiguration("lidar_tf_qy")
    lidar_tf_qz = LaunchConfiguration("lidar_tf_qz")
    lidar_tf_qw = LaunchConfiguration("lidar_tf_qw")

    driver_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[ydlidar_params_file],
        namespace="/",
    )

    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_laser",
        arguments=[
            lidar_tf_x,
            lidar_tf_y,
            lidar_tf_z,
            lidar_tf_qx,
            lidar_tf_qy,
            lidar_tf_qz,
            lidar_tf_qw,
            lidar_parent_frame,
            lidar_child_frame,
        ],
    )

    return [
        LogInfo(msg=[f"Using YDLidar params: {ydlidar_params_file}"]),
        LogInfo(
            msg=[
                "LiDAR TF ",
                lidar_parent_frame,
                " -> ",
                lidar_child_frame,
                " quat=(",
                lidar_tf_qx,
                ",",
                lidar_tf_qy,
                ",",
                lidar_tf_qz,
                ",",
                lidar_tf_qw,
                ")",
            ]
        ),
        driver_node,
        lidar_tf_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "lidar_type",
            default_value="tmini",
            description=(
                "YDLIDAR model preset (example: tmini, x4, g4, gs2, sdm15). "
                "Used when ydlidar_params_file is empty."
            ),
        ),
        DeclareLaunchArgument(
            "ydlidar_params_file",
            default_value="",
            description="Optional path to YDLidar ROS2 params YAML. Overrides lidar_type when set.",
        ),
        DeclareLaunchArgument(
            "lidar_parent_frame",
            default_value="base_link",
            description="Parent frame for LiDAR static transform.",
        ),
        DeclareLaunchArgument(
            "lidar_child_frame",
            default_value="laser_frame",
            description="Child frame for LiDAR static transform.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_x",
            default_value="0.0",
            description="Static TF translation X from parent to LiDAR frame (meters).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_y",
            default_value="0.0",
            description="Static TF translation Y from parent to LiDAR frame (meters).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_z",
            default_value="0.02",
            description="Static TF translation Z from parent to LiDAR frame (meters).",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qx",
            default_value="0.0",
            description="Static TF quaternion X.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qy",
            default_value="0.0",
            description="Static TF quaternion Y.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qz",
            default_value="0.0",
            description="Static TF quaternion Z.",
        ),
        DeclareLaunchArgument(
            "lidar_tf_qw",
            default_value="1.0",
            description="Static TF quaternion W.",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
