# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""OpaqueFunction handlers for hardware sensors: gamepad, LiDAR, and camera."""

import os
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _launch_helpers import _is_true, _profile_camera_tf_base_link  # noqa: E402
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory  # noqa: E402
from launch.actions import IncludeLaunchDescription, LogInfo  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402


def _maybe_include_gamepad(context, *args, **kwargs):
    use_joystick = LaunchConfiguration("use_joystick").perform(context)
    if not _is_true(use_joystick):
        return []

    use_sim_time_value = LaunchConfiguration("use_sim_time").perform(context)
    if _is_true(LaunchConfiguration("use_gz_sim").perform(context)):
        # Keep stamped joystick commands valid even if /clock is unavailable.
        use_sim_time_value = "false"

    try:
        studica_pkg = get_package_share_directory("studica_ros2_control")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_ros2_control not found; skipping joystick launch.")]

    joystick_cmd_vel_topic = LaunchConfiguration("joystick_cmd_vel_topic").perform(context).strip()
    if not joystick_cmd_vel_topic:
        joystick_cmd_vel_topic = LaunchConfiguration("drive_cmd_topic").perform(context).strip()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "gamepad_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time_value,
                "cmd_vel_topic": joystick_cmd_vel_topic,
                "publish_stamped": LaunchConfiguration("joystick_publish_stamped").perform(context),
            }.items(),
        )
    ]


def _maybe_include_lidar(context, *args, **kwargs):
    use_lidar = LaunchConfiguration("use_lidar").perform(context)
    use_hardware = LaunchConfiguration("use_hardware").perform(context)
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_lidar):
        return []
    if not _is_true(use_hardware):
        return [LogInfo(msg="use_lidar enabled but use_hardware is false; skipping LiDAR launch.")]
    if _is_true(use_gz_sim):
        return [LogInfo(msg="use_lidar enabled but use_gz_sim is true; skipping real LiDAR launch.")]

    try:
        studica_pkg = get_package_share_directory("studica_vmxpi_ros2")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_vmxpi_ros2 not found; skipping LiDAR launch.")]
    try:
        get_package_share_directory("ydlidar_ros2_driver")
    except PackageNotFoundError:
        return [LogInfo(msg="ydlidar_ros2_driver not found; skipping LiDAR launch.")]

    ydlidar_params_file = LaunchConfiguration("ydlidar_params_file").perform(context).strip()
    lidar_type = LaunchConfiguration("lidar_type").perform(context).strip()

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "lidar_hw.launch.py")
            ),
            launch_arguments={
                "lidar_type": lidar_type,
                "ydlidar_params_file": ydlidar_params_file,
                "lidar_parent_frame": LaunchConfiguration("lidar_parent_frame").perform(context),
                "lidar_child_frame": LaunchConfiguration("lidar_child_frame").perform(context),
                "lidar_tf_x": LaunchConfiguration("lidar_tf_x").perform(context),
                "lidar_tf_y": LaunchConfiguration("lidar_tf_y").perform(context),
                "lidar_tf_z": LaunchConfiguration("lidar_tf_z").perform(context),
                "lidar_tf_qx": LaunchConfiguration("lidar_tf_qx").perform(context),
                "lidar_tf_qy": LaunchConfiguration("lidar_tf_qy").perform(context),
                "lidar_tf_qz": LaunchConfiguration("lidar_tf_qz").perform(context),
                "lidar_tf_qw": LaunchConfiguration("lidar_tf_qw").perform(context),
            }.items(),
        )
    ]


def _maybe_include_camera(context, *args, **kwargs):
    use_camera = LaunchConfiguration("use_camera").perform(context)
    use_hardware = LaunchConfiguration("use_hardware").perform(context)
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_camera):
        return []
    if not _is_true(use_hardware):
        return [LogInfo(msg="use_camera enabled but use_hardware is false; skipping camera launch.")]
    if _is_true(use_gz_sim):
        return [LogInfo(msg="use_camera enabled but use_gz_sim is true; skipping real camera launch.")]

    try:
        studica_pkg = get_package_share_directory("studica_vmxpi_ros2")
    except PackageNotFoundError:
        return [LogInfo(msg="studica_vmxpi_ros2 not found; skipping camera launch.")]
    try:
        get_package_share_directory("orbbec_camera")
    except PackageNotFoundError:
        return [LogInfo(msg="orbbec_camera not found; skipping camera launch.")]

    camera_name = LaunchConfiguration("orbbec_camera_name").perform(context).strip() or "camera"
    camera_parent_frame = LaunchConfiguration("camera_parent_frame").perform(context).strip() or "base_link"
    camera_child_frame = LaunchConfiguration("camera_child_frame").perform(context).strip()
    if not camera_child_frame:
        camera_child_frame = f"{camera_name}_link"

    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip() or "training_4wd"
    camera_tf = None
    camera_tf_error = None
    try:
        camera_tf = _profile_camera_tf_base_link(robot_profile)
    except Exception as exc:  # pylint: disable=broad-except
        camera_tf_error = str(exc)

    launch_arguments = {
        "orbbec_launch_file": LaunchConfiguration("orbbec_launch_file").perform(context),
        "orbbec_camera_name": camera_name,
        "orbbec_serial_number": LaunchConfiguration("orbbec_serial_number").perform(context),
        "orbbec_enable_point_cloud": LaunchConfiguration("orbbec_enable_point_cloud").perform(context),
        "orbbec_enable_color": LaunchConfiguration("orbbec_enable_color").perform(context),
        "orbbec_enable_depth": LaunchConfiguration("orbbec_enable_depth").perform(context),
        "orbbec_enable_ir": LaunchConfiguration("orbbec_enable_ir").perform(context),
        "orbbec_color_width": LaunchConfiguration("orbbec_color_width").perform(context),
        "orbbec_color_height": LaunchConfiguration("orbbec_color_height").perform(context),
        "orbbec_color_fps": LaunchConfiguration("orbbec_color_fps").perform(context),
        "orbbec_depth_width": LaunchConfiguration("orbbec_depth_width").perform(context),
        "orbbec_depth_height": LaunchConfiguration("orbbec_depth_height").perform(context),
        "orbbec_depth_fps": LaunchConfiguration("orbbec_depth_fps").perform(context),
        # Align gemini_e static TF with robot URDF camera frame by default.
        "orbbec_base_frame_id": camera_parent_frame,
        "orbbec_camera_link_frame_id": camera_child_frame,
        "publish_camera_tf": LaunchConfiguration("publish_camera_tf").perform(context),
        "camera_parent_frame": camera_parent_frame,
        "camera_child_frame": camera_child_frame,
        "camera_tf_x": LaunchConfiguration("camera_tf_x").perform(context),
        "camera_tf_y": LaunchConfiguration("camera_tf_y").perform(context),
        "camera_tf_z": LaunchConfiguration("camera_tf_z").perform(context),
        "camera_tf_qx": LaunchConfiguration("camera_tf_qx").perform(context),
        "camera_tf_qy": LaunchConfiguration("camera_tf_qy").perform(context),
        "camera_tf_qz": LaunchConfiguration("camera_tf_qz").perform(context),
        "camera_tf_qw": LaunchConfiguration("camera_tf_qw").perform(context),
    }
    if camera_tf is not None:
        launch_arguments.update(
            {
                "orbbec_base_to_camera_x": camera_tf["x"],
                "orbbec_base_to_camera_y": camera_tf["y"],
                "orbbec_base_to_camera_z": camera_tf["z"],
                "orbbec_base_to_camera_roll": camera_tf["roll"],
                "orbbec_base_to_camera_pitch": camera_tf["pitch"],
                "orbbec_base_to_camera_yaw": camera_tf["yaw"],
            }
        )

    actions = []
    if camera_tf is not None:
        actions.append(
            LogInfo(
                msg=[
                    "Orbbec base->camera TF from profile ",
                    robot_profile,
                    ": xyz=(",
                    camera_tf["x"],
                    ",",
                    camera_tf["y"],
                    ",",
                    camera_tf["z"],
                    "), rpy=(",
                    camera_tf["roll"],
                    ",",
                    camera_tf["pitch"],
                    ",",
                    camera_tf["yaw"],
                    ")",
                ]
            )
        )
    elif camera_tf_error:
        actions.append(
            LogInfo(
                msg=[
                    "Failed to compute profile camera TF for Orbbec alignment: ",
                    camera_tf_error,
                    " (using Orbbec launch defaults).",
                ]
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(studica_pkg, "launch", "camera_hw.launch.py")
            ),
            launch_arguments=launch_arguments.items(),
        )
    )
    return actions
