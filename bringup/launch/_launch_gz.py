# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""OpaqueFunction handlers for Gazebo Sim: sim launch, runtime nodes, controller spawning."""

import math
import os
import shlex
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
if str(_THIS_DIR) not in sys.path:
    sys.path.insert(0, str(_THIS_DIR))

from _launch_helpers import _is_true, _profile_assets  # noqa: E402
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory  # noqa: E402
from launch.actions import (  # noqa: E402
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit  # noqa: E402
from launch.events import Shutdown  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from launch.substitutions import LaunchConfiguration  # noqa: E402
from launch_ros.actions import Node  # noqa: E402


def _maybe_include_gz_sim(context, *args, **kwargs):
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_gz_sim):
        return []

    try:
        ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError:
        return [LogInfo(msg="ros_gz_sim not found. Install ros-humble-ros-gzharmonic-sim.")]

    world = LaunchConfiguration("world").perform(context).strip()
    if not world:
        return [LogInfo(msg="World path is empty. Pass world:=/absolute/path/to/world.sdf")]

    gz_version = LaunchConfiguration("gz_version").perform(context).strip() or "8"
    gz_headless = _is_true(LaunchConfiguration("gz_headless").perform(context))
    gz_args = f"-s -r {world}" if gz_headless else f"-r {world}"

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": gz_args,
                "gz_version": gz_version,
            }.items(),
        )
    ]


def _maybe_add_gz_sim_runtime_nodes(context, *args, **kwargs):
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_gz_sim):
        return []

    actions = []

    try:
        get_package_share_directory("ros_gz_bridge")
    except PackageNotFoundError:
        actions.append(LogInfo(msg="ros_gz_bridge not found. Install ros-humble-ros-gzharmonic-bridge."))
        return actions

    try:
        get_package_share_directory("ros_gz_sim")
    except PackageNotFoundError:
        actions.append(LogInfo(msg="ros_gz_sim not found. Install ros-humble-ros-gzharmonic-sim."))
        return actions

    world_name = LaunchConfiguration("world_name").perform(context)
    spawn_x = LaunchConfiguration("spawn_x").perform(context)
    spawn_y = LaunchConfiguration("spawn_y").perform(context)
    spawn_z = LaunchConfiguration("spawn_z").perform(context)
    spawn_yaw = LaunchConfiguration("spawn_yaw").perform(context)
    use_hardware = LaunchConfiguration("use_hardware").perform(context)
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf").perform(context)
    robot_profile = LaunchConfiguration("robot_profile").perform(context).strip() or "training_4wd"
    gz_version = LaunchConfiguration("gz_version").perform(context).strip() or "8"
    sim_enable_camera = LaunchConfiguration("sim_enable_camera").perform(context)
    sim_camera_width = LaunchConfiguration("sim_camera_width").perform(context).strip() or "640"
    sim_camera_height = LaunchConfiguration("sim_camera_height").perform(context).strip() or "480"
    sim_camera_update_rate = (
        LaunchConfiguration("sim_camera_update_rate").perform(context).strip() or "30.0"
    )
    sim_lidar_samples = LaunchConfiguration("sim_lidar_samples").perform(context).strip() or "200"
    sim_lidar_update_rate = (
        LaunchConfiguration("sim_lidar_update_rate").perform(context).strip() or "20.0"
    )
    sim_lidar_visualize = LaunchConfiguration("sim_lidar_visualize").perform(context)
    sim_imu_update_rate = LaunchConfiguration("sim_imu_update_rate").perform(context).strip() or "100.0"
    use_sim_time = _is_true(LaunchConfiguration("use_sim_time").perform(context))
    profile_file, controllers_file = _profile_assets(robot_profile)

    if not os.path.exists(profile_file):
        actions.append(LogInfo(msg=f"Robot profile file not found: {profile_file}"))
        return actions
    if not os.path.exists(controllers_file):
        actions.append(LogInfo(msg=f"Controller file not found: {controllers_file}"))
        return actions

    try:
        spawn_yaw_value = float(spawn_yaw)
    except ValueError:
        spawn_yaw_value = 0.0
        actions.append(LogInfo(msg=f"Invalid spawn_yaw '{spawn_yaw}'. Falling back to 0.0."))
    spawn_qz = math.sin(spawn_yaw_value / 2.0)
    spawn_qw = math.cos(spawn_yaw_value / 2.0)

    bridge_arguments = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    ]
    if _is_true(sim_enable_camera):
        bridge_arguments.extend(
            [
                "/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            ]
        )
    if _is_true(use_ground_truth_odom_tf):
        bridge_arguments.extend(
            [
                "/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/ground_truth/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            ]
        )

    actions.append(
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=bridge_arguments,
            remappings=[("/scan", "/scan_raw")],
            output="screen",
        )
    )

    actions.append(
        Node(
            package="studica_vmxpi_ros2",
            executable="topic_adapter_node",
            name="scan_frame_relay",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "enable_scan_relay": True,
                    "scan_input_topic": "/scan_raw",
                    "scan_output_topic": "/scan",
                    "scan_output_frame_id": "laser_scan_frame",
                }
            ],
        )
    )

    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context).strip() or "robot_system_position"

    xacro_file = os.path.join(
        get_package_share_directory("studica_vmxpi_ros2"),
        "description",
        "urdf",
        "robot.urdf.xacro",
    )
    spawn_script = (
        "set -e; "
        "TMP_URDF=/tmp/robot_spawn.urdf; "
        f"xacro {shlex.quote(xacro_file)} "
        "use_gz_sim:=true "
        f"use_hardware:={shlex.quote(use_hardware)} "
        f"gz_version:={shlex.quote(gz_version)} "
        f"sim_enable_camera:={shlex.quote(sim_enable_camera)} "
        f"sim_camera_width:={shlex.quote(sim_camera_width)} "
        f"sim_camera_height:={shlex.quote(sim_camera_height)} "
        f"sim_camera_update_rate:={shlex.quote(sim_camera_update_rate)} "
        f"sim_lidar_samples:={shlex.quote(sim_lidar_samples)} "
        f"sim_lidar_update_rate:={shlex.quote(sim_lidar_update_rate)} "
        f"sim_lidar_visualize:={shlex.quote(sim_lidar_visualize)} "
        f"sim_imu_update_rate:={shlex.quote(sim_imu_update_rate)} "
        f"profile_file:={shlex.quote(profile_file)} "
        f"controllers_file:={shlex.quote(controllers_file)} "
        "> \"$TMP_URDF\"; "
        f"REQ=\"sdf_filename: \\\"$TMP_URDF\\\" name: \\\"{spawn_entity_name}\\\" "
        f"pose {{ position {{ x: {spawn_x} y: {spawn_y} z: {spawn_z} }} "
        f"orientation {{ z: {spawn_qz} w: {spawn_qw} }} }}\"; "
        f"echo \"[spawn-gz-service] world={world_name} yaw={spawn_yaw} entity={spawn_entity_name}\"; "
        f"gz service -s /world/{world_name}/create "
        "--reqtype gz.msgs.EntityFactory "
        "--reptype gz.msgs.Boolean "
        "--timeout 10000 "
        "--req \"$REQ\""
    )

    actions.append(
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=["bash", "-lc", spawn_script],
                    output="screen",
                )
            ],
        )
    )

    return actions


def _maybe_add_gz_sim_controller_spawners(context, *args, **kwargs):
    use_gz_sim = LaunchConfiguration("use_gz_sim").perform(context)
    if not _is_true(use_gz_sim):
        return []

    configured_cm = LaunchConfiguration("controller_manager").perform(context).strip() or "/controller_manager"
    spawn_entity_name = LaunchConfiguration("spawn_entity_name").perform(context).strip() or "robot_system_position"
    drive_controller_name = (
        LaunchConfiguration("drive_controller_name").perform(context).strip()
        or "robot_base_controller"
    )

    # Some Gazebo setups namespace controller_manager under the model name.
    # Try root path first, then common model-scoped fallbacks.
    candidates = []
    for candidate in (
        configured_cm,
        f"/{spawn_entity_name}/controller_manager",
        "/diffdrive_robot/controller_manager",
    ):
        if candidate not in candidates:
            candidates.append(candidate)

    quoted_candidates = " ".join(shlex.quote(candidate) for candidate in candidates)
    quoted_drive_controller_name = shlex.quote(drive_controller_name)
    spawn_script = f"""
set +e
CANDIDATES=({quoted_candidates})
DRIVE_CONTROLLER={quoted_drive_controller_name}

run_spawner() {{
  local name="$1"
  local cm="$2"
  local output
  local rc

  if controller_is_active "$name" "$cm"; then
    return 0
  fi

  output="$(ros2 run controller_manager spawner "$name" --controller-manager "$cm" \
    --controller-manager-timeout 20 2>&1)"
  rc=$?

  if [ "$rc" -eq 0 ]; then
    echo "$output"
    return 0
  fi

  if controller_is_active "$name" "$cm"; then
    return 0
  fi

  # Treat as success only if the controller is already active/running.
  # "already loaded" alone is not enough, because configuration may still fail
  # when hardware interfaces are not ready yet.
  if echo "$output" | grep -Eq \
    "Controller already active|already in state 'active'|already running|cannot be configured from 'active' state"; then
    return 0
  fi

  echo "$output"
  return "$rc"
}}

controller_is_active() {{
  local name="$1"
  local cm="$2"
  local output

  output="$(ros2 control list_controllers --controller-manager "$cm" 2>/dev/null | \
    sed -E 's/\x1B\\[[0-9;]*[mK]//g')"
  echo "$output" | grep -Eq "^${{name}}[[:space:]].*[[:space:]]active([[:space:]]|$)"
}}

for cm in "${{CANDIDATES[@]}}"; do
  echo "[spawner-fallback] trying ${{cm}}"
  done_joint=0
  done_imu=0
  done_drive=0

  for attempt in $(seq 1 20); do
    if [ "$done_joint" -eq 0 ]; then
      run_spawner joint_state_broadcaster "$cm" && done_joint=1
    fi
    if [ "$done_imu" -eq 0 ]; then
      run_spawner imu_sensor_broadcaster "$cm" && done_imu=1
    fi
    if [ "$done_drive" -eq 0 ]; then
      run_spawner "$DRIVE_CONTROLLER" "$cm" && done_drive=1
    fi

    if [ "$done_joint" -eq 1 ] && [ "$done_imu" -eq 1 ] && [ "$done_drive" -eq 1 ]; then
      echo "[spawner-fallback] controllers active on ${{cm}}"
      exit 0
    fi

    echo "[spawner-fallback] retry ${{attempt}}/20 for ${{cm}}"
    sleep 1
  done
done

echo "[spawner-fallback] failed. tried: ${{CANDIDATES[*]}}" >&2
exit 1
"""

    spawner_process = ExecuteProcess(
        cmd=["bash", "-lc", spawn_script],
        output="screen",
    )

    return [
        spawner_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawner_process,
                on_exit=_shutdown_if_controller_spawner_failed,
            )
        ),
    ]


def _shutdown_if_controller_spawner_failed(event, context):
    del context
    return_code = getattr(event, "returncode", None)
    if return_code == 0:
        return []

    code_text = str(return_code) if return_code is not None else "unknown"
    reason = f"Controller spawning failed (exit code: {code_text})"
    return [
        LogInfo(msg=f"{reason}. Shutting down bringup."),
        EmitEvent(event=Shutdown(reason=reason)),
    ]
