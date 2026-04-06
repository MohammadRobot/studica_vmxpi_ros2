# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Primary Gazebo Sim launch for robot model, control, bridges, and optional tools."""

import math
import os
import shlex
import yaml

from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_TRUE_VALUES_EXPR = "['true','1','yes','on']"
_FALSE_VALUES_EXPR = "['false','0','no','off']"


def _is_true(value: str) -> bool:
    return value.lower() in ("true", "1", "yes", "on")


def _expr_is_true(value):
    """Build a PythonExpression fragment that checks truthy launch values."""
    return ["('", value, f"').lower() in {_TRUE_VALUES_EXPR}"]


def _expr_is_false(value):
    """Build a PythonExpression fragment that checks falsy launch values."""
    return ["('", value, f"').lower() in {_FALSE_VALUES_EXPR}"]


def _declare_arg(name: str, default_value, description: str = ""):
    """Create a launch argument with optional description text."""
    if description:
        return DeclareLaunchArgument(
            name,
            default_value=default_value,
            description=description,
        )
    return DeclareLaunchArgument(name, default_value=default_value)


def _append_env_path(var_name: str, entry: str):
    """Append to an environment path variable while preserving prior contents."""
    if var_name in os.environ:
        os.environ[var_name] = f"{os.environ[var_name]}:{entry}"
    else:
        os.environ[var_name] = entry


def _sanitize_ld_library_path_for_rviz() -> str:
    """
    Drop environment-specific runtime entries that can break host ROS binaries.

    Removes:
    - Snap runtime entries (RViz issues on some desktops)
    - Conda/Miniconda paths (common libstdc++ ABI mismatch with ROS Humble)
    """
    ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    if not ld_library_path:
        return ""

    conda_roots = []
    conda_prefix = os.environ.get("CONDA_PREFIX", "").strip()
    if conda_prefix:
        conda_roots.append(conda_prefix)
    conda_exe = os.environ.get("CONDA_EXE", "").strip()
    if conda_exe:
        conda_roots.append(os.path.dirname(os.path.dirname(conda_exe)))
    conda_roots = [os.path.realpath(path) for path in conda_roots if path]

    def _is_conda_entry(path: str) -> bool:
        real_path = os.path.realpath(path)
        if any(real_path == root or real_path.startswith(root + os.sep) for root in conda_roots):
            return True
        lowered = real_path.lower()
        return any(
            token in lowered for token in ("/miniconda", "/anaconda", "/mambaforge", "/micromamba")
        )

    filtered_entries = []
    for entry in ld_library_path.split(":"):
        if not entry:
            continue
        if "/snap/" in entry:
            continue
        if _is_conda_entry(entry):
            continue
        filtered_entries.append(entry)
    return ":".join(filtered_entries)


def _profile_assets(profile_name: str):
    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    profile_dir = os.path.join(pkg_share, "config", "profiles", profile_name)
    profile_file = os.path.join(profile_dir, "robot_profile.yaml")
    controllers_file = os.path.join(profile_dir, "robot_controllers.yaml")
    return profile_file, controllers_file


def _profile_camera_tf_base_link(profile_name: str):
    """
    Resolve base_link -> camera_link transform using the same defaults as robot URDF.

    URDF chain:
      base_link -> chassis_link: z = wheel_radius - wheel_z_offset
      chassis_link -> camera_link: (cam_pos_x, cam_pos_y, cam_pos_z, cam_rpy)
    """
    profile_file, _ = _profile_assets(profile_name)
    with open(profile_file, "r", encoding="utf-8") as stream:
        profile_data = yaml.safe_load(stream) or {}

    xacro_cfg = profile_data.get("xacro")
    if not isinstance(xacro_cfg, dict):
        raise ValueError(f"Missing xacro mapping in profile: {profile_file}")

    # Keep defaults synchronized with description/robot/urdf/robot_description.urdf.xacro.
    base_length = float(xacro_cfg.get("base_length", 0.4))
    base_height = float(xacro_cfg.get("base_height", 0.1))
    wheel_radius = float(xacro_cfg.get("wheel_radius", 0.05))
    wheel_z_offset = float(xacro_cfg.get("wheel_z_offset", -0.027))

    cam_pos_x = float(xacro_cfg.get("cam_pos_x", base_length / 2.0))
    cam_pos_y = float(xacro_cfg.get("cam_pos_y", 0.0))
    cam_pos_z = float(xacro_cfg.get("cam_pos_z", base_height / 2.0))
    cam_roll = float(xacro_cfg.get("cam_roll", 0.0))
    cam_pitch = float(xacro_cfg.get("cam_pitch", 0.0))
    cam_yaw = float(xacro_cfg.get("cam_yaw", 0.0))

    base_to_chassis_z = wheel_radius - wheel_z_offset
    return {
        "x": f"{cam_pos_x:.6f}",
        "y": f"{cam_pos_y:.6f}",
        "z": f"{(base_to_chassis_z + cam_pos_z):.6f}",
        "roll": f"{cam_roll:.6f}",
        "pitch": f"{cam_pitch:.6f}",
        "yaw": f"{cam_yaw:.6f}",
    }


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


def generate_launch_description():
    declared_arguments = [
        _declare_arg(
            "gui",
            "false",
            "Start RViz2 automatically with this launch file.",
        ),
        _declare_arg(
            "rviz_start_delay",
            "10.0",
            "Delay before starting RViz2 (seconds). Lower for faster startup.",
        ),
        _declare_arg(
            "rviz_config_file",
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/robot/rviz", "robot.rviz"]
            ),
            "Absolute path to RViz config file.",
        ),
        _declare_arg(
            "use_hardware",
            "false",
            "Use Titan hardware instead of mock system.",
        ),
        _declare_arg("use_gz_sim", "true", "Start Gazebo Sim."),
        _declare_arg(
            "use_sim_time",
            LaunchConfiguration("use_gz_sim"),
            "Use simulation time (defaults to use_gz_sim).",
        ),
        _declare_arg(
            "use_ground_truth_odom_tf",
            "true",
            "In gz_sim, source /odom and /tf from Gazebo ground-truth odometry.",
        ),
        _declare_arg(
            "robot_profile",
            "training_4wd",
            "Robot profile under config/profiles (for example: training_4wd, training_2wd).",
        ),
        _declare_arg(
            "world",
            PathJoinSubstitution(
                [FindPackageShare("studica_vmxpi_ros2"), "description/gz/worlds", "diff_drive_world.sdf"]
            ),
            "Absolute path to Gazebo Sim world file.",
        ),
        _declare_arg("world_name", "default", "World name to spawn into."),
        _declare_arg(
            "gz_version",
            "8",
            "Gazebo Sim major version passed to ros_gz_sim (8 for Harmonic).",
        ),
        _declare_arg(
            "gz_headless",
            "false",
            "Run Gazebo Sim server only (no Gazebo GUI client).",
        ),
        _declare_arg(
            "sim_enable_camera",
            "true",
            "Enable simulated RGB + depth camera sensors in gz_sim.",
        ),
        _declare_arg(
            "sim_camera_width",
            "640",
            "Sim camera image width in pixels.",
        ),
        _declare_arg(
            "sim_camera_height",
            "480",
            "Sim camera image height in pixels.",
        ),
        _declare_arg(
            "sim_camera_update_rate",
            "30.0",
            "Sim camera update rate (Hz) for color and depth streams.",
        ),
        _declare_arg(
            "sim_lidar_samples",
            "200",
            "Sim lidar horizontal sample count.",
        ),
        _declare_arg(
            "sim_lidar_update_rate",
            "20.0",
            "Sim lidar update rate (Hz).",
        ),
        _declare_arg(
            "sim_lidar_visualize",
            "true",
            "Enable Gazebo visualization for lidar rays.",
        ),
        _declare_arg(
            "sim_imu_update_rate",
            "100.0",
            "Sim IMU update rate (Hz).",
        ),
        _declare_arg("imu_gz_topic", "/imu", "Gazebo IMU topic to bridge into ROS /imu."),
        _declare_arg("spawn_x", "0.0", "Initial robot spawn x (meters)."),
        _declare_arg("spawn_y", "0.0", "Initial robot spawn y (meters)."),
        _declare_arg("spawn_z", "0.10", "Initial robot spawn z (meters)."),
        _declare_arg("spawn_yaw", "0.0", "Initial robot spawn yaw (radians)."),
        _declare_arg(
            "spawn_entity_name",
            "robot_system_position",
            "Entity name used when spawning the robot into Gazebo Sim.",
        ),
        _declare_arg(
            "controller_manager",
            "/controller_manager",
            "Preferred controller_manager service root for controller spawners.",
        ),
        _declare_arg(
            "use_joystick",
            "false",
            "Launch joystick teleop from studica_ros2_control.",
        ),
        _declare_arg(
            "use_lidar",
            LaunchConfiguration("use_hardware"),
            "Launch YDLIDAR in real hardware mode (defaults to use_hardware).",
        ),
        _declare_arg(
            "ydlidar_params_file",
            "",
            "Optional YDLIDAR params YAML. When set, this overrides lidar_type.",
        ),
        _declare_arg(
            "lidar_type",
            "tmini",
            "YDLIDAR model preset (example: tmini, x4, g4, gs2, sdm15). Used when ydlidar_params_file is empty.",
        ),
        _declare_arg(
            "lidar_parent_frame",
            "base_link",
            "Parent frame for LiDAR static transform.",
        ),
        _declare_arg(
            "lidar_child_frame",
            "laser_frame",
            "Child frame for LiDAR static transform.",
        ),
        _declare_arg("lidar_tf_x", "0.0", "LiDAR static TF translation X (meters)."),
        _declare_arg("lidar_tf_y", "0.0", "LiDAR static TF translation Y (meters)."),
        _declare_arg("lidar_tf_z", "0.02", "LiDAR static TF translation Z (meters)."),
        _declare_arg("lidar_tf_qx", "0.0", "LiDAR static TF quaternion X."),
        _declare_arg("lidar_tf_qy", "0.0", "LiDAR static TF quaternion Y."),
        _declare_arg(
            "lidar_tf_qz",
            "0.0",
            "LiDAR static TF quaternion Z.",
        ),
        _declare_arg("lidar_tf_qw", "1.0", "LiDAR static TF quaternion W."),
        _declare_arg(
            "use_camera",
            LaunchConfiguration("use_hardware"),
            "Launch Orbbec camera in real hardware mode (defaults to use_hardware).",
        ),
        _declare_arg(
            "orbbec_launch_file",
            "gemini_e.launch.py",
            "Orbbec launch file in orbbec_camera/launch.",
        ),
        _declare_arg(
            "orbbec_camera_name",
            "camera",
            "Orbbec camera_name launch argument (also sets namespace).",
        ),
        _declare_arg(
            "orbbec_serial_number",
            "",
            "Optional Orbbec serial number for selecting a specific device.",
        ),
        _declare_arg(
            "orbbec_enable_point_cloud",
            "false",
            "Enable Orbbec point cloud output.",
        ),
        _declare_arg(
            "orbbec_enable_color",
            "",
            "Optional override for Orbbec launch arg enable_color.",
        ),
        _declare_arg(
            "orbbec_enable_depth",
            "",
            "Optional override for Orbbec launch arg enable_depth.",
        ),
        _declare_arg(
            "orbbec_enable_ir",
            "",
            "Optional override for Orbbec launch arg enable_ir.",
        ),
        _declare_arg(
            "orbbec_color_width",
            "",
            "Optional override for Orbbec launch arg color_width.",
        ),
        _declare_arg(
            "orbbec_color_height",
            "",
            "Optional override for Orbbec launch arg color_height.",
        ),
        _declare_arg(
            "orbbec_color_fps",
            "",
            "Optional override for Orbbec launch arg color_fps.",
        ),
        _declare_arg(
            "orbbec_depth_width",
            "",
            "Optional override for Orbbec launch arg depth_width.",
        ),
        _declare_arg(
            "orbbec_depth_height",
            "",
            "Optional override for Orbbec launch arg depth_height.",
        ),
        _declare_arg(
            "orbbec_depth_fps",
            "",
            "Optional override for Orbbec launch arg depth_fps.",
        ),
        _declare_arg(
            "publish_camera_tf",
            "false",
            "Publish additional static TF from base frame to camera frame.",
        ),
        _declare_arg(
            "camera_parent_frame",
            "base_link",
            "Parent frame for camera static transform.",
        ),
        _declare_arg(
            "camera_child_frame",
            "",
            "Child frame for camera static transform (empty => <orbbec_camera_name>_link).",
        ),
        _declare_arg("camera_tf_x", "0.0", "Camera static TF translation X (meters)."),
        _declare_arg("camera_tf_y", "0.0", "Camera static TF translation Y (meters)."),
        _declare_arg("camera_tf_z", "0.0", "Camera static TF translation Z (meters)."),
        _declare_arg("camera_tf_qx", "0.0", "Camera static TF quaternion X."),
        _declare_arg("camera_tf_qy", "0.0", "Camera static TF quaternion Y."),
        _declare_arg("camera_tf_qz", "0.0", "Camera static TF quaternion Z."),
        _declare_arg("camera_tf_qw", "1.0", "Camera static TF quaternion W."),
        _declare_arg(
            "joystick_cmd_vel_topic",
            "",
            "Joystick command velocity output topic (empty = auto from drive profile).",
        ),
        _declare_arg(
            "joystick_publish_stamped",
            "true",
            "Publish TwistStamped joystick commands.",
        ),
        _declare_arg(
            "drive_controller_name",
            "robot_base_controller",
            "Primary drive controller name loaded by controller_manager.",
        ),
        _declare_arg(
            "drive_controller_type",
            "diff_drive_controller/DiffDriveController",
            "Primary drive controller plugin type.",
        ),
        _declare_arg(
            "drive_cmd_topic",
            "/robot_base_controller/cmd_vel",
            "Primary drive command topic.",
        ),
        _declare_arg(
            "drive_odom_topic",
            "/robot_base_controller/odom",
            "Primary drive odometry topic.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    use_hardware = LaunchConfiguration("use_hardware")
    use_gz_sim = LaunchConfiguration("use_gz_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    use_ground_truth_odom_tf = LaunchConfiguration("use_ground_truth_odom_tf")
    robot_profile = LaunchConfiguration("robot_profile")
    rviz_start_delay = LaunchConfiguration("rviz_start_delay")
    drive_controller_name = LaunchConfiguration("drive_controller_name")
    drive_controller_type = LaunchConfiguration("drive_controller_type")
    drive_cmd_topic = LaunchConfiguration("drive_cmd_topic")
    drive_odom_topic = LaunchConfiguration("drive_odom_topic")

    pkg_studica_vmxpi_ros2 = get_package_share_directory("studica_vmxpi_ros2")
    install_dir = get_package_prefix("studica_vmxpi_ros2")
    gz_models_path = os.path.join(pkg_studica_vmxpi_ros2, "description", "models")

    _append_env_path("GZ_SIM_RESOURCE_PATH", install_dir + "/share")
    _append_env_path("GZ_SIM_RESOURCE_PATH", gz_models_path)
    _append_env_path("GZ_SIM_SYSTEM_PLUGIN_PATH", install_dir + "/lib")

    print("GZ SIM RESOURCE PATH==" + str(os.environ["GZ_SIM_RESOURCE_PATH"]))
    print("GZ SIM SYSTEM PLUGINS PATH==" + str(os.environ["GZ_SIM_SYSTEM_PLUGIN_PATH"]))

    profile_file = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "config", "profiles", robot_profile, "robot_profile.yaml"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("studica_vmxpi_ros2"), "config", "profiles", robot_profile, "robot_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("studica_vmxpi_ros2"), "description/urdf", "robot.urdf.xacro"]),
            " ",
            "use_gz_sim:=", use_gz_sim,
            " ",
            "use_hardware:=", use_hardware,
            " ",
            "gz_version:=", LaunchConfiguration("gz_version"),
            " ",
            "sim_enable_camera:=", LaunchConfiguration("sim_enable_camera"),
            " ",
            "sim_camera_width:=", LaunchConfiguration("sim_camera_width"),
            " ",
            "sim_camera_height:=", LaunchConfiguration("sim_camera_height"),
            " ",
            "sim_camera_update_rate:=", LaunchConfiguration("sim_camera_update_rate"),
            " ",
            "sim_lidar_samples:=", LaunchConfiguration("sim_lidar_samples"),
            " ",
            "sim_lidar_update_rate:=", LaunchConfiguration("sim_lidar_update_rate"),
            " ",
            "sim_lidar_visualize:=", LaunchConfiguration("sim_lidar_visualize"),
            " ",
            "sim_imu_update_rate:=", LaunchConfiguration("sim_imu_update_rate"),
            " ",
            "profile_file:=", profile_file,
            " ",
            "controllers_file:=", robot_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": use_sim_time_param,
    }

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    rviz_env = {}
    sanitized_ld_library_path = _sanitize_ld_library_path_for_rviz()
    if sanitized_ld_library_path:
        rviz_env["LD_LIBRARY_PATH"] = sanitized_ld_library_path

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gz_sim),
    )

    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(
            PythonExpression(
                _expr_is_true(use_hardware)
                + [" and "]
                + _expr_is_false(use_gz_sim)
            )
        ),
    )

    imu_alias_relay_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="imu_topic_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_imu_relay": True,
                "imu_input_topic": "/imu_sensor_broadcaster/imu",
                "imu_output_topic": "/imu",
                "imu_use_odom_fallback": ParameterValue(use_gz_sim, value_type=bool),
                "imu_fallback_odom_topic": drive_odom_topic,
                "imu_fallback_frame_id": "imu_link",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["("]
                + _expr_is_true(use_gz_sim)
                + [") or ("]
                + _expr_is_true(use_hardware)
                + [" and "]
                + _expr_is_false(use_gz_sim)
                + [")"]
            )
        ),
    )

    control_api_bridge_node_gt = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="nav2_topic_bridge",
        output="screen",
        parameters=[
                {
                    "use_sim_time": use_sim_time_param,
                    "enable_nav2_bridge": True,
                    "input_cmd_vel_topic": "/cmd_vel",
                    "output_cmd_vel_topic": drive_cmd_topic,
                    "input_odom_topic": "/ground_truth/odom",
                    "output_odom_topic": "/odom",
                    "cmd_vel_frame_id": "base_link",
                }
            ],
        condition=IfCondition(
            PythonExpression(
                _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_true(use_ground_truth_odom_tf)
            )
        ),
    )

    control_api_bridge_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="nav2_topic_bridge_fallback",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_nav2_bridge": True,
                "input_cmd_vel_topic": "/cmd_vel",
                "output_cmd_vel_topic": drive_cmd_topic,
                "input_odom_topic": drive_odom_topic,
                "output_odom_topic": "/odom",
                "cmd_vel_frame_id": "base_link",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                ["("]
                + _expr_is_true(use_hardware)
                + [") or ("]
                + _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_false(use_ground_truth_odom_tf)
                + [")"]
            )
        ),
    )

    drive_tf_relay_node_gt = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="ground_truth_tf_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_tf_relay": True,
                "tf_input_topic": "/ground_truth/tf",
                "tf_output_topic": "/tf",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_true(use_ground_truth_odom_tf)
            )
        ),
    )

    drive_tf_relay_node = Node(
        package="studica_vmxpi_ros2",
        executable="topic_adapter_node",
        name="drive_tf_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "enable_tf_relay": True,
                "tf_input_topic": PythonExpression(
                    ["'/' + '", drive_controller_name, "' + '/tf_odometry'"]
                ),
                "tf_output_topic": "/tf",
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "(('",
                    drive_controller_type,
                    "').strip() == 'mecanum_drive_controller/MecanumDriveController') and not (",
                ]
                + _expr_is_true(use_gz_sim)
                + [" and "]
                + _expr_is_true(use_ground_truth_odom_tf)
                + [")"]
            )
        ),
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[drive_controller_name, "--controller-manager", "/controller_manager"],
        condition=UnlessCondition(use_gz_sim),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[{"use_sim_time": use_sim_time_param}],
        arguments=["-d", rviz_config_file],
        additional_env=rviz_env,
        condition=IfCondition(gui),
    )
    rviz_node_delayed = TimerAction(
        # In Gazebo Sim, wheel/odom TF may appear only after ros2_control + broadcasters are active.
        # Delay RViz to avoid startup TF errors (left_wheel/right_wheel/caster -> odom/base_link).
        period=rviz_start_delay,
        actions=[rviz_node],
    )

    base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_footprint",
            "--child-frame-id",
            "base_link",
        ],
    )

    # Hardware-only control path (no simulation).
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="",
        parameters=[robot_description, robot_controllers, {"use_sim_time": use_sim_time_param}],
        output="screen",
        condition=UnlessCondition(use_gz_sim),
    )

    nodes = [
        SetEnvironmentVariable("LD_LIBRARY_PATH", sanitized_ld_library_path),
        LogInfo(msg=["Robot profile: ", robot_profile]),
        OpaqueFunction(function=_maybe_include_gz_sim),
        OpaqueFunction(function=_maybe_add_gz_sim_runtime_nodes),
        OpaqueFunction(function=_maybe_add_gz_sim_controller_spawners),
        control_node,
        node_robot_state_publisher,
        base_footprint_tf,
        joint_state_broadcaster_spawner,
        imu_sensor_broadcaster_spawner,
        imu_alias_relay_node,
        control_api_bridge_node_gt,
        control_api_bridge_node,
        drive_tf_relay_node_gt,
        drive_tf_relay_node,
        robot_controller_spawner,
        rviz_node_delayed,
        OpaqueFunction(function=_maybe_include_gamepad),
        OpaqueFunction(function=_maybe_include_lidar),
        OpaqueFunction(function=_maybe_include_camera),
    ]

    return LaunchDescription(declared_arguments + nodes)
