# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared utilities and profile helpers for studica_vmxpi_ros2 launch files."""

import os
import re
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument


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


def _profile_assets(profile_name: str, control_rate_hz=None, enable_odom_tf=None):
    """Materialize controller parameters with runtime control/TF overrides."""
    pkg_share = get_package_share_directory("studica_vmxpi_ros2")
    profile_dir = os.path.join(pkg_share, "config", "profiles", profile_name)
    profile_file = os.path.join(profile_dir, "robot_profile.yaml")
    source_controllers_file = os.path.join(profile_dir, "robot_controllers.yaml")

    # ros2_control controller parameter files cannot reference another YAML file.
    # Materialize a runtime copy so drive.wheel_radius_m remains the only source.
    with open(profile_file, "r", encoding="utf-8") as stream:
        profile_data = yaml.safe_load(stream) or {}
    with open(source_controllers_file, "r", encoding="utf-8") as stream:
        controllers_data = yaml.safe_load(stream) or {}

    drive_cfg = profile_data.get("drive")
    if not isinstance(drive_cfg, dict):
        raise ValueError(f"Missing drive mapping in profile: {profile_file}")
    controller_name = str(drive_cfg.get("controller_name", "")).strip()
    controller_type = str(drive_cfg.get("controller_type", "")).strip()
    wheel_radius = float(drive_cfg["wheel_radius_m"])
    controller_cfg = controllers_data.get(controller_name)
    controller_params = (
        controller_cfg.get("ros__parameters") if isinstance(controller_cfg, dict) else None
    )
    if not isinstance(controller_params, dict):
        raise ValueError(
            f"Missing {controller_name}.ros__parameters in {source_controllers_file}"
        )
    if controller_type == "diff_drive_controller/DiffDriveController":
        controller_params["wheel_radius"] = wheel_radius
    elif controller_type == "mecanum_drive_controller/MecanumDriveController":
        controller_params["kinematics.wheels_radius"] = wheel_radius
    else:
        raise ValueError(f"Unsupported drive controller type: {controller_type}")

    tf_suffix = ""
    if enable_odom_tf is not None:
        if not isinstance(enable_odom_tf, bool):
            raise ValueError("enable_odom_tf must be a boolean when provided")
        controller_params["enable_odom_tf"] = enable_odom_tf
        tf_suffix = "_tf" if enable_odom_tf else "_no_tf"

    rate_suffix = ""
    if control_rate_hz is not None:
        raw_rate = str(control_rate_hz).strip()
        try:
            rate = int(raw_rate)
        except (TypeError, ValueError) as exc:
            raise ValueError("control_rate_hz must be a positive integer") from exc
        if rate <= 0 or str(rate) != raw_rate:
            raise ValueError("control_rate_hz must be a positive integer")

        manager_cfg = controllers_data.get("controller_manager")
        manager_params = (
            manager_cfg.get("ros__parameters") if isinstance(manager_cfg, dict) else None
        )
        if not isinstance(manager_params, dict):
            raise ValueError(
                f"Missing controller_manager.ros__parameters in {source_controllers_file}"
            )
        manager_params["update_rate"] = rate
        rate_suffix = f"_{rate}hz"

        # A drive controller cannot publish more frequently than the manager
        # updates it. This keeps hardware odometry at the selected loop rate.
        if "publish_rate" in controller_params:
            controller_params["publish_rate"] = float(rate)

    safe_profile_name = re.sub(r"[^A-Za-z0-9_-]", "_", profile_name)
    # Hardware launch runs as root while builds and tests run as the normal
    # account. A shared predictable /tmp directory lets the first user block
    # the other with ownership and mode 0700. Give every process a private,
    # securely-created directory instead.
    runtime_dir = tempfile.mkdtemp(prefix=f"studica_vmxpi_ros2_{os.getpid()}_")
    controllers_file = os.path.join(
        runtime_dir,
        f"controllers_{safe_profile_name}{rate_suffix}{tf_suffix}.yaml",
    )
    with open(controllers_file, "w", encoding="utf-8") as stream:
        yaml.safe_dump(controllers_data, stream, sort_keys=False)
    return profile_file, controllers_file


def _profile_camera_tf_base_link(profile_name: str):
    """
    Resolve base_link -> camera_link transform using the same defaults as robot URDF.

    URDF chain:
      base_link -> chassis_link: z = ground_clearance + base_height / 2
        (legacy profiles fall back to wheel_radius - wheel_z_offset)
      chassis_link -> camera_link: (cam_pos_x, cam_pos_y, cam_pos_z, cam_rpy)
    """
    profile_file, _ = _profile_assets(profile_name)
    with open(profile_file, "r", encoding="utf-8") as stream:
        profile_data = yaml.safe_load(stream) or {}

    xacro_cfg = profile_data.get("xacro")
    drive_cfg = profile_data.get("drive")
    if not isinstance(xacro_cfg, dict):
        raise ValueError(f"Missing xacro mapping in profile: {profile_file}")
    if not isinstance(drive_cfg, dict):
        raise ValueError(f"Missing drive mapping in profile: {profile_file}")

    # Keep defaults synchronized with description/robot/urdf/robot_description.urdf.xacro.
    base_length = float(xacro_cfg.get("base_length", 0.4))
    base_height = float(xacro_cfg.get("base_height", 0.1))
    wheel_radius = float(drive_cfg.get("wheel_radius_m", 0.05))
    wheel_z_offset = float(xacro_cfg.get("wheel_z_offset", -0.027))

    cam_pos_x = float(xacro_cfg.get("cam_pos_x", base_length / 2.0))
    cam_pos_y = float(xacro_cfg.get("cam_pos_y", 0.0))
    cam_pos_z = float(xacro_cfg.get("cam_pos_z", base_height / 2.0))
    cam_roll = float(xacro_cfg.get("cam_roll", 0.0))
    cam_pitch = float(xacro_cfg.get("cam_pitch", 0.0))
    cam_yaw = float(xacro_cfg.get("cam_yaw", 0.0))

    if "ground_clearance" in xacro_cfg:
        base_to_chassis_z = float(xacro_cfg["ground_clearance"]) + base_height / 2.0
    else:
        base_to_chassis_z = wheel_radius - wheel_z_offset
    return {
        "x": f"{cam_pos_x:.6f}",
        "y": f"{cam_pos_y:.6f}",
        "z": f"{(base_to_chassis_z + cam_pos_z):.6f}",
        "roll": f"{cam_roll:.6f}",
        "pitch": f"{cam_pitch:.6f}",
        "yaw": f"{cam_yaw:.6f}",
    }


def _profile_lidar_type(profile_name: str, default: str = "tmini") -> str:
    """Resolve optional hardware.lidar_type from robot profile (fallback to default)."""
    profile_file, _ = _profile_assets(profile_name)
    with open(profile_file, "r", encoding="utf-8") as stream:
        profile_data = yaml.safe_load(stream) or {}

    hw_cfg = profile_data.get("hardware")
    if not isinstance(hw_cfg, dict):
        return default

    lidar_type = hw_cfg.get("lidar_type")
    if lidar_type is None:
        return default

    value = str(lidar_type).strip()
    return value if value else default
