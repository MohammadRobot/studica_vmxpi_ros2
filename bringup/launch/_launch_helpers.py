# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared utilities and profile helpers for studica_vmxpi_ros2 launch files."""

import os
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
