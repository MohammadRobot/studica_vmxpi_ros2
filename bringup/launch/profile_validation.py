#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Shared robot-profile validation helpers used by launch and scripts."""

from __future__ import annotations

import re
from numbers import Real
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        "Missing PyYAML. Install it with 'sudo apt install python3-yaml' "
        "or 'pip install pyyaml'."
    ) from exc


PROFILE_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")

REQUIRED_XACRO_KEYS = (
    "base_mass",
    "base_width",
    "base_length",
    "base_height",
    "wheel_mass",
    "wheel_len",
    "wheel_radius",
    "caster_wheel_mass",
    "caster_wheel_radius",
    "laser_model_z",
    "laser_frame_z",
)

REQUIRED_HW_KEYS = (
    "can_id",
    "motor_freq",
    "ticks_per_rotation",
    "wheel_radius",
    "speed_scale",
    "max_wheel_angular_velocity_rad_s",
    "left_front_motor",
    "left_rear_motor",
    "right_front_motor",
    "right_rear_motor",
    "invert_left_front_motor",
    "invert_left_rear_motor",
    "invert_right_front_motor",
    "invert_right_rear_motor",
    "invert_left_front_encoder",
    "invert_left_rear_encoder",
    "invert_right_front_encoder",
    "invert_right_rear_encoder",
)

NUMERIC_HW_KEYS = (
    "can_id",
    "motor_freq",
    "ticks_per_rotation",
    "wheel_radius",
    "speed_scale",
    "max_wheel_angular_velocity_rad_s",
    "left_front_motor",
    "left_rear_motor",
    "right_front_motor",
    "right_rear_motor",
)

BOOL_HW_KEYS = tuple(k for k in REQUIRED_HW_KEYS if k.startswith("invert_"))

SUPPORTED_WHEEL_LAYOUTS = ("diff", "diff_4wd", "mecanum", "omni")
SUPPORTED_DRIVE_CONTROLLER_TYPES = (
    "diff_drive_controller/DiffDriveController",
    "mecanum_drive_controller/MecanumDriveController",
)

DEFAULT_DRIVE_CONTROLLER_NAME = "robot_base_controller"
DEFAULT_DRIVE_CONTROLLER_TYPE = "diff_drive_controller/DiffDriveController"
DEFAULT_WHEEL_LAYOUT = "diff"


def _is_number(value: Any) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool)


def load_yaml(path: Path | str) -> dict[str, Any]:
    yaml_path = Path(path)
    with yaml_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping")
    return data


def drive_topics(controller_name: str, controller_type: str) -> tuple[str, str]:
    if controller_type == "mecanum_drive_controller/MecanumDriveController":
        return f"/{controller_name}/reference", f"/{controller_name}/odometry"
    return f"/{controller_name}/cmd_vel", f"/{controller_name}/odom"


def _read_drive_config(
    profile_name: str,
    profile_file: Path,
    profile: dict[str, Any],
    errors: list[str],
) -> tuple[str, str, str]:
    drive_cfg_raw = profile.get("drive")
    drive_cfg = drive_cfg_raw if isinstance(drive_cfg_raw, dict) else {}

    if drive_cfg_raw is not None and not isinstance(drive_cfg_raw, dict):
        errors.append(f"{profile_name}: {profile_file} 'drive' must be a mapping")

    wheel_layout = str(drive_cfg.get("wheel_layout", DEFAULT_WHEEL_LAYOUT)).strip().lower()
    controller_name = str(
        drive_cfg.get("controller_name", DEFAULT_DRIVE_CONTROLLER_NAME)
    ).strip()
    controller_type = str(
        drive_cfg.get("controller_type", DEFAULT_DRIVE_CONTROLLER_TYPE)
    ).strip()

    if wheel_layout not in SUPPORTED_WHEEL_LAYOUTS:
        errors.append(
            f"{profile_name}: {profile_file} drive.wheel_layout must be one of "
            f"{SUPPORTED_WHEEL_LAYOUTS}"
        )
    if not controller_name:
        errors.append(f"{profile_name}: {profile_file} drive.controller_name cannot be empty")
    if controller_type not in SUPPORTED_DRIVE_CONTROLLER_TYPES:
        errors.append(
            f"{profile_name}: {profile_file} drive.controller_type must be one of "
            f"{SUPPORTED_DRIVE_CONTROLLER_TYPES}"
        )

    if wheel_layout in ("diff", "diff_4wd") and (
        controller_type != "diff_drive_controller/DiffDriveController"
    ):
        errors.append(
            f"{profile_name}: {profile_file} {wheel_layout} layout requires "
            "diff_drive_controller"
        )
    if wheel_layout in ("mecanum", "omni") and (
        controller_type != "mecanum_drive_controller/MecanumDriveController"
    ):
        errors.append(
            f"{profile_name}: {profile_file} {wheel_layout} layout requires "
            "mecanum_drive_controller"
        )

    return controller_name, controller_type, wheel_layout


def validate_profile_files(
    profile_name: str,
    profile_file: Path | str,
    controllers_file: Path | str,
) -> tuple[list[str], str, str, str]:
    profile_path = Path(profile_file)
    controllers_path = Path(controllers_file)
    errors: list[str] = []

    drive_controller_name = DEFAULT_DRIVE_CONTROLLER_NAME
    drive_controller_type = DEFAULT_DRIVE_CONTROLLER_TYPE
    drive_wheel_layout = DEFAULT_WHEEL_LAYOUT

    if not PROFILE_NAME_RE.fullmatch(profile_name):
        errors.append(
            f"{profile_name}: invalid directory name "
            "(allowed: letters, numbers, '_' and '-')"
        )

    if not profile_path.exists():
        errors.append(f"{profile_name}: missing file {profile_path}")
    if not controllers_path.exists():
        errors.append(f"{profile_name}: missing file {controllers_path}")
    if errors:
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout

    try:
        profile = load_yaml(profile_path)
    except Exception as exc:  # pylint: disable=broad-except
        errors.append(f"{profile_name}: failed to parse {profile_path}: {exc}")
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout

    try:
        controllers = load_yaml(controllers_path)
    except Exception as exc:  # pylint: disable=broad-except
        errors.append(f"{profile_name}: failed to parse {controllers_path}: {exc}")
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout

    xacro_cfg = profile.get("xacro")
    hw_cfg = profile.get("hardware")

    if not isinstance(xacro_cfg, dict):
        errors.append(f"{profile_name}: {profile_path} missing 'xacro' mapping")
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout
    if not isinstance(hw_cfg, dict):
        errors.append(f"{profile_name}: {profile_path} missing 'hardware' mapping")
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout

    (
        drive_controller_name,
        drive_controller_type,
        drive_wheel_layout,
    ) = _read_drive_config(profile_name, profile_path, profile, errors)

    for key in REQUIRED_XACRO_KEYS:
        if key not in xacro_cfg:
            errors.append(f"{profile_name}: {profile_path} missing xacro key '{key}'")
            continue
        if not _is_number(xacro_cfg[key]):
            errors.append(
                f"{profile_name}: {profile_path} xacro key '{key}' must be numeric"
            )

    for key in REQUIRED_HW_KEYS:
        if key not in hw_cfg:
            errors.append(f"{profile_name}: {profile_path} missing hardware key '{key}'")

    if errors:
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout

    for key in NUMERIC_HW_KEYS:
        if not _is_number(hw_cfg[key]):
            errors.append(
                f"{profile_name}: {profile_path} hardware key '{key}' must be numeric"
            )
    for key in BOOL_HW_KEYS:
        if not isinstance(hw_cfg[key], bool):
            errors.append(
                f"{profile_name}: {profile_path} hardware key '{key}' must be bool"
            )

    if errors:
        return errors, drive_controller_name, drive_controller_type, drive_wheel_layout

    can_id = int(hw_cfg["can_id"])
    motor_freq = int(hw_cfg["motor_freq"])
    ticks_per_rotation = int(hw_cfg["ticks_per_rotation"])
    wheel_radius = float(hw_cfg["wheel_radius"])
    speed_scale = float(hw_cfg["speed_scale"])
    max_wheel_rad_s = float(hw_cfg["max_wheel_angular_velocity_rad_s"])

    left_front_motor = int(hw_cfg["left_front_motor"])
    left_rear_motor = int(hw_cfg["left_rear_motor"])
    right_front_motor = int(hw_cfg["right_front_motor"])
    right_rear_motor = int(hw_cfg["right_rear_motor"])

    if can_id < 0 or can_id > 255:
        errors.append(f"{profile_name}: {profile_path} can_id must be in [0, 255]")
    if motor_freq <= 0 or motor_freq > 65535:
        errors.append(f"{profile_name}: {profile_path} motor_freq must be in [1, 65535]")
    if ticks_per_rotation <= 0:
        errors.append(f"{profile_name}: {profile_path} ticks_per_rotation must be > 0")
    if wheel_radius <= 0.0:
        errors.append(f"{profile_name}: {profile_path} wheel_radius must be > 0")
    if speed_scale < 0.0:
        errors.append(f"{profile_name}: {profile_path} speed_scale must be >= 0")
    if max_wheel_rad_s <= 0.0:
        errors.append(
            f"{profile_name}: {profile_path} max_wheel_angular_velocity_rad_s must be > 0"
        )

    for motor in (
        left_front_motor,
        left_rear_motor,
        right_front_motor,
        right_rear_motor,
    ):
        if motor < -1 or motor > 3:
            errors.append(
                f"{profile_name}: {profile_path} motor indices must be -1 or in [0, 3]"
            )

    if left_front_motor < 0 and left_rear_motor < 0:
        errors.append(
            f"{profile_name}: {profile_path} at least one left motor index must be >= 0"
        )
    if right_front_motor < 0 and right_rear_motor < 0:
        errors.append(
            f"{profile_name}: {profile_path} at least one right motor index must be >= 0"
        )
    if drive_wheel_layout in ("diff_4wd", "mecanum", "omni"):
        for motor_name, motor_index in (
            ("left_front_motor", left_front_motor),
            ("left_rear_motor", left_rear_motor),
            ("right_front_motor", right_front_motor),
            ("right_rear_motor", right_rear_motor),
        ):
            if motor_index < 0:
                errors.append(
                    f"{profile_name}: {profile_path} {motor_name} must be >= 0 for "
                    f"{drive_wheel_layout} layout"
                )

    cm = controllers.get("controller_manager")
    drive_controller = controllers.get(drive_controller_name)
    if not isinstance(cm, dict):
        errors.append(f"{profile_name}: {controllers_path} missing 'controller_manager'")
    if not isinstance(drive_controller, dict):
        errors.append(
            f"{profile_name}: {controllers_path} missing '{drive_controller_name}'"
        )

    cm_params = cm.get("ros__parameters") if isinstance(cm, dict) else None
    drive_params = (
        drive_controller.get("ros__parameters")
        if isinstance(drive_controller, dict)
        else None
    )
    if not isinstance(cm_params, dict):
        errors.append(
            f"{profile_name}: {controllers_path} controller_manager missing 'ros__parameters'"
        )
    if not isinstance(drive_params, dict):
        errors.append(
            f"{profile_name}: {controllers_path} {drive_controller_name} missing 'ros__parameters'"
        )

    if isinstance(cm_params, dict):
        cm_drive_cfg = cm_params.get(drive_controller_name)
        if not isinstance(cm_drive_cfg, dict):
            errors.append(
                f"{profile_name}: {controllers_path} controller_manager must declare "
                f"{drive_controller_name}.type"
            )
        else:
            cm_drive_type = cm_drive_cfg.get("type")
            if cm_drive_type != drive_controller_type:
                errors.append(
                    f"{profile_name}: {controllers_path} {drive_controller_name}.type="
                    f"'{cm_drive_type}' does not match profile drive.controller_type="
                    f"'{drive_controller_type}'"
                )

    if isinstance(drive_params, dict) and (
        drive_controller_type == "diff_drive_controller/DiffDriveController"
    ):
        wheel_separation = drive_params.get("wheel_separation")
        wheel_radius_cfg = drive_params.get("wheel_radius")
        if not _is_number(wheel_separation) or float(wheel_separation) <= 0.0:
            errors.append(
                f"{profile_name}: {controllers_path} {drive_controller_name}.wheel_separation "
                "must be > 0"
            )
        if not _is_number(wheel_radius_cfg) or float(wheel_radius_cfg) <= 0.0:
            errors.append(
                f"{profile_name}: {controllers_path} {drive_controller_name}.wheel_radius "
                "must be > 0"
            )

    if isinstance(drive_params, dict) and (
        drive_controller_type == "mecanum_drive_controller/MecanumDriveController"
    ):
        required_joint_params = (
            "front_left_wheel_command_joint_name",
            "front_right_wheel_command_joint_name",
            "rear_right_wheel_command_joint_name",
            "rear_left_wheel_command_joint_name",
        )
        for key in required_joint_params:
            value = drive_params.get(key)
            if not isinstance(value, str) or not value.strip():
                errors.append(
                    f"{profile_name}: {controllers_path} {drive_controller_name}.{key} "
                    "must be a non-empty string"
                )

        wheels_radius_cfg = drive_params.get("kinematics.wheels_radius")
        proj_cfg = drive_params.get(
            "kinematics.sum_of_robot_center_projection_on_X_Y_axis"
        )
        if not _is_number(wheels_radius_cfg) or float(wheels_radius_cfg) <= 0.0:
            errors.append(
                f"{profile_name}: {controllers_path} {drive_controller_name}."
                "kinematics.wheels_radius must be > 0"
            )
        if not _is_number(proj_cfg):
            errors.append(
                f"{profile_name}: {controllers_path} {drive_controller_name}."
                "kinematics.sum_of_robot_center_projection_on_X_Y_axis must be numeric"
            )

    return errors, drive_controller_name, drive_controller_type, drive_wheel_layout


def validate_profile_directory(
    profile_dir: Path | str,
) -> tuple[list[str], str, str, str]:
    path = Path(profile_dir)
    return validate_profile_files(
        profile_name=path.name,
        profile_file=path / "robot_profile.yaml",
        controllers_file=path / "robot_controllers.yaml",
    )


def validate_profile_for_launch(
    package_share_dir: str,
    profile_name: str,
) -> tuple[str, str, str, str, str]:
    profile_dir = Path(package_share_dir) / "config" / "profiles" / profile_name
    profile_file = profile_dir / "robot_profile.yaml"
    controllers_file = profile_dir / "robot_controllers.yaml"
    errors, controller_name, controller_type, wheel_layout = validate_profile_files(
        profile_name=profile_name,
        profile_file=profile_file,
        controllers_file=controllers_file,
    )
    if errors:
        joined_errors = "\n  - ".join(errors)
        raise RuntimeError(
            f"Profile validation failed for '{profile_name}':\n  - {joined_errors}"
        )
    return (
        str(profile_file),
        str(controllers_file),
        controller_name,
        controller_type,
        wheel_layout,
    )
