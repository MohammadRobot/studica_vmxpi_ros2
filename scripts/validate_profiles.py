#!/usr/bin/env python3
"""Validate all robot profile YAML files."""

from __future__ import annotations

import argparse
import re
import sys
from numbers import Real
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError as exc:  # pragma: no cover
    print(
        "ERROR: Missing PyYAML. Install it with 'sudo apt install python3-yaml' "
        "or 'pip install pyyaml'.",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


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


def _is_number(value: Any) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool)


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError("YAML root must be a mapping")
    return data


def _validate_profile_dir(profile_dir: Path) -> list[str]:
    errors: list[str] = []
    profile_name = profile_dir.name

    profile_file = profile_dir / "robot_profile.yaml"
    controllers_file = profile_dir / "robot_controllers.yaml"

    if not PROFILE_NAME_RE.fullmatch(profile_name):
        errors.append(
            f"{profile_name}: invalid directory name "
            "(allowed: letters, numbers, '_' and '-')"
        )

    if not profile_file.exists():
        errors.append(f"{profile_name}: missing file {profile_file}")
    if not controllers_file.exists():
        errors.append(f"{profile_name}: missing file {controllers_file}")

    if errors:
        return errors

    try:
        profile = _load_yaml(profile_file)
    except Exception as exc:  # pylint: disable=broad-except
        errors.append(f"{profile_name}: failed to parse {profile_file}: {exc}")
        return errors

    try:
        controllers = _load_yaml(controllers_file)
    except Exception as exc:  # pylint: disable=broad-except
        errors.append(f"{profile_name}: failed to parse {controllers_file}: {exc}")
        return errors

    xacro_cfg = profile.get("xacro")
    hw_cfg = profile.get("hardware")
    drive_cfg_raw = profile.get("drive")
    drive_cfg = drive_cfg_raw if isinstance(drive_cfg_raw, dict) else {}

    if not isinstance(xacro_cfg, dict):
        errors.append(f"{profile_name}: {profile_file} missing 'xacro' mapping")
        return errors
    if not isinstance(hw_cfg, dict):
        errors.append(f"{profile_name}: {profile_file} missing 'hardware' mapping")
        return errors
    if drive_cfg_raw is not None and not isinstance(drive_cfg_raw, dict):
        errors.append(f"{profile_name}: {profile_file} 'drive' must be a mapping")
        return errors

    drive_wheel_layout = str(drive_cfg.get("wheel_layout", "diff")).strip().lower()
    drive_controller_name = str(
        drive_cfg.get("controller_name", "robot_base_controller")
    ).strip()
    drive_controller_type = str(
        drive_cfg.get(
            "controller_type", "diff_drive_controller/DiffDriveController"
        )
    ).strip()

    if drive_wheel_layout not in SUPPORTED_WHEEL_LAYOUTS:
        errors.append(
            f"{profile_name}: {profile_file} drive.wheel_layout must be one of "
            f"{SUPPORTED_WHEEL_LAYOUTS}"
        )
    if not drive_controller_name:
        errors.append(f"{profile_name}: {profile_file} drive.controller_name cannot be empty")
    if drive_controller_type not in SUPPORTED_DRIVE_CONTROLLER_TYPES:
        errors.append(
            f"{profile_name}: {profile_file} drive.controller_type must be one of "
            f"{SUPPORTED_DRIVE_CONTROLLER_TYPES}"
        )

    if drive_wheel_layout in ("diff", "diff_4wd") and (
        drive_controller_type != "diff_drive_controller/DiffDriveController"
    ):
        errors.append(
            f"{profile_name}: {profile_file} {drive_wheel_layout} layout requires "
            "diff_drive_controller"
        )
    if drive_wheel_layout in ("mecanum", "omni") and (
        drive_controller_type != "mecanum_drive_controller/MecanumDriveController"
    ):
        errors.append(
            f"{profile_name}: {profile_file} {drive_wheel_layout} layout requires "
            "mecanum_drive_controller"
        )

    for key in REQUIRED_XACRO_KEYS:
        if key not in xacro_cfg:
            errors.append(f"{profile_name}: {profile_file} missing xacro key '{key}'")
            continue
        if not _is_number(xacro_cfg[key]):
            errors.append(
                f"{profile_name}: {profile_file} xacro key '{key}' must be numeric"
            )

    for key in REQUIRED_HW_KEYS:
        if key not in hw_cfg:
            errors.append(f"{profile_name}: {profile_file} missing hardware key '{key}'")

    if errors:
        return errors

    for key in NUMERIC_HW_KEYS:
        if not _is_number(hw_cfg[key]):
            errors.append(
                f"{profile_name}: {profile_file} hardware key '{key}' must be numeric"
            )
    for key in BOOL_HW_KEYS:
        if not isinstance(hw_cfg[key], bool):
            errors.append(
                f"{profile_name}: {profile_file} hardware key '{key}' must be bool"
            )

    if errors:
        return errors

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
        errors.append(f"{profile_name}: {profile_file} can_id must be in [0, 255]")
    if motor_freq <= 0 or motor_freq > 65535:
        errors.append(f"{profile_name}: {profile_file} motor_freq must be in [1, 65535]")
    if ticks_per_rotation <= 0:
        errors.append(f"{profile_name}: {profile_file} ticks_per_rotation must be > 0")
    if wheel_radius <= 0.0:
        errors.append(f"{profile_name}: {profile_file} wheel_radius must be > 0")
    if speed_scale < 0.0:
        errors.append(f"{profile_name}: {profile_file} speed_scale must be >= 0")
    if max_wheel_rad_s <= 0.0:
        errors.append(
            f"{profile_name}: {profile_file} max_wheel_angular_velocity_rad_s must be > 0"
        )

    for motor in (
        left_front_motor,
        left_rear_motor,
        right_front_motor,
        right_rear_motor,
    ):
        if motor < -1 or motor > 3:
            errors.append(
                f"{profile_name}: {profile_file} motor indices must be -1 or in [0, 3]"
            )

    if left_front_motor < 0 and left_rear_motor < 0:
        errors.append(
            f"{profile_name}: {profile_file} at least one left motor index must be >= 0"
        )
    if right_front_motor < 0 and right_rear_motor < 0:
        errors.append(
            f"{profile_name}: {profile_file} at least one right motor index must be >= 0"
        )
    if drive_wheel_layout in ("mecanum", "omni"):
        for motor_name, motor_index in (
            ("left_front_motor", left_front_motor),
            ("left_rear_motor", left_rear_motor),
            ("right_front_motor", right_front_motor),
            ("right_rear_motor", right_rear_motor),
        ):
            if motor_index < 0:
                errors.append(
                    f"{profile_name}: {profile_file} {motor_name} must be >= 0 for "
                    f"{drive_wheel_layout} layout"
                )

    cm = controllers.get("controller_manager")
    drive_controller = controllers.get(drive_controller_name)
    if not isinstance(cm, dict):
        errors.append(f"{profile_name}: {controllers_file} missing 'controller_manager'")
    if not isinstance(drive_controller, dict):
        errors.append(
            f"{profile_name}: {controllers_file} missing '{drive_controller_name}'"
        )

    cm_params = cm.get("ros__parameters") if isinstance(cm, dict) else None
    drive_params = (
        drive_controller.get("ros__parameters")
        if isinstance(drive_controller, dict)
        else None
    )
    if not isinstance(cm_params, dict):
        errors.append(
            f"{profile_name}: {controllers_file} controller_manager missing 'ros__parameters'"
        )
    if not isinstance(drive_params, dict):
        errors.append(
            f"{profile_name}: {controllers_file} {drive_controller_name} missing 'ros__parameters'"
        )

    if isinstance(cm_params, dict):
        cm_drive_cfg = cm_params.get(drive_controller_name)
        if not isinstance(cm_drive_cfg, dict):
            errors.append(
                f"{profile_name}: {controllers_file} controller_manager must declare "
                f"{drive_controller_name}.type"
            )
        else:
            cm_drive_type = cm_drive_cfg.get("type")
            if cm_drive_type != drive_controller_type:
                errors.append(
                    f"{profile_name}: {controllers_file} {drive_controller_name}.type="
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
                f"{profile_name}: {controllers_file} {drive_controller_name}.wheel_separation "
                "must be > 0"
            )
        if not _is_number(wheel_radius_cfg) or float(wheel_radius_cfg) <= 0.0:
            errors.append(
                f"{profile_name}: {controllers_file} {drive_controller_name}.wheel_radius "
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
                    f"{profile_name}: {controllers_file} {drive_controller_name}.{key} "
                    "must be a non-empty string"
                )

        wheels_radius_cfg = drive_params.get("kinematics.wheels_radius")
        proj_cfg = drive_params.get(
            "kinematics.sum_of_robot_center_projection_on_X_Y_axis"
        )
        if not _is_number(wheels_radius_cfg) or float(wheels_radius_cfg) <= 0.0:
            errors.append(
                f"{profile_name}: {controllers_file} {drive_controller_name}."
                "kinematics.wheels_radius must be > 0"
            )
        if not _is_number(proj_cfg):
            errors.append(
                f"{profile_name}: {controllers_file} {drive_controller_name}."
                "kinematics.sum_of_robot_center_projection_on_X_Y_axis must be numeric"
            )

    return errors


def _parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_profiles_dir = script_dir.parent / "bringup" / "config" / "profiles"
    default_template_dir = script_dir.parent / "bringup" / "config" / "profile_template"

    parser = argparse.ArgumentParser(description="Validate all robot profile YAML files.")
    parser.add_argument(
        "--profiles-dir",
        default=str(default_profiles_dir),
        help="Path to profiles directory (default: %(default)s)",
    )
    parser.add_argument(
        "--template-dir",
        default=str(default_template_dir),
        help="Path to template directory (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    profiles_dir = Path(args.profiles_dir).resolve()
    template_dir = Path(args.template_dir).resolve()

    if not profiles_dir.exists():
        print(f"ERROR: profiles directory does not exist: {profiles_dir}", file=sys.stderr)
        return 2
    if not profiles_dir.is_dir():
        print(f"ERROR: profiles path is not a directory: {profiles_dir}", file=sys.stderr)
        return 2

    profile_dirs = sorted(
        profile_dir
        for profile_dir in profiles_dir.iterdir()
        if profile_dir.is_dir() and not profile_dir.name.startswith(".")
    )
    if not profile_dirs:
        print(f"ERROR: no profile directories found in {profiles_dir}", file=sys.stderr)
        return 2

    if template_dir.exists():
        if not template_dir.is_dir():
            print(
                f"ERROR: template path is not a directory: {template_dir}",
                file=sys.stderr,
            )
            return 2
        profile_dirs.append(template_dir)
    else:
        print(f"WARNING: template directory not found: {template_dir}", file=sys.stderr)

    all_errors: list[str] = []
    for profile_dir in profile_dirs:
        errors = _validate_profile_dir(profile_dir)
        if errors:
            all_errors.extend(errors)
            print(f"[FAIL] {profile_dir.name}")
        else:
            print(f"[OK]   {profile_dir.name}")

    if all_errors:
        print("\nProfile validation failed with the following errors:", file=sys.stderr)
        for error in all_errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(f"\nValidated {len(profile_dirs)} profile(s) successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
