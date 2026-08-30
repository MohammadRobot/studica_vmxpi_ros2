#!/usr/bin/env python3
"""Validate the VMXPi production package manifest without changing the host."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any

import yaml


GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")
REQUIRED_APT_PACKAGES = {
    "bluez",
    "network-manager",
    "openssh-server",
    "ros-humble-ros-base",
    "ros-humble-rmw-cyclonedds-cpp",
    "ufw",
    "wpasupplicant",
}
REQUIRED_SOURCE_PACKAGES = {
    "studica_drivers",
    "studica_robot_monitor",
    "studica_ros2_control",
    "studica_vmxpi_ros2",
}


def load_json(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return document


def package_list(value: Any, field: str, failures: list[str]) -> list[str]:
    if not isinstance(value, list) or any(
        not isinstance(item, str) or not item for item in value
    ):
        failures.append(f"{field} must be a list of non-empty package names")
        return []
    if value != sorted(value):
        failures.append(f"{field} must be sorted")
    if len(value) != len(set(value)):
        failures.append(f"{field} contains duplicate package names")
    return value


def load_hardware_pins(root: Path, failures: list[str]) -> dict[str, str]:
    path = root / "dependencies" / "hardware.repos"
    try:
        repositories = yaml.safe_load(path.read_text(encoding="utf-8"))["repositories"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        failures.append(f"cannot read dependencies/hardware.repos: {error}")
        return {}

    pins: dict[str, str] = {}
    for name, metadata in repositories.items():
        version = str(metadata.get("version", ""))
        pins[str(name)] = version
        if GIT_SHA_RE.fullmatch(version) is None:
            failures.append(f"hardware source {name} is not pinned to a full commit")
    return pins


def validate_manifest(root: Path, manifest: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if manifest.get("schema_version") != 1:
        failures.append("schema_version must be 1")
    if manifest.get("profile") != "vmxpi-runtime-packages-v1":
        failures.append("profile must be vmxpi-runtime-packages-v1")

    platform = manifest.get("platform")
    if not isinstance(platform, dict):
        failures.append("platform must be an object")
        platform = {}
    expected_platform = {
        "os_id": "ubuntu",
        "version_id": "22.04",
        "architecture": "arm64",
        "ros_distro": "humble",
        "ros_variant": "ros-base",
    }
    if platform != expected_platform:
        failures.append(f"platform must equal {expected_platform}")

    audit_profile_path = root / "deployment" / "vmxpi-production-v1.json"
    try:
        audit_platform = load_json(audit_profile_path)["platform"]
    except (OSError, ValueError, json.JSONDecodeError, KeyError) as error:
        failures.append(f"cannot read production audit platform: {error}")
        audit_platform = {}
    for key in ("os_id", "version_id", "architecture"):
        if platform.get(key) != audit_platform.get(key):
            failures.append(f"package and audit profiles disagree on platform.{key}")

    base_image = manifest.get("base_image")
    if base_image != {"family": "ubuntu-server", "install_mode": "minimal"}:
        failures.append("base_image must select the minimal Ubuntu Server base")

    required_groups = manifest.get("required_apt_packages")
    if not isinstance(required_groups, dict):
        failures.append("required_apt_packages must be an object")
        required_groups = {}
    if set(required_groups) != {"operating_system", "ros_core"}:
        failures.append("required_apt_packages must contain operating_system and ros_core")

    enabled_apt: list[str] = []
    all_declared_apt: list[str] = []
    for group_name in sorted(required_groups):
        packages = package_list(
            required_groups[group_name],
            f"required_apt_packages.{group_name}",
            failures,
        )
        enabled_apt.extend(packages)
        all_declared_apt.extend(packages)

    required_source = package_list(
        manifest.get("required_source_packages"),
        "required_source_packages",
        failures,
    )
    enabled_source = list(required_source)
    all_declared_source = list(required_source)

    features = manifest.get("features")
    if not isinstance(features, dict) or not features:
        failures.append("features must be a non-empty object")
        features = {}
    for feature_name in sorted(features):
        feature = features[feature_name]
        if not isinstance(feature, dict) or set(feature) != {
            "enabled",
            "apt_packages",
            "source_packages",
        }:
            failures.append(
                f"features.{feature_name} must contain enabled, apt_packages, and source_packages"
            )
            continue
        if not isinstance(feature["enabled"], bool):
            failures.append(f"features.{feature_name}.enabled must be boolean")
        apt_packages = package_list(
            feature["apt_packages"],
            f"features.{feature_name}.apt_packages",
            failures,
        )
        source_packages = package_list(
            feature["source_packages"],
            f"features.{feature_name}.source_packages",
            failures,
        )
        all_declared_apt.extend(apt_packages)
        all_declared_source.extend(source_packages)
        if feature.get("enabled") is True:
            enabled_apt.extend(apt_packages)
            enabled_source.extend(source_packages)

    for package_type, packages in (
        ("APT", all_declared_apt),
        ("source", all_declared_source),
    ):
        duplicates = sorted(
            package for package in set(packages) if packages.count(package) > 1
        )
        if duplicates:
            failures.append(f"{package_type} packages occur in multiple groups: {duplicates}")

    missing_apt = sorted(REQUIRED_APT_PACKAGES - set(enabled_apt))
    if missing_apt:
        failures.append(f"enabled runtime is missing required APT packages: {missing_apt}")
    missing_source = sorted(REQUIRED_SOURCE_PACKAGES - set(enabled_source))
    if missing_source:
        failures.append(f"enabled runtime is missing required source packages: {missing_source}")

    joystick = features.get("bluetooth_joystick", {})
    if joystick.get("enabled") is not True:
        failures.append("Bluetooth joystick support must remain enabled")
    joystick_packages = set(joystick.get("apt_packages", []))
    for package in ("ros-humble-joy", "ros-humble-teleop-twist-joy"):
        if package not in joystick_packages:
            failures.append(f"Bluetooth joystick feature is missing {package}")

    prohibited = package_list(
        manifest.get("prohibited_apt_packages"),
        "prohibited_apt_packages",
        failures,
    )
    prohibited_prefixes = package_list(
        manifest.get("prohibited_apt_prefixes"),
        "prohibited_apt_prefixes",
        failures,
    )
    for package in enabled_apt:
        if package in prohibited:
            failures.append(f"enabled package is prohibited: {package}")
        for prefix in prohibited_prefixes:
            if package.startswith(prefix):
                failures.append(f"enabled package matches prohibited prefix {prefix}: {package}")
    if "ros-humble-desktop" not in prohibited or "ubuntu-desktop" not in prohibited:
        failures.append("desktop ROS and Ubuntu metapackages must be prohibited")

    hardware_pins = load_hardware_pins(root, failures)
    for package in sorted(set(all_declared_source) - {"studica_vmxpi_ros2"}):
        if package not in hardware_pins:
            failures.append(f"source package is missing from hardware.repos: {package}")

    return failures


def enabled_apt_packages(manifest: dict[str, Any]) -> list[str]:
    packages: list[str] = []
    for group in manifest["required_apt_packages"].values():
        packages.extend(group)
    for feature in manifest["features"].values():
        if feature["enabled"]:
            packages.extend(feature["apt_packages"])
    return sorted(packages)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--manifest", type=Path)
    parser.add_argument(
        "--print-enabled-apt",
        action="store_true",
        help="print the validated enabled APT package names, one per line",
    )
    arguments = parser.parse_args()
    root = arguments.root.resolve()
    manifest_path = arguments.manifest or (
        root / "deployment" / "vmxpi-runtime-packages-v1.json"
    )
    try:
        manifest = load_json(manifest_path)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"Production package manifest check failed: {error}")
        return 1

    failures = validate_manifest(root, manifest)
    if failures:
        print("Production package manifest check failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    if arguments.print_enabled_apt:
        print("\n".join(enabled_apt_packages(manifest)))
    else:
        print("[check] Production package manifest: Ubuntu 22.04/Humble minimal runtime")
    return 0


if __name__ == "__main__":
    sys.exit(main())
