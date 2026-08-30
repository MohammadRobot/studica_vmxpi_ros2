#!/usr/bin/env python3
"""Validate the VMXPi production package manifest without changing the host."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any
import xml.etree.ElementTree as ET

import yaml


GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")
REQUIRED_APT_PACKAGES = {
    "bluez",
    "network-manager",
    "openssh-server",
    "ros-humble-backward-ros",
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
EXPECTED_OVERLAY_ROS_PACKAGES = {
    "orbbec_camera",
    "orbbec_camera_msgs",
    "orbbec_description",
    "studica_drivers",
    "studica_robot_monitor",
    "studica_ros2_control",
    "studica_vmxpi_ros2",
    "ydlidar_ros2_driver",
}
EXPECTED_EXTERNAL_RUNTIME_DEPENDENCIES = {
    "studica_vmxpi_hal_cpp": {
        "build_headers": "/usr/local/include/vmxpi",
        "identity": "content-sha256",
        "license": "NOASSERTION",
        "package_manager": "unmanaged",
        "runtime_library": "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so",
        "supplier": "Studica",
    }
}
EXPECTED_TRANSITIVE_BUILD_TOOLS = {
    "cmake": {
        "reason": (
            "ROS Humble binary runtime packages depend on ament-cmake, which depends "
            "on CMake; removing it also removes the ROS control runtime."
        ),
        "required_by": [
            "ros-humble-ament-cmake",
            "ros-humble-ament-cmake-core",
        ],
    },
    "cmake-data": {
        "reason": (
            "CMake data files are an inseparable dependency of the CMake package "
            "required transitively by ROS Humble binary packages."
        ),
        "required_by": ["cmake"],
    },
}
APT_BUNDLES = (
    "dependencies/apt/development-core.txt",
    "dependencies/apt/development-desktop.txt",
    "dependencies/apt/simulation-harmonic.txt",
)
CORE_REQUIRED_ROSDEP_KEYS = {
    "controller_manager",
    "diagnostic_aggregator",
    "diff_drive_controller",
    "forward_command_controller",
    "imu_sensor_broadcaster",
    "joy",
    "rmw_cyclonedds_cpp",
    "robot_localization",
    "robot_state_publisher",
    "studica_drivers",
    "studica_robot_monitor",
    "teleop_twist_joy",
}
CORE_FORBIDDEN_ROSDEP_KEYS = {
    "action_msgs",
    "foxglove_bridge",
    "gz_ros2_control",
    "joint_state_publisher_gui",
    "mecanum_drive_controller",
    "nav2_bringup",
    "nav2_msgs",
    "ros2controlcli",
    "ros_gz_bridge",
    "ros_gz_sim",
    "rosbag2_storage_mcap",
    "rviz2",
    "slam_toolbox",
    "teleop_twist_keyboard",
}
DEVELOPMENT_APT_COVERAGE = {
    "action_msgs": "ros-humble-action-msgs",
    "foxglove_bridge": "ros-humble-foxglove-bridge",
    "joint_state_publisher_gui": "ros-humble-joint-state-publisher-gui",
    "mecanum_drive_controller": "ros-humble-mecanum-drive-controller",
    "nav2_bringup": "ros-humble-nav2-bringup",
    "nav2_msgs": "ros-humble-nav2-msgs",
    "ros2controlcli": "ros-humble-ros2controlcli",
    "ros_gz_bridge": "ros-humble-ros-gzharmonic-bridge",
    "ros_gz_sim": "ros-humble-ros-gzharmonic",
    "rosbag2_storage_mcap": "ros-humble-rosbag2-storage-mcap",
    "rviz2": "ros-humble-rviz2",
    "slam_toolbox": "ros-humble-slam-toolbox",
    "teleop_twist_keyboard": "ros-humble-teleop-twist-keyboard",
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


def load_development_apt_bundles(root: Path, failures: list[str]) -> set[str]:
    all_packages: list[str] = []
    package_name = re.compile(r"[a-z0-9][a-z0-9+.-]*")
    for relative_path in APT_BUNDLES:
        path = root / relative_path
        try:
            packages = [
                line.strip()
                for line in path.read_text(encoding="utf-8").splitlines()
                if line.strip() and not line.lstrip().startswith("#")
            ]
        except OSError as error:
            failures.append(f"cannot read {relative_path}: {error}")
            continue
        if packages != sorted(packages):
            failures.append(f"{relative_path} package names must be sorted")
        if len(packages) != len(set(packages)):
            failures.append(f"{relative_path} contains duplicate package names")
        invalid = [item for item in packages if package_name.fullmatch(item) is None]
        if invalid:
            failures.append(f"{relative_path} contains invalid package names: {invalid}")
        all_packages.extend(packages)

    duplicates = sorted(
        package for package in set(all_packages) if all_packages.count(package) > 1
    )
    if duplicates:
        failures.append(f"APT packages occur in multiple development bundles: {duplicates}")
    return set(all_packages)


def validate_dependency_boundary(root: Path, failures: list[str]) -> None:
    try:
        package_root = ET.parse(root / "package.xml").getroot()
    except (OSError, ET.ParseError) as error:
        failures.append(f"cannot read package.xml: {error}")
        return

    dependency_tags = {
        "build_depend",
        "build_export_depend",
        "depend",
        "exec_depend",
    }
    dependencies = {
        element.text.strip()
        for element in package_root
        if element.tag in dependency_tags and element.text
    }
    forbidden = sorted(dependencies & CORE_FORBIDDEN_ROSDEP_KEYS)
    if forbidden:
        failures.append(f"robot-core package.xml contains developer dependencies: {forbidden}")
    missing = sorted(CORE_REQUIRED_ROSDEP_KEYS - dependencies)
    if missing:
        failures.append(f"robot-core package.xml is missing runtime dependencies: {missing}")

    apt_packages = load_development_apt_bundles(root, failures)
    for rosdep_key, apt_package in DEVELOPMENT_APT_COVERAGE.items():
        if apt_package not in apt_packages:
            failures.append(
                f"developer dependency {rosdep_key} is not covered by {apt_package}"
            )

    simulation_path = root / "dependencies" / "simulation.repos"
    try:
        simulation_sources = yaml.safe_load(
            simulation_path.read_text(encoding="utf-8")
        )["repositories"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        failures.append(f"cannot read dependencies/simulation.repos: {error}")
        simulation_sources = {}
    if "gz_ros2_control" not in simulation_sources:
        failures.append("developer dependency gz_ros2_control is missing from simulation.repos")


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

    production_marker_path = (
        root / "deployment" / "vmxpi-production-install-v1.json.in"
    )
    try:
        production_marker = json.loads(
            production_marker_path.read_text(encoding="utf-8")
        )
    except (OSError, json.JSONDecodeError) as error:
        failures.append(f"cannot read production install marker: {error}")
        production_marker = {}
    expected_marker_platform = {
        "os_id": platform.get("os_id"),
        "version_id": platform.get("version_id"),
        "architecture": platform.get("architecture"),
        "ros_distro": platform.get("ros_distro"),
    }
    if (
        production_marker.get("profile") != "vmxpi-production-install-v1"
        or production_marker.get("production_install") is not True
        or production_marker.get("hardware_profile") != "stack_4wd"
        or production_marker.get("platform") != expected_marker_platform
    ):
        failures.append("production install marker does not match the runtime profile")

    base_image = manifest.get("base_image")
    if base_image != {"family": "ubuntu-server", "install_mode": "minimal"}:
        failures.append("base_image must select the minimal Ubuntu Server base")

    if (
        manifest.get("external_runtime_dependencies")
        != EXPECTED_EXTERNAL_RUNTIME_DEPENDENCIES
    ):
        failures.append(
            "external_runtime_dependencies must declare the unmanaged VMXPi HAL SDK"
        )

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

    overlay_ros_packages = package_list(
        manifest.get("overlay_ros_packages"),
        "overlay_ros_packages",
        failures,
    )
    if set(overlay_ros_packages) != EXPECTED_OVERLAY_ROS_PACKAGES:
        failures.append(
            "overlay_ros_packages must contain only the qualified hardware overlay"
        )

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
    transitive_build_tools = manifest.get("documented_transitive_build_tools")
    if transitive_build_tools != EXPECTED_TRANSITIVE_BUILD_TOOLS:
        failures.append(
            "documented_transitive_build_tools must record the approved ROS Humble "
            "CMake dependency exception"
        )
        transitive_build_tools = {}
    direct_transitive_tools = sorted(set(enabled_apt) & set(transitive_build_tools))
    if direct_transitive_tools:
        failures.append(
            "transitive build tools must not be directly requested: "
            f"{direct_transitive_tools}"
        )
    prohibited_transitive_tools = sorted(
        set(prohibited) & set(transitive_build_tools)
    )
    if prohibited_transitive_tools:
        failures.append(
            "documented transitive build tools cannot also be prohibited: "
            f"{prohibited_transitive_tools}"
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

    validate_dependency_boundary(root, failures)

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
