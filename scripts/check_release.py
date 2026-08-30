#!/usr/bin/env python3
"""Validate offline release reproducibility and CI supply-chain pins."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import yaml


SHA256_RE = re.compile(r"[0-9a-f]{64}")
GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")
SEMVER_RE = re.compile(
    r"(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)"
)
MANIFEST_NAMES = ("simulation.repos", "hardware.repos")
FIRST_PARTY = (
    "studica_drivers",
    "studica_robot_monitor",
    "studica_ros2_control",
)


def fail_if(condition: bool, message: str, failures: list[str]) -> None:
    if condition:
        failures.append(message)


def load_manifests(root: Path, failures: list[str]) -> dict[str, dict[str, str]]:
    manifests: dict[str, dict[str, str]] = {}
    for name in MANIFEST_NAMES:
        path = root / "dependencies" / name
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
            repositories = document["repositories"]
        except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
            failures.append(f"cannot read {path.relative_to(root)}: {error}")
            continue

        versions: dict[str, str] = {}
        for repository, metadata in repositories.items():
            version = str(metadata.get("version", ""))
            versions[repository] = version
            fail_if(
                GIT_SHA_RE.fullmatch(version) is None,
                f"{name}: {repository} must use an immutable 40-character commit",
                failures,
            )
        manifests[name] = versions
    return manifests


def check_cross_manifest_pins(
    manifests: dict[str, dict[str, str]], failures: list[str]
) -> None:
    simulation = manifests.get("simulation.repos", {})
    hardware = manifests.get("hardware.repos", {})
    for repository in sorted(simulation.keys() & hardware.keys()):
        fail_if(
            simulation[repository] != hardware[repository],
            f"{repository} uses different commits in simulation and hardware manifests",
            failures,
        )


def check_ci_pins(
    root: Path, manifests: dict[str, dict[str, str]], failures: list[str]
) -> None:
    workflow_path = root / ".github" / "workflows" / "ros-ci.yml"
    try:
        workflow = workflow_path.read_text(encoding="utf-8")
    except OSError as error:
        failures.append(f"cannot read ROS CI workflow: {error}")
        return

    workflow_dir = root / ".github" / "workflows"
    workflow_paths = sorted(workflow_dir.glob("*.yml")) + sorted(
        workflow_dir.glob("*.yaml")
    )
    for checked_path in workflow_paths:
        checked_workflow = checked_path.read_text(encoding="utf-8")
        for line_number, line in enumerate(checked_workflow.splitlines(), start=1):
            stripped = line.strip()
            if not stripped.startswith("uses:"):
                continue
            action = stripped.removeprefix("uses:").strip().split()[0]
            if action.startswith("./"):
                continue
            reference = action.rsplit("@", maxsplit=1)[-1]
            fail_if(
                GIT_SHA_RE.fullmatch(reference) is None,
                f"{checked_path.name}:{line_number}: action must be pinned to a full commit",
                failures,
            )

    hardware = manifests.get("hardware.repos", {})
    for repository in FIRST_PARTY:
        version = hardware.get(repository)
        fail_if(version is None, f"hardware manifest is missing {repository}", failures)
        if version is None:
            continue
        checkout = re.compile(
            rf"repository:\s*MohammadRobot/{re.escape(repository)}\s+ref:\s*{version}",
            re.MULTILINE,
        )
        fail_if(
            checkout.search(workflow) is None,
            f"ROS CI does not checkout the locked {repository} commit",
            failures,
        )


def check_ros_apt_source(root: Path, failures: list[str]) -> None:
    setup = (root / "scripts" / "setup_ubuntu.sh").read_text(encoding="utf-8")
    workflow = (root / ".github" / "workflows" / "ros-ci.yml").read_text(
        encoding="utf-8"
    )
    version_match = re.search(r'ROS_APT_SOURCE_VERSION="([^"]+)"', setup)
    digest_match = re.search(r'ROS_APT_SOURCE_SHA256="([^"]+)"', setup)
    fail_if(version_match is None, "installer ROS APT source version is missing", failures)
    fail_if(
        digest_match is None or SHA256_RE.fullmatch(digest_match.group(1)) is None,
        "installer ROS APT source SHA-256 is missing or invalid",
        failures,
    )
    if version_match is not None:
        fail_if(
            version_match.group(1) not in workflow,
            "ROS CI and installer use different ROS APT source versions",
            failures,
        )
    if digest_match is not None:
        fail_if(
            digest_match.group(1) not in workflow,
            "ROS CI and installer use different ROS APT source digests",
            failures,
        )


def check_package_version(root: Path, failures: list[str]) -> None:
    try:
        version = ET.parse(root / "package.xml").findtext("version", default="")
    except (OSError, ET.ParseError) as error:
        failures.append(f"cannot read package.xml version: {error}")
        return
    fail_if(
        SEMVER_RE.fullmatch(version) is None,
        f"package.xml version is not MAJOR.MINOR.PATCH: {version!r}",
        failures,
    )
    changelog = (root / "CHANGELOG.md").read_text(encoding="utf-8")
    fail_if(
        re.search(rf"^## {re.escape(version)}(?:\s|$)", changelog, re.MULTILINE) is None,
        f"CHANGELOG.md has no section for package version {version}",
        failures,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    root = parser.parse_args().root.resolve()
    failures: list[str] = []

    manifests = load_manifests(root, failures)
    check_cross_manifest_pins(manifests, failures)
    check_ci_pins(root, manifests, failures)
    check_ros_apt_source(root, failures)
    check_package_version(root, failures)

    if failures:
        print("Release contract check failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("[check] Release contract: immutable sources, CI actions, and package version")
    return 0


if __name__ == "__main__":
    sys.exit(main())
