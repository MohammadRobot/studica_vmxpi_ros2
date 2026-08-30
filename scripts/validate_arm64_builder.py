#!/usr/bin/env python3
"""Validate the isolated ARM64 release-builder contract without running Docker."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys
from typing import Any


SHA256_RE = re.compile(r"sha256:[0-9a-f]{64}")
BASE_IMAGE_REFERENCE = (
    "ubuntu:jammy-20260810@"
    "sha256:2edbbc5dc405e9612ba3584ce95480277e3eb374407b5505fe26f17df77c7dbc"
)
ROS_APT_SOURCE_VERSION = "1.2.0"
ROS_APT_SOURCE_SHA256 = (
    "767884cf4ed03116b9d64438930a832ed854147ae435279a7924dfdf60f94433"
)


def read_json(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return document


def require_text(
    text: str,
    required: tuple[str, ...],
    label: str,
    failures: list[str],
) -> None:
    for value in required:
        if value not in text:
            failures.append(f"{label} is missing required contract: {value}")


def validate_builder(root: Path, manifest: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    if manifest.get("schema_version") != 1:
        failures.append("builder schema_version must be 1")
    if manifest.get("profile") != "studica-arm64-builder-v1":
        failures.append("builder profile must be studica-arm64-builder-v1")
    if manifest.get("platform") != {
        "architecture": "arm64",
        "container_platform": "linux/arm64",
        "os_id": "ubuntu",
        "ros_distro": "humble",
        "version_id": "22.04",
    }:
        failures.append("builder platform must select Ubuntu 22.04/Humble ARM64")

    base_image = manifest.get("base_image")
    if not isinstance(base_image, dict):
        failures.append("base_image must be an object")
        base_image = {}
    if base_image.get("reference") != BASE_IMAGE_REFERENCE:
        failures.append("builder base image reference must use the approved immutable digest")
    digest = str(base_image.get("digest", ""))
    if SHA256_RE.fullmatch(digest) is None or not BASE_IMAGE_REFERENCE.endswith(digest):
        failures.append("builder base image digest is invalid or inconsistent")
    if base_image.get("tag") != "jammy-20260810":
        failures.append("builder base image must use the dated Jammy tag")

    if manifest.get("ros_apt_source") != {
        "sha256": ROS_APT_SOURCE_SHA256,
        "version": ROS_APT_SOURCE_VERSION,
    }:
        failures.append("builder ROS APT bootstrap must match the approved package")
    if manifest.get("container") != {
        "dockerfile": "deployment/arm64-builder.Dockerfile",
        "dockerignore": "deployment/arm64-builder.Dockerfile.dockerignore",
        "build_target": "build-env",
        "runtime_inventory_target": "runtime-inventory",
        "source_preparation_entrypoint": (
            "/usr/local/bin/prepare-studica-arm64-sources"
        ),
    }:
        failures.append("builder container targets differ from the v1 contract")
    if manifest.get("execution") != {
        "emulation_requires_opt_in": True,
        "native_arm64_default": True,
        "network_required_for_image_and_source_preparation": True,
        "source_preparation_without_sdk": True,
        "sdk_enabled_build_network": "none",
        "sdk_mounts_read_only": [
            "/usr/local/include/vmxpi",
            "/usr/local/lib/vmxpi",
        ],
        "source_mount_read_only": True,
    }:
        failures.append("builder execution isolation differs from the v1 contract")
    if manifest.get("artifact_policy") != {
        "activation_authorized": False,
        "channel": "development",
    }:
        failures.append("builder must emit development-only, non-activatable artifacts")

    reproducibility = manifest.get("reproducibility")
    if reproducibility != {
        "apt_repository_snapshot": False,
        "base_image_pinned": True,
        "exact_installed_package_inventory": True,
        "source_commits_pinned": True,
    }:
        failures.append("builder reproducibility declaration differs from v1")

    dockerfile_path = root / "deployment/arm64-builder.Dockerfile"
    dockerignore_path = root / "deployment/arm64-builder.Dockerfile.dockerignore"
    host_script_path = root / "scripts/build_arm64_release.sh"
    prepare_script_path = root / "deployment/prepare_arm64_release_sources.sh"
    container_script_path = root / "deployment/build_arm64_release_in_container.sh"
    verifier_path = root / "scripts/verify_hardware_checkout.py"
    artifact_verifier_path = root / "scripts/verify_release_artifacts.py"
    release_builder_path = root / "scripts/build_release_bundle.py"
    workflow_path = root / ".github/workflows/arm64-development-release.yml"
    try:
        dockerfile = dockerfile_path.read_text(encoding="utf-8")
        dockerignore = dockerignore_path.read_text(encoding="utf-8")
        host_script = host_script_path.read_text(encoding="utf-8")
        prepare_script = prepare_script_path.read_text(encoding="utf-8")
        container_script = container_script_path.read_text(encoding="utf-8")
        artifact_verifier = artifact_verifier_path.read_text(encoding="utf-8")
        release_builder = release_builder_path.read_text(encoding="utf-8")
        workflow = workflow_path.read_text(encoding="utf-8")
    except OSError as error:
        failures.append(f"cannot read builder implementation: {error}")
        return failures

    require_text(
        dockerfile,
        (
            f"ARG UBUNTU_BASE_IMAGE={BASE_IMAGE_REFERENCE}",
            "FROM --platform=linux/arm64 ${UBUNTU_BASE_IMAGE} AS ros-apt-base",
            "AS build-env",
            "AS runtime-inventory",
            f"--checksum=sha256:{ROS_APT_SOURCE_SHA256}",
            f"/download/{ROS_APT_SOURCE_VERSION}/ros2-apt-source_{ROS_APT_SOURCE_VERSION}.jammy_all.deb",
            "rosdep update --rosdistro humble",
            "cp -a /root/.ros/rosdep/sources.cache/.",
            "--sources-cache-dir /opt/studica/rosdep-cache",
            "/usr/local/bin/prepare-studica-arm64-sources",
        ),
        "ARM64 Dockerfile",
        failures,
    )
    if re.search(
        r"rosdep\s+--sources-cache-dir\s+/opt/studica/rosdep-cache\s+"
        r"update\b",
        dockerfile,
    ):
        failures.append(
            "ARM64 Dockerfile must populate the portable rosdep cache from the "
            "default update directory"
        )
    require_text(
        dockerignore,
        (
            "**",
            "!deployment/build_arm64_release_in_container.sh",
            "!deployment/prepare_arm64_release_sources.sh",
            "!deployment/vmxpi-runtime-packages-v1.json",
            "!dependencies/apt/development-core.txt",
            "!dependencies/hardware.repos",
            "!package.xml",
        ),
        "ARM64 Docker build-context filter",
        failures,
    )
    require_text(
        host_script,
        (
            "--allow-emulation",
            "--read-only",
            "--cap-drop ALL",
            "--security-opt no-new-privileges",
            "dst=/source,readonly",
            "dst=/usr/local/include/vmxpi,readonly",
            "dst=/usr/local/lib/vmxpi,readonly",
            "source repository is dirty",
            "VMXPi SDK root must be outside the Git repository",
            "filesystem root cannot be the output directory",
            "output directory must not overlap the VMXPi SDK root",
            "output directory must not contain the VMXPi SDK root",
            "validate_arm64_builder.py",
            "--entrypoint /usr/local/bin/prepare-studica-arm64-sources",
            "--network none",
            "src=${prepared_sources},dst=/prepared,readonly",
        ),
        "host builder",
        failures,
    )
    require_text(
        prepare_script,
        (
            'readonly source_mount="/source"',
            'readonly prepared_root="/prepared"',
            "vcs import --recursive",
            "verify_hardware_checkout.py",
            'rev-parse HEAD > "${prepared_root}/source-commit"',
        ),
        "networked source-preparation stage",
        failures,
    )
    for sdk_path in ("/usr/local/include/vmxpi", "/usr/local/lib/vmxpi"):
        if sdk_path in prepare_script:
            failures.append(
                f"source-preparation stage must not reference the SDK mount: {sdk_path}"
            )
    require_text(
        container_script,
        (
            'readonly prepared_root="/prepared"',
            "verify_hardware_checkout.py",
            "--sources-cache-dir /opt/studica/rosdep-cache",
            "-DSWIG_FOUND=FALSE",
            "-DPYTHONLIBS_FOUND=FALSE",
            "-DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE",
            "-DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE",
            "-DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=TRUE",
            "-DSTUDICA_PRODUCTION_INSTALL=ON",
            "--merge-install",
            "--builder-image-id",
            "build_release_bundle.py",
        ),
        "container builder",
        failures,
    )
    for network_operation in ("vcs import", "git clone", "curl ", "wget "):
        if network_operation in container_script:
            failures.append(
                "offline SDK-enabled builder contains a network/source acquisition "
                f"operation: {network_operation}"
            )
    require_text(
        release_builder,
        (
            "deployment/arm64-builder-v1.json",
            "builder_image_id: str",
            '"arm64_builder_profile_sha256"',
        ),
        "release bundle builder",
        failures,
    )
    require_text(
        artifact_verifier,
        (
            'metadata.get("activation_authorized") is not False',
            'rollback.get("activation_authorized") is not False',
            "external archive checksum does not match",
            "internal checksum mismatch",
            "release input hash mismatch",
            "expected_commit",
        ),
        "release artifact verifier",
        failures,
    )
    require_text(
        workflow,
        (
            "workflow_dispatch:",
            "approved_commit:",
            "environment: arm64-development-release",
            "runs-on: [self-hosted, linux, ARM64, studica-arm64-release]",
            "persist-credentials: false",
            "actions/checkout@fbc6f3992d24b796d5a048ff273f7fcc4a7b6c09",
            "actions/upload-artifact@043fb46d1a93c77aae656e7c1c64a875d1fc6a0a",
            "--check-only",
            "verify_release_artifacts.py",
            "Activation remains blocked.",
        ),
        "manual ARM64 release workflow",
        failures,
    )
    for automatic_trigger in ("pull_request:", "schedule:"):
        if automatic_trigger in workflow:
            failures.append(
                f"ARM64 SDK workflow must not have automatic trigger: {automatic_trigger}"
            )
    sdk_mount_position = host_script.find("dst=/usr/local/include/vmxpi,readonly")
    offline_position = host_script.rfind("--network none", 0, sdk_mount_position)
    if sdk_mount_position < 0 or offline_position < 0:
        failures.append("VMXPi SDK mount must occur only in a network-disabled container")
    for forbidden in ("192.168.1.173", "ssh ", "git pull", "/opt/studica/current"):
        if any(
            forbidden in text
            for text in (host_script, prepare_script, container_script, workflow)
        ):
            failures.append(f"builder must not contain live-robot action: {forbidden}")
    for path in (
        host_script_path,
        prepare_script_path,
        container_script_path,
        verifier_path,
        artifact_verifier_path,
    ):
        if not path.is_file() or not path.stat().st_mode & 0o111:
            failures.append(f"builder executable is missing or not executable: {path.name}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    root = parser.parse_args().root.resolve()
    try:
        manifest = read_json(root / "deployment/arm64-builder-v1.json")
        failures = validate_builder(root, manifest)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        failures = [str(error)]
    if failures:
        print("ARM64 builder contract check failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("[check] ARM64 builder: pinned base, isolated mounts, activation blocked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
