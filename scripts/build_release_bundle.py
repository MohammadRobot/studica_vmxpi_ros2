#!/usr/bin/env python3
"""Build a deterministic, development-only VMXPi ARM64 release bundle."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tarfile
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import yaml


PRODUCT = "studica-robot"
PROFILE_NAME = "vmxpi-runtime-packages-v1"
PRODUCTION_INSTALL_PROFILE = "vmxpi-production-install-v1"
BUILDER_PROFILE_NAME = "studica-arm64-builder-v1"
GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")
IMAGE_ID_RE = re.compile(r"sha256:[0-9a-f]{64}")
RELEASE_VERSION_RE = re.compile(r"[0-9A-Za-z][0-9A-Za-z.+-]*")
AARCH64_ELF_MACHINE = 183
REQUIRED_ARM64_FILES = (
    "lib/libstudica_drivers.so",
    "lib/libstudica_vmxpi_ros2.so",
    "lib/orbbec_camera/orbbec_camera_node",
    "lib/studica_vmxpi_ros2/safety_input_check",
    "lib/studica_vmxpi_ros2/safety_supervisor_node",
    "lib/studica_vmxpi_ros2/topic_adapter_node",
    "lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node",
)
VMXPI_SDK_PROFILE = "studica-vmxpi-hal-cpp-content-v1"
VMXPI_SDK_HEADER = "include/vmxpi/VMXPi.h"
VMXPI_SDK_LIBRARY = "lib/vmxpi/libvmxpi_hal_cpp.so"
VMXPI_EXTERNAL_DEPENDENCY = {
    "studica_vmxpi_hal_cpp": {
        "build_headers": "/usr/local/include/vmxpi",
        "identity": "content-sha256",
        "license": "NOASSERTION",
        "package_manager": "unmanaged",
        "runtime_library": "/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so",
        "supplier": "Studica",
    }
}
FORBIDDEN_INSTALL_PATHS = (
    "share/studica_vmxpi_ros2/config/slam_toolbox_hardware_mapper_params.yaml",
    "share/studica_vmxpi_ros2/config/slam_toolbox_mapper_params.yaml",
    "share/studica_vmxpi_ros2/config/waypoints",
    "share/studica_vmxpi_ros2/dependencies",
    "share/studica_vmxpi_ros2/description/gz",
    "share/studica_vmxpi_ros2/description/robot/rviz",
    "share/studica_vmxpi_ros2/docs",
    "share/studica_vmxpi_ros2/examples",
    "share/studica_vmxpi_ros2/foxglove",
    "share/studica_vmxpi_ros2/launch/mapping.launch.py",
    "share/studica_vmxpi_ros2/launch/navigation.launch.py",
    "share/studica_vmxpi_ros2/launch/sim.launch.py",
    "share/studica_vmxpi_ros2/maps",
)
FORBIDDEN_TREE_NAMES = {
    ".git",
    ".pytest_cache",
    "__pycache__",
    "authorized_keys",
    "id_ed25519",
    "id_rsa",
}


class BundleError(RuntimeError):
    """A release input violates the production bundle contract."""


@dataclass(frozen=True, order=True)
class DebianPackage:
    """One exact package from the target root filesystem inventory."""

    name: str
    version: str
    architecture: str


def read_json(path: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise BundleError(f"{path} must contain a JSON object")
    return document


def write_json(path: Path, document: Any) -> None:
    path.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def package_version(package_xml: Path) -> str:
    try:
        version = ET.parse(package_xml).findtext("version", default="").strip()
    except (OSError, ET.ParseError) as error:
        raise BundleError(f"cannot read {package_xml}: {error}") from error
    if not re.fullmatch(r"(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)", version):
        raise BundleError(f"invalid package version in {package_xml}: {version!r}")
    return version


def parse_dpkg_inventory(path: Path) -> list[DebianPackage]:
    packages: list[DebianPackage] = []
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise BundleError(f"cannot read Debian inventory {path}: {error}") from error
    for line_number, line in enumerate(lines, start=1):
        if not line or line.startswith("#"):
            continue
        fields = line.split("\t")
        if len(fields) != 3 or any(not field for field in fields):
            raise BundleError(
                f"{path}:{line_number} must be PACKAGE<TAB>VERSION<TAB>ARCHITECTURE"
            )
        packages.append(DebianPackage(*fields))
    if not packages:
        raise BundleError("Debian inventory is empty")
    if packages != sorted(packages):
        raise BundleError("Debian inventory must be sorted by package, version, architecture")
    names = [package.name for package in packages]
    duplicates = sorted(name for name in set(names) if names.count(name) > 1)
    if duplicates:
        raise BundleError(f"Debian inventory contains duplicate packages: {duplicates}")
    invalid_architectures = sorted(
        {package.architecture for package in packages} - {"all", "arm64"}
    )
    if invalid_architectures:
        raise BundleError(
            f"Debian inventory is not ARM64-only: {invalid_architectures}"
        )
    return packages


def enabled_apt_packages(profile: dict[str, Any]) -> set[str]:
    packages: set[str] = set()
    for group in profile["required_apt_packages"].values():
        packages.update(group)
    for feature in profile["features"].values():
        if feature["enabled"]:
            packages.update(feature["apt_packages"])
    return packages


def validate_dpkg_inventory(
    packages: list[DebianPackage], profile: dict[str, Any]
) -> None:
    installed = {package.name for package in packages}
    missing = sorted(enabled_apt_packages(profile) - installed)
    if missing:
        raise BundleError(f"Debian inventory is missing enabled runtime packages: {missing}")

    prohibited = set(profile["prohibited_apt_packages"])
    prohibited_prefixes = tuple(profile["prohibited_apt_prefixes"])
    exact_hits = sorted(installed & prohibited)
    prefix_hits = sorted(
        package
        for package in installed
        if any(package.startswith(prefix) for prefix in prohibited_prefixes)
    )
    if exact_hits or prefix_hits:
        raise BundleError(
            "Debian inventory contains non-production packages: "
            f"{sorted(set(exact_hits + prefix_hits))}"
        )


def elf_machine(path: Path) -> int:
    try:
        header = path.read_bytes()[:20]
    except OSError as error:
        raise BundleError(f"cannot read required binary {path}: {error}") from error
    if len(header) < 20 or header[:4] != b"\x7fELF" or header[4] != 2:
        raise BundleError(f"required binary is not a 64-bit ELF file: {path}")
    if header[5] == 1:
        return struct.unpack_from("<H", header, 18)[0]
    if header[5] == 2:
        return struct.unpack_from(">H", header, 18)[0]
    raise BundleError(f"required binary has an invalid ELF byte order: {path}")


def is_elf(path: Path) -> bool:
    try:
        with path.open("rb") as stream:
            return stream.read(4) == b"\x7fELF"
    except OSError as error:
        raise BundleError(f"cannot inspect binary {path}: {error}") from error


def vmxpi_sdk_inventory(sdk_root: Path) -> dict[str, Any]:
    sdk_root = sdk_root.resolve()
    header_root = sdk_root / "include/vmxpi"
    library = sdk_root / VMXPI_SDK_LIBRARY
    required_header = sdk_root / VMXPI_SDK_HEADER
    if not header_root.is_dir() or not required_header.is_file():
        raise BundleError(
            f"VMXPi SDK headers are missing beneath {header_root}"
        )
    if not library.is_file():
        raise BundleError(f"VMXPi SDK runtime library is missing: {library}")
    if elf_machine(library.resolve()) != AARCH64_ELF_MACHINE:
        raise BundleError("VMXPi SDK runtime library is not AArch64")

    selected_files = sorted(
        [path for path in header_root.rglob("*") if path.is_file()] + [library]
    )
    files: list[dict[str, Any]] = []
    identity_lines: list[str] = []
    for path in selected_files:
        relative = path.relative_to(sdk_root).as_posix()
        try:
            path.resolve(strict=True).relative_to(sdk_root)
        except (OSError, ValueError) as error:
            raise BundleError(f"VMXPi SDK path escapes its root: {relative}") from error
        if path.stat().st_mode & 0o002:
            raise BundleError(f"VMXPi SDK file is world-writable: {relative}")
        digest = sha256_file(path)
        size = path.stat().st_size
        files.append({"path": relative, "sha256": digest, "bytes": size})
        identity_lines.append(f"{digest}  {relative}\n")

    content_sha256 = hashlib.sha256(
        "".join(identity_lines).encode("utf-8")
    ).hexdigest()
    return {
        "schema_version": 1,
        "profile": VMXPI_SDK_PROFILE,
        "package_name": "studica-vmxpi-hal-cpp",
        "supplier": "Studica",
        "package_manager": "unmanaged",
        "license": "NOASSERTION",
        "platform": {"architecture": "arm64", "elf_machine": "AArch64"},
        "content_identity": f"sha256:{content_sha256}",
        "runtime_library": VMXPI_SDK_LIBRARY,
        "build_headers": "include/vmxpi",
        "files": files,
    }


def validate_install_tree(
    install_prefix: Path,
    profile: dict[str, Any],
    expected_package_version: str,
) -> list[str]:
    install_prefix = install_prefix.resolve()
    if not install_prefix.is_dir():
        raise BundleError(f"merged install prefix does not exist: {install_prefix}")
    if not install_prefix.joinpath("setup.bash").is_file():
        raise BundleError("install prefix is not a merged Colcon install: setup.bash is missing")

    marker_path = install_prefix / (
        "share/studica_vmxpi_ros2/deployment/vmxpi-production-install-v1.json"
    )
    try:
        marker = read_json(marker_path)
    except (OSError, json.JSONDecodeError) as error:
        raise BundleError(
            "install prefix was not built with -DSTUDICA_PRODUCTION_INSTALL=ON"
        ) from error
    if (
        marker.get("profile") != PRODUCTION_INSTALL_PROFILE
        or marker.get("production_install") is not True
        or marker.get("platform") != {
            "os_id": "ubuntu",
            "version_id": "22.04",
            "architecture": "arm64",
            "ros_distro": "humble",
        }
    ):
        raise BundleError("production install marker does not match the selected platform")

    discovered_packages = sorted(
        path.parent.name for path in install_prefix.glob("share/*/package.xml")
    )
    expected_packages = profile.get("overlay_ros_packages")
    if discovered_packages != expected_packages:
        raise BundleError(
            "merged install contains the wrong ROS package set; "
            f"expected {expected_packages}, found {discovered_packages}"
        )

    installed_package_xml = (
        install_prefix / "share/studica_vmxpi_ros2/package.xml"
    )
    installed_version = package_version(installed_package_xml)
    if installed_version != expected_package_version:
        raise BundleError(
            "source and installed studica_vmxpi_ros2 versions differ: "
            f"{expected_package_version} != {installed_version}"
        )

    for relative_path in REQUIRED_ARM64_FILES:
        binary = install_prefix / relative_path
        if not binary.is_file():
            raise BundleError(f"required hardware artifact is missing: {relative_path}")
        if not relative_path.endswith(".so") and not binary.stat().st_mode & 0o111:
            raise BundleError(f"required hardware executable is not executable: {relative_path}")
        if elf_machine(binary.resolve()) != AARCH64_ELF_MACHINE:
            raise BundleError(f"required hardware artifact is not AArch64: {relative_path}")

    for relative_path in FORBIDDEN_INSTALL_PATHS:
        if install_prefix.joinpath(relative_path).exists():
            raise BundleError(f"developer asset leaked into production install: {relative_path}")

    for path in sorted(install_prefix.rglob("*")):
        relative = path.relative_to(install_prefix)
        if any(part in FORBIDDEN_TREE_NAMES for part in relative.parts):
            raise BundleError(f"forbidden generated or credential path: {relative}")
        if path.suffix == ".pyc":
            raise BundleError(f"Python cache leaked into production install: {relative}")
        if path.is_symlink():
            try:
                path.resolve(strict=True).relative_to(install_prefix)
            except (OSError, ValueError) as error:
                raise BundleError(f"symlink escapes the install prefix: {relative}") from error
        if path.is_file():
            if path.stat().st_mode & 0o002:
                raise BundleError(f"world-writable file in production install: {relative}")
            if is_elf(path) and elf_machine(path.resolve()) != AARCH64_ELF_MACHINE:
                raise BundleError(f"non-AArch64 ELF leaked into production install: {relative}")
    return discovered_packages


def prune_build_only_files(install_prefix: Path) -> list[str]:
    removed: list[str] = []
    directory_targets = [
        install_prefix / "include",
        install_prefix / "share/ydlidar_sdk",
    ]
    directory_targets.extend(sorted(install_prefix.glob("share/*/cmake")))
    for path in directory_targets:
        if not path.exists():
            continue
        removed.append(path.relative_to(install_prefix).as_posix())
        if path.is_symlink():
            path.unlink()
        else:
            shutil.rmtree(path)

    for pattern in ("lib/**/*.a", "lib/**/*.la"):
        for path in sorted(install_prefix.glob(pattern)):
            if not path.is_file():
                continue
            removed.append(path.relative_to(install_prefix).as_posix())
            path.unlink()
    return sorted(set(removed))


def tree_stats(root: Path) -> tuple[int, int]:
    files = [path for path in root.rglob("*") if path.is_file()]
    return len(files), sum(path.stat().st_size for path in files)


def hardware_repositories(source_root: Path) -> dict[str, dict[str, str]]:
    path = source_root / "dependencies" / "hardware.repos"
    try:
        repositories = yaml.safe_load(path.read_text(encoding="utf-8"))["repositories"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise BundleError(f"cannot read {path}: {error}") from error
    for name, metadata in repositories.items():
        if GIT_SHA_RE.fullmatch(str(metadata.get("version", ""))) is None:
            raise BundleError(f"source repository {name} is not pinned to a full commit")
        if not isinstance(metadata.get("url"), str) or not metadata["url"].startswith(
            "https://"
        ):
            raise BundleError(f"source repository {name} has an invalid HTTPS URL")
    return repositories


def spdx_id(prefix: str, name: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9.-]", "-", name)
    return f"SPDXRef-{prefix}-{normalized}"


def build_spdx_sbom(
    release_version: str,
    application_version: str,
    commit: str,
    epoch: int,
    debian_packages: list[DebianPackage],
    repositories: dict[str, dict[str, str]],
    sdk_inventory: dict[str, Any],
) -> dict[str, Any]:
    product_id = spdx_id("Product", PRODUCT)
    packages: list[dict[str, Any]] = [
        {
            "SPDXID": product_id,
            "name": PRODUCT,
            "versionInfo": release_version,
            "downloadLocation": (
                "git+https://github.com/MohammadRobot/studica_vmxpi_ros2.git@"
                f"{commit}"
            ),
            "filesAnalyzed": False,
            "licenseConcluded": "Apache-2.0",
            "licenseDeclared": "Apache-2.0",
            "copyrightText": "NOASSERTION",
            "comment": f"ROS package version {application_version}",
        }
    ]
    relationships: list[dict[str, str]] = [
        {
            "spdxElementId": "SPDXRef-DOCUMENT",
            "relationshipType": "DESCRIBES",
            "relatedSpdxElement": product_id,
        }
    ]
    sdk_id = spdx_id("Vendor", sdk_inventory["package_name"])
    packages.append(
        {
            "SPDXID": sdk_id,
            "name": sdk_inventory["package_name"],
            "versionInfo": sdk_inventory["content_identity"],
            "downloadLocation": "NOASSERTION",
            "filesAnalyzed": False,
            "licenseConcluded": "NOASSERTION",
            "licenseDeclared": sdk_inventory["license"],
            "copyrightText": "NOASSERTION",
            "supplier": f"Organization: {sdk_inventory['supplier']}",
            "comment": (
                "Unmanaged VMXPi vendor SDK; content-addressed because the "
                "installed SDK does not expose a package version."
            ),
        }
    )
    relationships.append(
        {
            "spdxElementId": product_id,
            "relationshipType": "DEPENDS_ON",
            "relatedSpdxElement": sdk_id,
        }
    )
    for package in debian_packages:
        package_id = spdx_id("Deb", package.name)
        packages.append(
            {
                "SPDXID": package_id,
                "name": package.name,
                "versionInfo": package.version,
                "downloadLocation": "NOASSERTION",
                "filesAnalyzed": False,
                "licenseConcluded": "NOASSERTION",
                "licenseDeclared": "NOASSERTION",
                "copyrightText": "NOASSERTION",
                "supplier": "Organization: Ubuntu or ROS package repository",
                "comment": f"Debian architecture {package.architecture}",
            }
        )
        relationships.append(
            {
                "spdxElementId": product_id,
                "relationshipType": "DEPENDS_ON",
                "relatedSpdxElement": package_id,
            }
        )
    for name, metadata in sorted(repositories.items()):
        package_id = spdx_id("Source", name)
        packages.append(
            {
                "SPDXID": package_id,
                "name": name,
                "versionInfo": str(metadata["version"]),
                "downloadLocation": f"git+{metadata['url']}@{metadata['version']}",
                "filesAnalyzed": False,
                "licenseConcluded": "NOASSERTION",
                "licenseDeclared": "NOASSERTION",
                "copyrightText": "NOASSERTION",
            }
        )
        relationships.append(
            {
                "spdxElementId": product_id,
                "relationshipType": "DEPENDS_ON",
                "relatedSpdxElement": package_id,
            }
        )

    created = datetime.fromtimestamp(epoch, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    return {
        "spdxVersion": "SPDX-2.3",
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"{PRODUCT}-{release_version}",
        "documentNamespace": (
            "https://github.com/MohammadRobot/studica_vmxpi_ros2/spdx/"
            f"{release_version}/{commit}"
        ),
        "creationInfo": {
            "created": created,
            "creators": ["Tool: studica-build-release-bundle-v1"],
        },
        "packages": packages,
        "relationships": relationships,
    }


def write_payload_checksums(release_root: Path) -> None:
    checksum_path = release_root / "SHA256SUMS"
    lines = []
    for path in sorted(release_root.rglob("*")):
        if not path.is_file() or path == checksum_path:
            continue
        relative = path.relative_to(release_root).as_posix()
        lines.append(f"{sha256_file(path)}  {relative}")
    checksum_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def add_deterministic_tree(
    archive: tarfile.TarFile, payload_root: Path, epoch: int
) -> None:
    paths = [payload_root, *sorted(payload_root.rglob("*"))]
    for path in paths:
        arcname = "." if path == payload_root else path.relative_to(payload_root).as_posix()
        info = archive.gettarinfo(str(path), arcname=arcname)
        info.uid = 0
        info.gid = 0
        info.uname = "root"
        info.gname = "root"
        info.mtime = epoch
        info.pax_headers = {}
        if info.isfile():
            with path.open("rb") as stream:
                archive.addfile(info, stream)
        else:
            archive.addfile(info)


def create_deterministic_archive(
    payload_root: Path, archive_path: Path, epoch: int
) -> None:
    with archive_path.open("wb") as raw_stream:
        with gzip.GzipFile(
            filename="",
            mode="wb",
            compresslevel=9,
            fileobj=raw_stream,
            mtime=epoch,
        ) as compressed_stream:
            with tarfile.open(
                fileobj=compressed_stream,
                mode="w",
                format=tarfile.GNU_FORMAT,
            ) as archive:
                add_deterministic_tree(archive, payload_root, epoch)


def build_release_bundle(
    source_root: Path,
    install_prefix: Path,
    vmxpi_sdk_root: Path,
    dpkg_inventory_path: Path,
    output_dir: Path,
    commit: str,
    epoch: int,
    builder_image_id: str,
    release_version: str | None = None,
) -> tuple[Path, Path]:
    source_root = source_root.resolve()
    install_prefix = install_prefix.resolve()
    vmxpi_sdk_root = vmxpi_sdk_root.resolve()
    output_dir = output_dir.resolve()
    if GIT_SHA_RE.fullmatch(commit) is None:
        raise BundleError("source commit must be a full 40-character Git SHA")
    if IMAGE_ID_RE.fullmatch(builder_image_id) is None:
        raise BundleError("builder image ID must be a full sha256 digest")
    if epoch < 0:
        raise BundleError("SOURCE_DATE_EPOCH must not be negative")

    profile_path = source_root / "deployment" / "vmxpi-runtime-packages-v1.json"
    profile = read_json(profile_path)
    if profile.get("profile") != PROFILE_NAME:
        raise BundleError(f"unexpected runtime package profile: {profile.get('profile')}")
    if profile.get("platform") != {
        "os_id": "ubuntu",
        "version_id": "22.04",
        "architecture": "arm64",
        "ros_distro": "humble",
        "ros_variant": "ros-base",
    }:
        raise BundleError("runtime package profile does not select Ubuntu 22.04/Humble ARM64")
    if profile.get("external_runtime_dependencies") != VMXPI_EXTERNAL_DEPENDENCY:
        raise BundleError("runtime package profile does not pin the VMXPi vendor SDK contract")
    builder_profile_path = source_root / "deployment/arm64-builder-v1.json"
    builder_profile = read_json(builder_profile_path)
    if (
        builder_profile.get("profile") != BUILDER_PROFILE_NAME
        or builder_profile.get("platform") != {
            "architecture": "arm64",
            "container_platform": "linux/arm64",
            "os_id": "ubuntu",
            "ros_distro": "humble",
            "version_id": "22.04",
        }
        or builder_profile.get("artifact_policy")
        != {"activation_authorized": False, "channel": "development"}
    ):
        raise BundleError("ARM64 builder profile does not match the release contract")
    application_version = package_version(source_root / "package.xml")
    release_version = release_version or f"{application_version}-dev.{commit[:12]}"
    if (
        RELEASE_VERSION_RE.fullmatch(release_version) is None
        or not release_version.startswith(f"{application_version}-")
    ):
        raise BundleError(
            "development release version must start with the ROS package version "
            f"and a hyphen: {application_version}-..."
        )

    debian_packages = parse_dpkg_inventory(dpkg_inventory_path)
    validate_dpkg_inventory(debian_packages, profile)
    overlay_packages = validate_install_tree(
        install_prefix,
        profile,
        application_version,
    )
    try:
        vmxpi_sdk_root.relative_to(source_root)
    except ValueError:
        pass
    else:
        raise BundleError("VMXPi vendor SDK root must be outside the Git source tree")
    for parent, child in (
        (vmxpi_sdk_root, output_dir),
        (output_dir, vmxpi_sdk_root),
    ):
        try:
            child.relative_to(parent)
        except ValueError:
            continue
        raise BundleError("release output and VMXPi SDK root must not overlap")
    sdk_inventory = vmxpi_sdk_inventory(vmxpi_sdk_root)
    repositories = hardware_repositories(source_root)

    artifact_name = (
        f"{PRODUCT}-{release_version}-ubuntu22.04-humble-arm64.tar.gz"
    )
    checksum_name = f"{artifact_name}.sha256"
    try:
        output_dir.relative_to(source_root)
    except ValueError:
        pass
    else:
        raise BundleError("release output directory must be outside the Git source tree")
    output_dir.mkdir(parents=True, exist_ok=True)
    final_archive = output_dir / artifact_name
    final_checksum = output_dir / checksum_name
    if final_archive.exists() or final_checksum.exists():
        raise BundleError(f"release artifact already exists: {final_archive}")

    with tempfile.TemporaryDirectory(prefix=".studica-release-", dir=output_dir) as temporary:
        temporary_root = Path(temporary)
        payload_root = temporary_root / "payload"
        release_root = payload_root / "opt" / "studica" / "releases" / release_version
        metadata_root = release_root / "metadata"
        metadata_root.mkdir(parents=True)
        staged_install = release_root / "install"
        shutil.copytree(install_prefix, staged_install, symlinks=True)
        pruned_build_artifacts = prune_build_only_files(staged_install)
        install_file_count, install_bytes = tree_stats(staged_install)

        shutil.copy2(profile_path, metadata_root / "runtime-packages.json")
        shutil.copy2(builder_profile_path, metadata_root / "arm64-builder.json")
        shutil.copy2(
            source_root / "dependencies" / "hardware.repos",
            metadata_root / "hardware.repos",
        )
        shutil.copy2(source_root / "package.xml", metadata_root / "package.xml")
        sdk_inventory_path = metadata_root / "vmxpi-sdk-inventory.json"
        write_json(sdk_inventory_path, sdk_inventory)
        inventory_lines = [
            f"{package.name}\t{package.version}\t{package.architecture}"
            for package in debian_packages
        ]
        (metadata_root / "dpkg-inventory.tsv").write_text(
            "\n".join(inventory_lines) + "\n",
            encoding="utf-8",
        )

        sbom = build_spdx_sbom(
            release_version,
            application_version,
            commit,
            epoch,
            debian_packages,
            repositories,
            sdk_inventory,
        )
        write_json(metadata_root / "sbom.spdx.json", sbom)
        rollback = {
            "schema_version": 1,
            "activation_strategy": "atomic-symlink",
            "current_link": "/opt/studica/current",
            "previous_release_state": "/var/lib/studica/update-state/previous-release",
            "health_check_timeout_sec": 120,
            "automatic_rollback": True,
            "activation_authorized": False,
        }
        write_json(metadata_root / "rollback.json", rollback)
        release_metadata = {
            "schema_version": 1,
            "product": PRODUCT,
            "release_version": release_version,
            "application_version": application_version,
            "channel": "development",
            "activation_authorized": False,
            "activation_blocker": "physical drivetrain hardware safety gate has not passed",
            "source": {
                "commit": commit,
                "source_date_epoch": epoch,
                "repository": "https://github.com/MohammadRobot/studica_vmxpi_ros2.git",
            },
            "platform": profile["platform"],
            "hardware_profile": "stack_4wd",
            "builder": {
                "profile": builder_profile["profile"],
                "base_image": builder_profile["base_image"]["reference"],
                "image_id": builder_image_id,
            },
            "install": {
                "layout": "merged-colcon",
                "relative_prefix": "install",
                "ros_packages": overlay_packages,
                "file_count": install_file_count,
                "bytes": install_bytes,
                "pruned_build_artifacts": pruned_build_artifacts,
            },
            "inputs": {
                "runtime_packages_sha256": sha256_file(profile_path),
                "hardware_repos_sha256": sha256_file(
                    source_root / "dependencies" / "hardware.repos"
                ),
                "dpkg_inventory_sha256": sha256_file(
                    metadata_root / "dpkg-inventory.tsv"
                ),
                "vmxpi_sdk_inventory_sha256": sha256_file(sdk_inventory_path),
                "arm64_builder_profile_sha256": sha256_file(builder_profile_path),
            },
            "sbom": "metadata/sbom.spdx.json",
            "rollback": "metadata/rollback.json",
        }
        write_json(metadata_root / "release.json", release_metadata)
        (metadata_root / "DO_NOT_ACTIVATE").write_text(
            "Development artifact only. Physical drivetrain acceptance is blocked.\n",
            encoding="utf-8",
        )
        write_payload_checksums(release_root)

        temporary_archive = temporary_root / artifact_name
        create_deterministic_archive(payload_root, temporary_archive, epoch)
        archive_digest = sha256_file(temporary_archive)
        temporary_checksum = temporary_root / checksum_name
        temporary_checksum.write_text(
            f"{archive_digest}  {artifact_name}\n",
            encoding="utf-8",
        )
        os.replace(temporary_archive, final_archive)
        os.replace(temporary_checksum, final_checksum)
    return final_archive, final_checksum


def git_output(source_root: Path, arguments: list[str]) -> str:
    result = subprocess.run(
        ["git", *arguments],
        cwd=source_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise BundleError(result.stderr.strip() or f"git {' '.join(arguments)} failed")
    return result.stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument("--install-prefix", type=Path, required=True)
    parser.add_argument(
        "--vmxpi-sdk-root",
        type=Path,
        required=True,
        help="root containing include/vmxpi and lib/vmxpi from the build host",
    )
    parser.add_argument(
        "--dpkg-inventory",
        type=Path,
        required=True,
        help="sorted PACKAGE<TAB>VERSION<TAB>ARCHITECTURE inventory of the target rootfs",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--release-version")
    parser.add_argument(
        "--builder-image-id",
        required=True,
        help="immutable sha256 image ID reported by the isolated ARM64 builder",
    )
    parser.add_argument(
        "--source-date-epoch",
        type=int,
        help="override the Git commit time used for deterministic archive timestamps",
    )
    arguments = parser.parse_args()
    source_root = arguments.source_root.resolve()
    try:
        dirty = git_output(source_root, ["status", "--porcelain", "--untracked-files=normal"])
        if dirty:
            raise BundleError("source repository is dirty; commit or remove every change")
        commit = git_output(source_root, ["rev-parse", "HEAD"])
        epoch = arguments.source_date_epoch
        if epoch is None:
            epoch = int(git_output(source_root, ["show", "-s", "--format=%ct", "HEAD"]))
        archive, checksum = build_release_bundle(
            source_root=source_root,
            install_prefix=arguments.install_prefix,
            vmxpi_sdk_root=arguments.vmxpi_sdk_root,
            dpkg_inventory_path=arguments.dpkg_inventory,
            output_dir=arguments.output_dir,
            commit=commit,
            epoch=epoch,
            builder_image_id=arguments.builder_image_id,
            release_version=arguments.release_version,
        )
    except (
        BundleError,
        OSError,
        ValueError,
        json.JSONDecodeError,
        KeyError,
        TypeError,
    ) as error:
        print(f"Release bundle build failed: {error}", file=sys.stderr)
        return 1
    print(f"Created {archive}")
    print(f"Created {checksum}")
    print("Activation remains blocked pending the physical drivetrain safety gate.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
