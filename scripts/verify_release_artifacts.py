#!/usr/bin/env python3
"""Verify a development release archive without extracting or activating it."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import posixpath
import re
import sys
import tarfile
from typing import Any, BinaryIO


GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")
SHA256_RE = re.compile(r"[0-9a-f]{64}")
IMAGE_ID_RE = re.compile(r"sha256:[0-9a-f]{64}")
PRODUCT = "studica-robot"
BUILDER_PROFILE = "studica-arm64-builder-v1"
MAX_METADATA_BYTES = 4 * 1024 * 1024
MAX_ARCHIVE_BYTES = 2 * 1024 * 1024 * 1024
MAX_ARCHIVE_MEMBERS = 100_000
MAX_UNCOMPRESSED_BYTES = 4 * 1024 * 1024 * 1024


class VerificationError(RuntimeError):
    """A release artifact violates the development release contract."""


def sha256_stream(stream: BinaryIO) -> str:
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def sha256_file(path: Path) -> str:
    with path.open("rb") as stream:
        return sha256_stream(stream)


def safe_member_name(name: str) -> str:
    if name == ".":
        return name
    path = PurePosixPath(name)
    if path.is_absolute() or not path.parts or ".." in path.parts:
        raise VerificationError(f"archive member has an unsafe path: {name!r}")
    normalized = path.as_posix()
    if normalized in ("", ".") or normalized.startswith("./"):
        raise VerificationError(f"archive member path is not canonical: {name!r}")
    return normalized


def validate_link(member: tarfile.TarInfo, name: str) -> None:
    if not (member.issym() or member.islnk()):
        return
    if not member.linkname or PurePosixPath(member.linkname).is_absolute():
        raise VerificationError(f"archive link has an unsafe target: {name!r}")
    base = posixpath.dirname(name) if member.issym() else ""
    target = posixpath.normpath(posixpath.join(base, member.linkname))
    if target == ".." or target.startswith("../"):
        raise VerificationError(f"archive link escapes its release tree: {name!r}")


def read_member(
    archive: tarfile.TarFile,
    member: tarfile.TarInfo,
    *,
    maximum_bytes: int | None = None,
) -> bytes:
    if maximum_bytes is not None and member.size > maximum_bytes:
        raise VerificationError(f"archive metadata is too large: {member.name}")
    stream = archive.extractfile(member)
    if stream is None:
        raise VerificationError(f"archive member is not a readable file: {member.name}")
    with stream:
        return stream.read()


def parse_checksum_line(text: str, label: str) -> tuple[str, str]:
    lines = text.splitlines()
    if len(lines) != 1:
        raise VerificationError(f"{label} must contain exactly one checksum line")
    match = re.fullmatch(r"([0-9a-f]{64})  ([^/\s]+)", lines[0])
    if match is None:
        raise VerificationError(f"{label} has an invalid SHA-256 record")
    return match.group(1), match.group(2)


def validate_release_policy(
    metadata: dict[str, Any],
    rollback: dict[str, Any],
    release_version: str,
    expected_commit: str | None,
) -> None:
    if metadata.get("schema_version") != 1 or metadata.get("product") != PRODUCT:
        raise VerificationError("release metadata has an unexpected schema or product")
    if metadata.get("release_version") != release_version:
        raise VerificationError("archive path and release metadata version differ")
    if metadata.get("channel") != "development":
        raise VerificationError("only the development channel is accepted")
    if metadata.get("activation_authorized") is not False:
        raise VerificationError("release metadata authorizes activation")
    if not str(metadata.get("activation_blocker", "")).strip():
        raise VerificationError("release metadata has no activation blocker")

    source = metadata.get("source")
    commit = source.get("commit") if isinstance(source, dict) else None
    if not isinstance(commit, str) or GIT_SHA_RE.fullmatch(commit) is None:
        raise VerificationError("release source commit is not an immutable Git SHA")
    if expected_commit is not None and commit != expected_commit:
        raise VerificationError(
            f"release source commit {commit} does not match {expected_commit}"
        )
    source_date_epoch = source.get("source_date_epoch")
    if not isinstance(source_date_epoch, int) or source_date_epoch < 0:
        raise VerificationError("release source timestamp is invalid")

    platform = metadata.get("platform")
    if not isinstance(platform, dict) or any(
        platform.get(key) != value
        for key, value in {
            "architecture": "arm64",
            "os_id": "ubuntu",
            "ros_distro": "humble",
            "version_id": "22.04",
        }.items()
    ):
        raise VerificationError("release platform is not Ubuntu 22.04/Humble ARM64")

    builder = metadata.get("builder")
    if (
        not isinstance(builder, dict)
        or builder.get("profile") != BUILDER_PROFILE
        or IMAGE_ID_RE.fullmatch(str(builder.get("image_id", ""))) is None
    ):
        raise VerificationError("release builder provenance is incomplete")
    if rollback.get("activation_authorized") is not False:
        raise VerificationError("rollback metadata authorizes activation")


def verify_release_artifacts(
    output_dir: Path,
    expected_commit: str | None = None,
) -> dict[str, str]:
    output_dir = output_dir.resolve()
    if expected_commit is not None and GIT_SHA_RE.fullmatch(expected_commit) is None:
        raise VerificationError("expected commit must be a full 40-character Git SHA")
    if not output_dir.is_dir():
        raise VerificationError(f"release output directory does not exist: {output_dir}")

    entries = sorted(output_dir.iterdir())
    if any(not path.is_file() or path.is_symlink() for path in entries):
        raise VerificationError("release output contains a directory or symlink")
    archives = [path for path in entries if path.name.endswith(".tar.gz")]
    checksums = [path for path in entries if path.name.endswith(".tar.gz.sha256")]
    if len(archives) != 1 or len(checksums) != 1 or len(entries) != 2:
        raise VerificationError(
            "release output must contain exactly one archive and its checksum"
        )
    archive_path = archives[0]
    checksum_path = checksums[0]
    if checksum_path.name != f"{archive_path.name}.sha256":
        raise VerificationError("external checksum filename does not match the archive")
    if archive_path.stat().st_size > MAX_ARCHIVE_BYTES:
        raise VerificationError("release archive exceeds the maximum accepted size")
    if checksum_path.stat().st_size > 4096:
        raise VerificationError("external checksum file is unexpectedly large")
    expected_digest, expected_name = parse_checksum_line(
        checksum_path.read_text(encoding="utf-8"), "external checksum"
    )
    if expected_name != archive_path.name or sha256_file(archive_path) != expected_digest:
        raise VerificationError("external archive checksum does not match")

    try:
        archive = tarfile.open(archive_path, mode="r:gz")
    except (OSError, tarfile.TarError) as error:
        raise VerificationError(f"cannot open release archive: {error}") from error

    with archive:
        archive_members = archive.getmembers()
        if len(archive_members) > MAX_ARCHIVE_MEMBERS:
            raise VerificationError("release archive contains too many members")
        if sum(member.size for member in archive_members) > MAX_UNCOMPRESSED_BYTES:
            raise VerificationError("release archive expands beyond the accepted size")
        members: dict[str, tarfile.TarInfo] = {}
        for member in archive_members:
            if member.size < 0:
                raise VerificationError("archive member has a negative size")
            name = safe_member_name(member.name)
            if name in members:
                raise VerificationError(f"archive contains duplicate member: {name}")
            if member.ischr() or member.isblk() or member.isfifo():
                raise VerificationError(f"archive contains a device or FIFO: {name}")
            if not (member.isdir() or member.isfile() or member.issym() or member.islnk()):
                raise VerificationError(f"archive contains an unsupported member: {name}")
            # POSIX ignores permission bits on symbolic links, and tar writers
            # conventionally record them as 0777.  The target is constrained
            # to the release root below; regular files and directories must
            # still never be world-writable.
            if not (member.issym() or member.islnk()) and member.mode & 0o002:
                raise VerificationError(f"archive member is world-writable: {name}")
            if member.uid != 0 or member.gid != 0:
                raise VerificationError(f"archive member ownership is not root: {name}")
            validate_link(member, name)
            members[name] = member

        metadata_paths = [
            name for name in members if name.endswith("/metadata/release.json")
        ]
        if len(metadata_paths) != 1:
            raise VerificationError("archive must contain exactly one release.json")
        metadata_path = metadata_paths[0]
        suffix = "/metadata/release.json"
        release_root = metadata_path[: -len(suffix)]
        prefix = "opt/studica/releases/"
        if not release_root.startswith(prefix):
            raise VerificationError("release payload is outside /opt/studica/releases")
        release_version = release_root.removeprefix(prefix)
        if not release_version or "/" in release_version:
            raise VerificationError("archive release version path is invalid")
        expected_archive_name = (
            f"{PRODUCT}-{release_version}-ubuntu22.04-humble-arm64.tar.gz"
        )
        if archive_path.name != expected_archive_name:
            raise VerificationError("archive filename does not match release metadata path")

        allowed_ancestors = {".", "opt", "opt/studica", "opt/studica/releases"}
        for ancestor in allowed_ancestors:
            if ancestor not in members or not members[ancestor].isdir():
                raise VerificationError(
                    f"archive is missing release ancestor directory: {ancestor}"
                )
        for name, member in members.items():
            if name in allowed_ancestors:
                if not member.isdir():
                    raise VerificationError(
                        f"archive release ancestor is not a directory: {name}"
                    )
                continue
            if name != release_root and not name.startswith(f"{release_root}/"):
                raise VerificationError(f"archive member is outside the release root: {name}")
            if name == release_root and not member.isdir():
                raise VerificationError("archive release root is not a directory")
            if member.issym() or member.islnk():
                base = posixpath.dirname(name) if member.issym() else ""
                target = posixpath.normpath(posixpath.join(base, member.linkname))
                if target != release_root and not target.startswith(
                    f"{release_root}/"
                ):
                    raise VerificationError(
                        f"archive symlink escapes the release root: {name}"
                    )

        required = {
            "metadata": metadata_path,
            "rollback": f"{release_root}/metadata/rollback.json",
            "blocker": f"{release_root}/metadata/DO_NOT_ACTIVATE",
            "checksums": f"{release_root}/SHA256SUMS",
            "sbom": f"{release_root}/metadata/sbom.spdx.json",
            "runtime_packages": f"{release_root}/metadata/runtime-packages.json",
            "hardware_repos": f"{release_root}/metadata/hardware.repos",
            "sdk_inventory": f"{release_root}/metadata/vmxpi-sdk-inventory.json",
            "builder_profile": f"{release_root}/metadata/arm64-builder.json",
            "dpkg_inventory": f"{release_root}/metadata/dpkg-inventory.tsv",
            "package_manifest": f"{release_root}/metadata/package.xml",
        }
        for label, name in required.items():
            if name not in members or not members[name].isfile():
                raise VerificationError(f"archive is missing required {label}: {name}")

        try:
            metadata = json.loads(
                read_member(
                    archive, members[required["metadata"]], maximum_bytes=MAX_METADATA_BYTES
                )
            )
            rollback = json.loads(
                read_member(
                    archive, members[required["rollback"]], maximum_bytes=MAX_METADATA_BYTES
                )
            )
        except (json.JSONDecodeError, UnicodeDecodeError) as error:
            raise VerificationError(f"release metadata is invalid JSON: {error}") from error
        if not isinstance(metadata, dict) or not isinstance(rollback, dict):
            raise VerificationError("release and rollback metadata must be JSON objects")
        validate_release_policy(metadata, rollback, release_version, expected_commit)
        source_date_epoch = metadata["source"]["source_date_epoch"]
        if any(member.mtime != source_date_epoch for member in archive_members):
            raise VerificationError("archive timestamps differ from source provenance")

        if metadata.get("sbom") != "metadata/sbom.spdx.json":
            raise VerificationError("release metadata has an unexpected SBOM path")
        input_hashes = metadata.get("inputs")
        if not isinstance(input_hashes, dict):
            raise VerificationError("release input hashes are missing")
        hash_contract = {
            "runtime_packages_sha256": "runtime_packages",
            "hardware_repos_sha256": "hardware_repos",
            "dpkg_inventory_sha256": "dpkg_inventory",
            "vmxpi_sdk_inventory_sha256": "sdk_inventory",
            "arm64_builder_profile_sha256": "builder_profile",
        }
        for input_name, required_name in hash_contract.items():
            expected_input_digest = input_hashes.get(input_name)
            if (
                not isinstance(expected_input_digest, str)
                or SHA256_RE.fullmatch(expected_input_digest) is None
            ):
                raise VerificationError(f"release input hash is invalid: {input_name}")
            input_stream = archive.extractfile(members[required[required_name]])
            if input_stream is None:
                raise VerificationError(f"release input is unreadable: {required_name}")
            with input_stream:
                if sha256_stream(input_stream) != expected_input_digest:
                    raise VerificationError(f"release input hash mismatch: {input_name}")

        try:
            sbom = json.loads(
                read_member(
                    archive, members[required["sbom"]], maximum_bytes=MAX_METADATA_BYTES
                )
            )
            builder_profile = json.loads(
                read_member(
                    archive,
                    members[required["builder_profile"]],
                    maximum_bytes=MAX_METADATA_BYTES,
                )
            )
            sdk_inventory = json.loads(
                read_member(
                    archive,
                    members[required["sdk_inventory"]],
                    maximum_bytes=MAX_METADATA_BYTES,
                )
            )
        except (json.JSONDecodeError, UnicodeDecodeError) as error:
            raise VerificationError(f"release inventory is invalid JSON: {error}") from error
        if not isinstance(sbom, dict) or sbom.get("spdxVersion") != "SPDX-2.3":
            raise VerificationError("release SBOM is missing or is not SPDX 2.3")
        builder_execution = (
            builder_profile.get("execution")
            if isinstance(builder_profile, dict)
            else None
        )
        if (
            not isinstance(builder_profile, dict)
            or builder_profile.get("profile") != BUILDER_PROFILE
            or builder_profile.get("artifact_policy")
            != {"activation_authorized": False, "channel": "development"}
            or not isinstance(builder_execution, dict)
            or builder_execution.get("sdk_enabled_build_network") != "none"
        ):
            raise VerificationError("embedded ARM64 builder profile is invalid")
        if (
            not isinstance(sdk_inventory, dict)
            or sdk_inventory.get("profile")
            != "studica-vmxpi-hal-cpp-content-v1"
        ):
            raise VerificationError("embedded VMXPi SDK inventory is invalid")

        blocker = read_member(
            archive, members[required["blocker"]], maximum_bytes=4096
        ).decode("utf-8")
        if "Development artifact only" not in blocker:
            raise VerificationError("DO_NOT_ACTIVATE marker has unexpected content")

        checksum_text = read_member(
            archive, members[required["checksums"]], maximum_bytes=MAX_METADATA_BYTES
        ).decode("utf-8")
        recorded: set[str] = set()
        for line in checksum_text.splitlines():
            match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
            if match is None:
                raise VerificationError("internal SHA256SUMS has an invalid record")
            relative = safe_member_name(match.group(2))
            if relative == "." or relative in recorded:
                raise VerificationError("internal SHA256SUMS has a duplicate path")
            recorded.add(relative)
            member_name = f"{release_root}/{relative}"
            member = members.get(member_name)
            if member is None:
                raise VerificationError(f"checksummed member is missing: {relative}")
            stream = archive.extractfile(member)
            if stream is None:
                raise VerificationError(f"checksummed member is unreadable: {relative}")
            with stream:
                if sha256_stream(stream) != match.group(1):
                    raise VerificationError(f"internal checksum mismatch: {relative}")

        regular_files = {
            name.removeprefix(f"{release_root}/")
            for name, member in members.items()
            if name.startswith(f"{release_root}/")
            and member.isfile()
            and name != required["checksums"]
        }
        missing_checksums = sorted(regular_files - recorded)
        if missing_checksums:
            raise VerificationError(
                f"regular files missing from internal SHA256SUMS: {missing_checksums}"
            )

    return {
        "archive": archive_path.name,
        "archive_sha256": expected_digest,
        "release_version": release_version,
        "source_commit": str(metadata["source"]["commit"]),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--expected-commit")
    arguments = parser.parse_args()
    try:
        result = verify_release_artifacts(
            arguments.output_dir,
            expected_commit=arguments.expected_commit,
        )
    except (OSError, UnicodeDecodeError, VerificationError) as error:
        print(f"Release artifact verification failed: {error}", file=sys.stderr)
        return 1
    print(
        "[check] Development release verified: "
        f"{result['archive']} ({result['archive_sha256']})"
    )
    print("[check] Activation remains blocked")
    return 0


if __name__ == "__main__":
    sys.exit(main())
