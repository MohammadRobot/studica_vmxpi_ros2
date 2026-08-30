#!/usr/bin/env python3
"""Verify and stage a development release without activating robot software."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import fcntl
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import sys
import tarfile
import tempfile
from typing import Any, BinaryIO, Iterator

from verify_release_artifacts import (
    MAX_ARCHIVE_BYTES,
    VerificationError,
    sha256_file,
    verify_release_artifacts,
)


INSTALL_PROFILE = "studica-inactive-install-v1"
RELEASE_VERSION_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._+-]{0,127}")
RELEASES_RELATIVE = Path("opt/studica/releases")
STUDICA_RELATIVE = Path("opt/studica")
STATE_RELATIVE = Path("var/lib/studica/update-state/inactive-releases")
CURRENT_RELATIVE = Path("opt/studica/current")
PREVIOUS_RELATIVE = Path("var/lib/studica/update-state/previous-release")


class InstallationError(RuntimeError):
    """An inactive release cannot be installed without violating policy."""


def _root_path(root: Path) -> Path:
    if not root.is_absolute():
        raise InstallationError("installation root must be an absolute path")
    try:
        root_stat = root.lstat()
    except FileNotFoundError as error:
        raise InstallationError(f"installation root does not exist: {root}") from error
    if stat.S_ISLNK(root_stat.st_mode) or not stat.S_ISDIR(root_stat.st_mode):
        raise InstallationError("installation root must be a real directory")
    resolved = root.resolve()
    if resolved != root:
        raise InstallationError("installation root must be canonical and contain no symlink")
    return resolved


def _secure_directories(root: Path, relative: Path, mode: int = 0o755) -> Path:
    current = root
    for part in relative.parts:
        current = current / part
        try:
            current_stat = current.lstat()
        except FileNotFoundError:
            current.mkdir(mode=mode)
            current_stat = current.lstat()
            if os.geteuid() == 0:
                os.chown(current, 0, 0)
        if stat.S_ISLNK(current_stat.st_mode) or not stat.S_ISDIR(
            current_stat.st_mode
        ):
            raise InstallationError(f"managed path is not a real directory: {current}")
    return current


def _lstat_fingerprint(path: Path) -> dict[str, Any]:
    try:
        path_stat = path.lstat()
    except FileNotFoundError:
        return {"exists": False}
    result: dict[str, Any] = {
        "exists": True,
        "device": path_stat.st_dev,
        "inode": path_stat.st_ino,
        "mode": stat.S_IMODE(path_stat.st_mode),
    }
    if stat.S_ISLNK(path_stat.st_mode):
        result.update({"type": "symlink", "target": os.readlink(path)})
    elif stat.S_ISDIR(path_stat.st_mode):
        result["type"] = "directory"
    elif stat.S_ISREG(path_stat.st_mode):
        result["type"] = "file"
    else:
        result["type"] = "other"
    return result


def _public_link_state(fingerprint: dict[str, Any]) -> dict[str, Any]:
    result = {"exists": fingerprint["exists"]}
    if fingerprint["exists"]:
        result["type"] = fingerprint["type"]
        if fingerprint["type"] == "symlink":
            result["target"] = fingerprint["target"]
    return result


def _regular_artifact_entries(artifact_dir: Path) -> tuple[Path, Path]:
    try:
        entries = sorted(artifact_dir.iterdir())
    except OSError as error:
        raise InstallationError(f"cannot inspect artifact directory: {error}") from error
    if any(path.is_symlink() or not path.is_file() for path in entries):
        raise InstallationError("artifact directory contains a symlink or non-file")
    archives = [path for path in entries if path.name.endswith(".tar.gz")]
    checksums = [path for path in entries if path.name.endswith(".tar.gz.sha256")]
    if len(entries) != 2 or len(archives) != 1 or len(checksums) != 1:
        raise InstallationError(
            "artifact directory must contain exactly one archive and its checksum"
        )
    archive = archives[0]
    checksum = checksums[0]
    if checksum.name != f"{archive.name}.sha256":
        raise InstallationError("artifact checksum filename does not match archive")
    if archive.stat().st_size > MAX_ARCHIVE_BYTES:
        raise InstallationError("release archive exceeds the maximum accepted size")
    if checksum.stat().st_size > 4096:
        raise InstallationError("release checksum is unexpectedly large")
    return archive, checksum


def _copy_regular_file(source: Path, destination: Path) -> None:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    source_fd = os.open(source, flags)
    try:
        source_stat = os.fstat(source_fd)
        if not stat.S_ISREG(source_stat.st_mode):
            raise InstallationError(f"artifact input is not a regular file: {source}")
        destination_fd = os.open(
            destination,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o600,
        )
        try:
            with os.fdopen(source_fd, "rb", closefd=False) as source_stream:
                with os.fdopen(destination_fd, "wb", closefd=False) as output_stream:
                    shutil.copyfileobj(source_stream, output_stream, 1024 * 1024)
                    output_stream.flush()
                    os.fsync(output_stream.fileno())
        finally:
            os.close(destination_fd)
    finally:
        os.close(source_fd)


def _copy_artifacts(artifact_dir: Path, destination: Path) -> None:
    archive, checksum = _regular_artifact_entries(artifact_dir)
    _copy_regular_file(archive, destination / archive.name)
    _copy_regular_file(checksum, destination / checksum.name)


@contextmanager
def _installation_lock(studica_root: Path) -> Iterator[None]:
    lock_path = studica_root / ".inactive-install.lock"
    flags = os.O_RDWR | os.O_CREAT
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(lock_path, flags, 0o600)
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise InstallationError("inactive installation lock is not a regular file")
        fcntl.flock(descriptor, fcntl.LOCK_EX)
        yield
    finally:
        os.close(descriptor)


def _safe_parent(staging: Path, destination: Path) -> None:
    try:
        relative = destination.parent.relative_to(staging)
    except ValueError as error:
        raise InstallationError("extraction destination escapes staging root") from error
    current = staging
    for part in relative.parts:
        current = current / part
        try:
            current_stat = current.lstat()
        except FileNotFoundError:
            current.mkdir(mode=0o755)
            current_stat = current.lstat()
        if stat.S_ISLNK(current_stat.st_mode) or not stat.S_ISDIR(
            current_stat.st_mode
        ):
            raise InstallationError(
                f"archive member has a non-directory parent: {relative}"
            )


def _copy_member(stream: BinaryIO, destination: Path, mode: int) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(destination, flags, mode & 0o777)
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as output:
            shutil.copyfileobj(stream, output, 1024 * 1024)
            output.flush()
            os.fsync(output.fileno())
    finally:
        os.close(descriptor)


def _set_owner(path: Path, *, follow_symlinks: bool = True) -> None:
    if os.geteuid() == 0:
        os.chown(path, 0, 0, follow_symlinks=follow_symlinks)


def _extract_release(archive_path: Path, staging: Path, release_version: str) -> None:
    release_root = f"opt/studica/releases/{release_version}"
    directory_metadata: list[tuple[Path, int, int]] = []
    hardlinks: list[tuple[tarfile.TarInfo, Path]] = []
    with tarfile.open(archive_path, "r:gz") as archive:
        for member in archive.getmembers():
            if member.name == release_root:
                relative = PurePosixPath(".")
            elif member.name.startswith(f"{release_root}/"):
                relative = PurePosixPath(member.name.removeprefix(f"{release_root}/"))
            else:
                continue
            destination = staging.joinpath(*relative.parts)
            if relative.as_posix() == ".":
                directory_metadata.append((staging, member.mode, member.mtime))
                continue
            _safe_parent(staging, destination)
            if member.isdir():
                try:
                    destination.mkdir(mode=0o700)
                except FileExistsError:
                    destination_stat = destination.lstat()
                    if stat.S_ISLNK(destination_stat.st_mode) or not stat.S_ISDIR(
                        destination_stat.st_mode
                    ):
                        raise InstallationError(
                            f"archive directory collides with another member: {member.name}"
                        )
                directory_metadata.append((destination, member.mode, member.mtime))
            elif member.isfile():
                stream = archive.extractfile(member)
                if stream is None:
                    raise InstallationError(f"cannot read archive member: {member.name}")
                with stream:
                    _copy_member(stream, destination, member.mode)
                os.chmod(destination, member.mode & 0o777)
                os.utime(destination, (member.mtime, member.mtime))
                _set_owner(destination)
            elif member.issym():
                os.symlink(member.linkname, destination)
                _set_owner(destination, follow_symlinks=False)
            elif member.islnk():
                hardlinks.append((member, destination))
            else:
                raise InstallationError(f"unsupported archive member: {member.name}")

        for member, destination in hardlinks:
            target_name = PurePosixPath(member.linkname)
            try:
                target_relative = target_name.relative_to(PurePosixPath(release_root))
            except ValueError as error:
                raise InstallationError(
                    f"hardlink target escapes release: {member.name}"
                ) from error
            target = staging.joinpath(*target_relative.parts)
            _safe_parent(staging, destination)
            try:
                target_stat = target.lstat()
            except FileNotFoundError as error:
                raise InstallationError(
                    f"hardlink target is missing: {member.linkname}"
                ) from error
            if not stat.S_ISREG(target_stat.st_mode):
                raise InstallationError(
                    f"hardlink target is not a regular file: {member.linkname}"
                )
            os.link(target, destination, follow_symlinks=False)
            _set_owner(destination)

    for directory, mode, modified in sorted(
        directory_metadata, key=lambda item: len(item[0].parts), reverse=True
    ):
        os.chmod(directory, mode & 0o777)
        os.utime(directory, (modified, modified))
        _set_owner(directory)


def _verify_extracted_release(release_root: Path) -> dict[str, Any]:
    metadata_path = release_root / "metadata/release.json"
    blocker_path = release_root / "metadata/DO_NOT_ACTIVATE"
    checksums_path = release_root / "SHA256SUMS"
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        blocker = blocker_path.read_text(encoding="utf-8")
        checksum_lines = checksums_path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise InstallationError(f"installed release metadata is unreadable: {error}") from error
    if not isinstance(metadata, dict) or metadata.get("activation_authorized") is not False:
        raise InstallationError("installed release does not explicitly block activation")
    if "Development artifact only" not in blocker:
        raise InstallationError("installed release has no valid DO_NOT_ACTIVATE marker")
    checked: set[str] = set()
    for line in checksum_lines:
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        if match is None:
            raise InstallationError("installed SHA256SUMS contains an invalid record")
        relative = PurePosixPath(match.group(2))
        if relative.is_absolute() or ".." in relative.parts:
            raise InstallationError("installed SHA256SUMS contains an unsafe path")
        relative_name = relative.as_posix()
        if relative_name in checked:
            raise InstallationError("installed SHA256SUMS contains a duplicate path")
        checked.add(relative_name)
        path = release_root.joinpath(*relative.parts)
        try:
            path_stat = path.lstat()
        except FileNotFoundError as error:
            raise InstallationError(f"installed checksummed file is missing: {path}") from error
        hash_path = path
        if stat.S_ISLNK(path_stat.st_mode):
            try:
                hash_path = path.resolve(strict=True)
                hash_path.relative_to(release_root.resolve(strict=True))
                target_stat = hash_path.lstat()
            except (FileNotFoundError, RuntimeError, ValueError) as error:
                raise InstallationError(
                    f"installed checksummed symlink is unsafe: {path}"
                ) from error
            if not stat.S_ISREG(target_stat.st_mode):
                raise InstallationError(
                    f"installed checksummed symlink target is not a file: {path}"
                )
        elif not stat.S_ISREG(path_stat.st_mode):
            raise InstallationError(f"installed checksummed path is not a file: {path}")
        if sha256_file(hash_path) != match.group(1):
            raise InstallationError(f"installed checksum mismatch: {path}")
    return metadata


def _atomic_json(path: Path, content: dict[str, Any]) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(content, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o644)
        _set_owner(temporary)
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _fsync_release_directories(release_root: Path) -> None:
    directories = [release_root]
    for parent, names, _ in os.walk(release_root, followlinks=False):
        parent_path = Path(parent)
        for name in names:
            child = parent_path / name
            if child.is_dir() and not child.is_symlink():
                directories.append(child)
    for directory in sorted(set(directories), key=lambda path: len(path.parts), reverse=True):
        descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


def install_inactive_release(
    artifact_dir: Path,
    expected_commit: str,
    root: Path = Path("/"),
) -> dict[str, Any]:
    root = _root_path(root)
    if root == Path("/") and os.geteuid() != 0:
        raise InstallationError("installing under / requires root privileges")
    artifact_dir = artifact_dir.resolve()
    current_path = root / CURRENT_RELATIVE
    previous_path = root / PREVIOUS_RELATIVE
    current_before = _lstat_fingerprint(current_path)
    previous_before = _lstat_fingerprint(previous_path)

    studica_root = _secure_directories(root, STUDICA_RELATIVE)
    releases_root = _secure_directories(root, RELEASES_RELATIVE)
    state_root = _secure_directories(root, STATE_RELATIVE, mode=0o750)
    with _installation_lock(studica_root):
        with tempfile.TemporaryDirectory(
            prefix=".inactive-install-", dir=studica_root
        ) as temporary_name:
            temporary_root = Path(temporary_name)
            os.chmod(temporary_root, 0o700)
            artifact_copy = temporary_root / "artifact"
            artifact_copy.mkdir(mode=0o700)
            _copy_artifacts(artifact_dir, artifact_copy)
            try:
                verified = verify_release_artifacts(
                    artifact_copy, expected_commit=expected_commit
                )
            except (OSError, UnicodeDecodeError, VerificationError) as error:
                raise InstallationError(f"release verification failed: {error}") from error
            release_version = verified["release_version"]
            if RELEASE_VERSION_RE.fullmatch(release_version) is None:
                raise InstallationError("verified release version is not filesystem-safe")
            final_release = releases_root / release_version
            record_path = state_root / f"{release_version}.json"
            try:
                final_release.lstat()
            except FileNotFoundError:
                pass
            else:
                raise InstallationError(f"release destination already exists: {final_release}")
            try:
                record_path.lstat()
            except FileNotFoundError:
                pass
            else:
                raise InstallationError(
                    f"inactive install record already exists: {record_path}"
                )

            staging = Path(
                tempfile.mkdtemp(prefix=f".{release_version}.", dir=releases_root)
            )
            os.chmod(staging, 0o700)
            try:
                archive_copy = artifact_copy / verified["archive"]
                _extract_release(archive_copy, staging, release_version)
                metadata = _verify_extracted_release(staging)
                if metadata.get("release_version") != release_version:
                    raise InstallationError("installed release metadata version changed")
                if metadata.get("source", {}).get("commit") != expected_commit:
                    raise InstallationError("installed release source commit changed")
                _fsync_release_directories(staging)
                staging.rename(final_release)
                releases_descriptor = os.open(
                    releases_root, os.O_RDONLY | os.O_DIRECTORY
                )
                try:
                    os.fsync(releases_descriptor)
                finally:
                    os.close(releases_descriptor)
                staging = Path()
            finally:
                if staging != Path() and staging.exists():
                    shutil.rmtree(staging)

            record = {
                "schema_version": 1,
                "profile": INSTALL_PROFILE,
                "release_version": release_version,
                "release_path": f"/opt/studica/releases/{release_version}",
                "source_commit": expected_commit,
                "archive": verified["archive"],
                "archive_sha256": verified["archive_sha256"],
                "installer_sha256": sha256_file(Path(__file__).resolve()),
                "channel": "development",
                "cryptographic_signature_verified": False,
                "activation_authorized": False,
                "activation_blocker": metadata["activation_blocker"],
                "installed_at": datetime.now(timezone.utc).isoformat(),
                "current_link_observed": _public_link_state(current_before),
                "previous_release_state_observed": _public_link_state(
                    previous_before
                ),
            }
            _atomic_json(record_path, record)

    current_after = _lstat_fingerprint(current_path)
    previous_after = _lstat_fingerprint(previous_path)
    if current_after != current_before:
        raise InstallationError("current release link changed during inactive install")
    if previous_after != previous_before:
        raise InstallationError("previous release state changed during inactive install")
    return record


def inspect_inactive_releases(root: Path = Path("/")) -> dict[str, Any]:
    root = _root_path(root)
    releases_root = root / RELEASES_RELATIVE
    installed: list[dict[str, Any]] = []
    staging_entries: list[str] = []
    if releases_root.is_dir() and not releases_root.is_symlink():
        for release_path in sorted(releases_root.iterdir()):
            if release_path.name.startswith("."):
                staging_entries.append(release_path.name)
                continue
            entry: dict[str, Any] = {"release_version": release_path.name}
            if release_path.is_symlink() or not release_path.is_dir():
                entry.update({"valid": False, "reason": "not a real directory"})
            else:
                try:
                    metadata = json.loads(
                        release_path.joinpath("metadata/release.json").read_text(
                            encoding="utf-8"
                        )
                    )
                    marker = release_path.joinpath(
                        "metadata/DO_NOT_ACTIVATE"
                    ).read_text(encoding="utf-8")
                    valid = (
                        isinstance(metadata, dict)
                        and metadata.get("release_version") == release_path.name
                        and metadata.get("activation_authorized") is False
                        and "Development artifact only" in marker
                    )
                    entry.update(
                        {
                            "valid": valid,
                            "source_commit": metadata.get("source", {}).get("commit"),
                            "channel": metadata.get("channel"),
                            "activation_authorized": metadata.get(
                                "activation_authorized"
                            ),
                        }
                    )
                except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
                    entry.update({"valid": False, "reason": str(error)})
            installed.append(entry)
    return {
        "profile": INSTALL_PROFILE,
        "current": _public_link_state(_lstat_fingerprint(root / CURRENT_RELATIVE)),
        "previous_release_state": _public_link_state(
            _lstat_fingerprint(root / PREVIOUS_RELATIVE)
        ),
        "staging_entries": staging_entries,
        "installed_releases": installed,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    install_parser = subparsers.add_parser(
        "install", help="verify and atomically stage one inactive release"
    )
    install_parser.add_argument("--artifact-dir", type=Path, required=True)
    install_parser.add_argument("--expected-commit", required=True)
    install_parser.add_argument("--root", type=Path, default=Path("/"))
    status_parser = subparsers.add_parser(
        "status", help="inspect releases and activation pointers without changing them"
    )
    status_parser.add_argument("--root", type=Path, default=Path("/"))
    arguments = parser.parse_args()
    try:
        if arguments.command == "install":
            result = install_inactive_release(
                arguments.artifact_dir,
                arguments.expected_commit,
                arguments.root,
            )
            print(
                "[install] Inactive development release staged: "
                f"{result['release_path']}"
            )
            print(f"[install] Source commit: {result['source_commit']}")
            print("[install] Activation remains blocked; current link was not changed")
        else:
            print(json.dumps(inspect_inactive_releases(arguments.root), indent=2))
    except (InstallationError, OSError) as error:
        print(f"Inactive release operation failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
