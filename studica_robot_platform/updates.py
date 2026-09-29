"""Signed, staged, atomic application update primitives."""

from __future__ import annotations

import base64
import hashlib
import json
import os
import posixpath
from pathlib import Path, PurePosixPath
import re
import shutil
import tarfile
import tempfile
import time
from typing import Any, BinaryIO, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey


PRODUCT = "studica-robot"
VERSION = re.compile(r"[0-9A-Za-z][0-9A-Za-z.+-]{0,63}")
SHA256 = re.compile(r"[0-9a-f]{64}")
MAX_MANIFEST_BYTES = 64 * 1024
MAX_ARTIFACT_BYTES = 2 * 1024 * 1024 * 1024
PLATFORM = {
    "architecture": "arm64",
    "os_id": "ubuntu",
    "ros_distro": "humble",
    "version_id": "22.04",
}


class UpdateError(RuntimeError):
    """An update violated signature, filesystem, or activation policy."""


def _sync_directory(path: Path) -> None:
    descriptor = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def canonical_json(document: Any) -> bytes:
    return json.dumps(
        document, separators=(",", ":"), sort_keys=True, ensure_ascii=True
    ).encode("ascii")


def sha256_stream(stream: BinaryIO) -> str:
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()


def sha256_file(path: Path) -> str:
    with path.open("rb") as stream:
        return sha256_stream(stream)


def verify_manifest(document: Any, public_key_path: Path) -> dict[str, Any]:
    """Verify an Ed25519 envelope and return its constrained signed payload."""
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise UpdateError("update envelope schema is unsupported")
    signed = document.get("signed")
    signature_text = document.get("signature")
    if not isinstance(signed, dict) or not isinstance(signature_text, str):
        raise UpdateError("update envelope is incomplete")
    try:
        key = serialization.load_pem_public_key(public_key_path.read_bytes())
        signature = base64.b64decode(signature_text, validate=True)
    except (OSError, ValueError, TypeError) as error:
        raise UpdateError(f"update signature material is invalid: {error}") from error
    if not isinstance(key, Ed25519PublicKey):
        raise UpdateError("update public key must be Ed25519")
    try:
        key.verify(signature, canonical_json(signed))
    except InvalidSignature as error:
        raise UpdateError("update manifest signature is invalid") from error
    version = signed.get("version")
    if signed.get("product") != PRODUCT or not isinstance(version, str):
        raise UpdateError("update product or version is invalid")
    if VERSION.fullmatch(version) is None:
        raise UpdateError("update version is unsafe")
    if signed.get("platform") != PLATFORM:
        raise UpdateError("update targets a different platform")
    if not isinstance(signed.get("artifact_url"), str) or not signed[
        "artifact_url"
    ].startswith("https://"):
        raise UpdateError("artifact URL must use HTTPS")
    if SHA256.fullmatch(str(signed.get("artifact_sha256", ""))) is None:
        raise UpdateError("artifact SHA-256 is invalid")
    size = signed.get("artifact_bytes")
    if not isinstance(size, int) or not 1 <= size <= MAX_ARTIFACT_BYTES:
        raise UpdateError("artifact size is invalid")
    if signed.get("release_root") != f"opt/studica/releases/{version}":
        raise UpdateError("release root does not match version")
    if "qualification" in signed:
        validate_signed_qualification(signed["qualification"], signed["artifact_sha256"])
    return signed


def validate_signed_qualification(report: Any, digest: str) -> None:
    """Bind production acceptance to the exact immutable development artifact."""
    if (not isinstance(report, dict) or report.get("schema_version") != 1
            or report.get("tested_release_sha256") != digest
            or type(report.get("cold_boots_passed")) is not int
            or report["cold_boots_passed"] < 50
            or any(report.get(key) is not True for key in (
                "independent_torque_removal", "failure_injection_passed", "zero_motion_all_boots"))):
        raise UpdateError("signed hardware qualification is incomplete or identifies another artifact")


def read_https_json(url: str) -> dict[str, Any]:
    if not url.startswith("https://"):
        raise UpdateError("update manifest URL must use HTTPS")
    try:
        request = Request(url, headers={"User-Agent": "studica-update/1"})
        with urlopen(request, timeout=20) as response:
            payload = response.read(MAX_MANIFEST_BYTES + 1)
    except (HTTPError, URLError, OSError) as error:
        raise UpdateError(f"cannot download update manifest: {error}") from error
    if len(payload) > MAX_MANIFEST_BYTES:
        raise UpdateError("update manifest is too large")
    try:
        document = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise UpdateError("update manifest is not valid JSON") from error
    if not isinstance(document, dict):
        raise UpdateError("update manifest must contain an object")
    return document


def _atomic_json(
    path: Path,
    document: Any,
    mode: int = 0o600,
    owner: Optional[tuple[int, int]] = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        os.chmod(temporary, mode)
        if owner is not None:
            os.fchown(stream.fileno(), owner[0], owner[1])
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    _sync_directory(path.parent)


def stage_update(
    envelope: dict[str, Any], public_key: Path, state_root: Path
) -> dict[str, Any]:
    """Download and verify an artifact without changing the active release."""
    signed = verify_manifest(envelope, public_key)
    version = signed["version"]
    state_root.mkdir(parents=True, exist_ok=True)
    destination = state_root / "staged" / version
    if destination.exists():
        status = json.loads((destination / "status.json").read_text(encoding="utf-8"))
        if status.get("artifact_sha256") == signed["artifact_sha256"]:
            return status
        raise UpdateError("version is already staged with different content")
    staging = Path(tempfile.mkdtemp(prefix=f".{version}.", dir=state_root))
    os.chmod(staging, 0o700)
    artifact = staging / "release.tar.gz"
    try:
        request = Request(
            signed["artifact_url"], headers={"User-Agent": "studica-update/1"}
        )
        try:
            response = urlopen(request, timeout=60)
        except (HTTPError, URLError, OSError) as error:
            raise UpdateError(f"cannot download release artifact: {error}") from error
        digest = hashlib.sha256()
        count = 0
        with response, artifact.open("xb") as stream:
            os.chmod(artifact, 0o600)
            while True:
                block = response.read(1024 * 1024)
                if not block:
                    break
                count += len(block)
                if count > signed["artifact_bytes"] or count > MAX_ARTIFACT_BYTES:
                    raise UpdateError("release artifact exceeds declared size")
                digest.update(block)
                stream.write(block)
            stream.flush()
            os.fsync(stream.fileno())
        if count != signed["artifact_bytes"]:
            raise UpdateError("release artifact size does not match manifest")
        if digest.hexdigest() != signed["artifact_sha256"]:
            raise UpdateError("release artifact SHA-256 does not match manifest")
        (staging / "manifest.json").write_bytes(canonical_json(envelope) + b"\n")
        status = {
            "schema_version": 1,
            "version": version,
            "state": "VERIFIED_PENDING_APPROVAL",
            "artifact_sha256": signed["artifact_sha256"],
            "signature_verified": True,
            "activation_approved": False,
        }
        _atomic_json(staging / "status.json", status)
        destination.parent.mkdir(parents=True, exist_ok=True)
        os.replace(staging, destination)
        _sync_directory(destination.parent)
        return status
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def staged_status(state_root: Path) -> dict[str, Any]:
    staged = state_root / "staged"
    updates = []
    if staged.is_dir() and not staged.is_symlink():
        for entry in sorted(staged.iterdir()):
            try:
                value = json.loads((entry / "status.json").read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if isinstance(value, dict):
                updates.append(value)
    return {"schema_version": 1, "updates": updates}


def _safe_member(member: tarfile.TarInfo, release_root: str) -> None:
    path = PurePosixPath(member.name)
    if path.is_absolute() or ".." in path.parts:
        raise UpdateError(f"unsafe archive member: {member.name}")
    name = path.as_posix()
    ancestors = {".", "opt", "opt/studica", "opt/studica/releases"}
    if name in ancestors and not member.isdir():
        raise UpdateError("release ancestors must be directories")
    if name not in ancestors and name != release_root and not name.startswith(
        release_root + "/"
    ):
        raise UpdateError(f"archive member is outside release root: {name}")
    if not (member.isdir() or member.isfile() or member.issym() or member.islnk()):
        raise UpdateError(f"unsupported archive member: {name}")
    if not (member.issym() or member.islnk()) and member.mode & 0o002:
        raise UpdateError(f"world-writable archive member: {name}")
    if member.mode & 0o6000:
        raise UpdateError(f"privileged archive permissions: {name}")
    if member.uid != 0 or member.gid != 0:
        raise UpdateError(f"archive member ownership is not root: {name}")
    if member.issym() or member.islnk():
        target = PurePosixPath(member.linkname)
        if target.is_absolute() or ".." in target.parts:
            raise UpdateError(f"unsafe archive link: {name}")
        resolved = posixpath.normpath(posixpath.join(
            posixpath.dirname(name) if member.issym() else "", member.linkname))
        if not resolved.startswith(release_root + "/"):
            raise UpdateError(f"archive link leaves release: {name}")


def extract_verified_release(
    version: str, state_root: Path, releases_root: Path, public_key: Path
) -> Path:
    """Reverify and atomically extract a production release."""
    if VERSION.fullmatch(version) is None:
        raise UpdateError("unsafe version")
    staged = state_root / "staged" / version
    envelope = json.loads((staged / "manifest.json").read_text(encoding="utf-8"))
    signed = verify_manifest(envelope, public_key)
    if signed["version"] != version:
        raise UpdateError("signed version does not match the requested release")
    artifact = staged / "release.tar.gz"
    if artifact.is_symlink() or not artifact.is_file():
        raise UpdateError("staged artifact is missing")
    final = releases_root / version
    if final.exists() or final.is_symlink():
        raise UpdateError("release already exists; refusing to trust or overwrite existing contents")
    releases_root.mkdir(parents=True, exist_ok=True)
    extraction = Path(tempfile.mkdtemp(prefix=f".{version}.", dir=releases_root))
    release_root = signed["release_root"]
    try:
        snapshot = extraction / "verified.tar.gz"
        with artifact.open("rb") as source, snapshot.open("xb") as target:
            size = 0
            while True:
                block = source.read(1024 * 1024)
                if not block:
                    break
                size += len(block)
                if size > signed["artifact_bytes"]:
                    raise UpdateError("staged artifact exceeds signed size")
                target.write(block)
        if size != signed["artifact_bytes"] or sha256_file(snapshot) != signed["artifact_sha256"]:
            raise UpdateError("staged artifact changed after verification")
        with tarfile.open(snapshot, "r:gz") as archive:
            members = archive.getmembers()
            expanded_size = sum(item.size for item in members)
            if len(members) > 100_000 or expanded_size > 4 * MAX_ARTIFACT_BYTES:
                raise UpdateError("release archive expands beyond accepted limits")
            for member in members:
                _safe_member(member, release_root)
            names = [str(PurePosixPath(member.name)) for member in members]
            if len(set(names)) != len(names):
                raise UpdateError("archive contains duplicate paths")
            links = {str(PurePosixPath(member.name)) for member in members
                     if member.issym() or member.islnk()}
            if any(str(parent) in links for member in members for parent in PurePosixPath(member.name).parents):
                raise UpdateError("archive entry traverses a link")
            # Python 3.10 has no extraction_filter; validation above is mandatory.
            archive.extractall(extraction, members=members)
        extracted = extraction / release_root
        metadata_path = extracted / "metadata/release.json"
        if not (extracted / "install/setup.bash").is_file() or not metadata_path.is_file():
            raise UpdateError("release runtime or metadata is missing")
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        qualified_development = (
            metadata.get("channel") == "development"
            and metadata.get("activation_authorized") is False
            and "qualification" in signed
        )
        if (
            metadata.get("product") != PRODUCT
            or metadata.get("release_version") != version
            or not (qualified_development or (
                metadata.get("channel") == "production" and metadata.get("activation_authorized") is True))
        ):
            raise UpdateError("release metadata does not authorize production activation")
        if (extracted / "metadata/DO_NOT_ACTIVATE").exists() and not qualified_development:
            raise UpdateError("release contains an activation blocker")
        for directory, _, files in os.walk(extracted):
            for name in files:
                path = Path(directory) / name
                if not path.is_symlink():
                    with path.open("rb") as stream:
                        os.fsync(stream.fileno())
            _sync_directory(Path(directory))
        os.replace(extracted, final)
        _sync_directory(releases_root)
        return final
    finally:
        shutil.rmtree(extraction, ignore_errors=True)


def switch_current(current: Path, release: Path) -> Optional[str]:
    """Atomically point current at a verified release and return the old target."""
    if not release.is_dir() or release.is_symlink():
        raise UpdateError("new release path is invalid")
    old_target: Optional[str] = None
    if current.is_symlink():
        old_target = os.readlink(current)
    elif current.exists():
        raise UpdateError("current release pointer is not a symlink")
    temporary = current.with_name(f".current.{os.getpid()}")
    temporary.unlink(missing_ok=True)
    temporary.symlink_to(release)
    os.replace(temporary, current)
    _sync_directory(current.parent)
    return old_target


def _direct_release(path: Path, releases_root: Path) -> Path:
    """Resolve one immutable direct child of the release directory."""
    try:
        resolved = path.resolve(strict=True)
        root = releases_root.resolve(strict=True)
    except OSError as error:
        raise UpdateError(f"release path is unavailable: {error}") from error
    if resolved.parent != root or not resolved.is_dir() or resolved.is_symlink():
        raise UpdateError("activation release is outside the immutable release root")
    return resolved


def prepare_activation(
    version: str,
    state_root: Path,
    releases_root: Path,
    current: Path,
    release: Path,
) -> Path:
    """Persist rollback intent before changing the active release pointer."""
    if VERSION.fullmatch(version) is None:
        raise UpdateError("unsafe activation version")
    journal = state_root / "activation.json"
    if journal.exists():
        raise UpdateError("an earlier activation still requires recovery")
    if not current.is_symlink():
        raise UpdateError("current release pointer is not a symlink")
    previous = _direct_release(current, releases_root)
    selected = _direct_release(release, releases_root)
    if selected.name != version:
        raise UpdateError("activation version does not match release directory")
    _atomic_json(
        journal,
        {
            "schema_version": 1,
            "version": version,
            "state": "PREPARED",
            "previous_release": str(previous),
            "selected_release": str(selected),
            "created_at_ns": time.time_ns(),
        },
    )
    return previous


def finalize_activation(
    state_root: Path, outcome: str, detail: str = ""
) -> dict[str, Any]:
    """Move an activation journal into immutable update history."""
    accepted = {
        "ACTIVE_HEALTHY",
        "ROLLED_BACK_FAILED",
        "ROLLED_BACK_INTERRUPTED",
    }
    if outcome not in accepted:
        raise UpdateError("unsupported activation outcome")
    journal = state_root / "activation.json"
    if journal.is_symlink() or not journal.is_file():
        raise UpdateError("activation journal is unavailable")
    try:
        document = json.loads(journal.read_text(encoding="utf-8"))
        version = str(document["version"])
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        raise UpdateError("activation journal is invalid") from error
    if VERSION.fullmatch(version) is None:
        raise UpdateError("activation journal version is unsafe")
    document.update(
        {
            "state": outcome,
            "detail": detail[:512],
            "completed_at_ns": time.time_ns(),
        }
    )
    status_path = state_root / "staged" / version / "status.json"
    if status_path.is_file() and not status_path.is_symlink():
        status_stat = status_path.stat()
        try:
            status = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise UpdateError("staged activation status is invalid") from error
        status.update(
            {
                "state": outcome,
                "activation_approved": outcome == "ACTIVE_HEALTHY",
                "activation_detail": detail[:512],
            }
        )
        _atomic_json(
            status_path,
            status,
            owner=(status_stat.st_uid, status_stat.st_gid),
        )
    history = state_root / "history"
    history.mkdir(mode=0o700, parents=True, exist_ok=True)
    os.chmod(history, 0o700)
    history_path = history / f"{document['completed_at_ns']}-{version}.json"
    _atomic_json(history_path, document)
    journal.unlink()
    _sync_directory(state_root)
    return document


def recover_interrupted_activation(
    state_root: Path, releases_root: Path, current: Path
) -> str:
    """Rollback a release switch left incomplete by power or process loss."""
    journal = state_root / "activation.json"
    if not journal.exists():
        return "no interrupted activation"
    if journal.is_symlink() or not journal.is_file():
        raise UpdateError("activation journal is unsafe")
    try:
        document = json.loads(journal.read_text(encoding="utf-8"))
        if document.get("schema_version") != 1 or document.get("state") != "PREPARED":
            raise ValueError("unsupported journal state")
        previous = _direct_release(
            Path(str(document["previous_release"])), releases_root
        )
        selected = _direct_release(
            Path(str(document["selected_release"])), releases_root
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
        raise UpdateError("activation journal is invalid") from error
    if not current.is_symlink():
        raise UpdateError("current release pointer is not a symlink")
    active = _direct_release(current, releases_root)
    if active == selected:
        switch_current(current, previous)
    elif active != previous:
        raise UpdateError("active release does not match the recovery journal")
    finalize_activation(
        state_root,
        "ROLLED_BACK_INTERRUPTED",
        "boot recovered an incomplete activation",
    )
    return f"rolled back interrupted activation to {previous.name}"
