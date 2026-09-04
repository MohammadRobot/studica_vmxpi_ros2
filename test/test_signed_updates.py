import base64
import hashlib
import io
import json
from pathlib import Path
import tarfile

import pytest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from studica_robot_platform.updates import (
    PLATFORM,
    UpdateError,
    canonical_json,
    extract_verified_release,
    prepare_activation,
    recover_interrupted_activation,
    switch_current,
    verify_manifest,
)


def production_archive(version: str) -> bytes:
    files = {
        f"opt/studica/releases/{version}/install/setup.bash": b"#!/bin/bash\n",
        f"opt/studica/releases/{version}/metadata/release.json": json.dumps(
            {
                "product": "studica-robot",
                "release_version": version,
                "channel": "production",
                "activation_authorized": True,
            }
        ).encode(),
    }
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for directory in (
            ".",
            "opt",
            "opt/studica",
            "opt/studica/releases",
            f"opt/studica/releases/{version}",
            f"opt/studica/releases/{version}/install",
            f"opt/studica/releases/{version}/metadata",
        ):
            item = tarfile.TarInfo(directory)
            item.type = tarfile.DIRTYPE
            item.mode = 0o755
            item.uid = item.gid = 0
            archive.addfile(item)
        for name, value in files.items():
            item = tarfile.TarInfo(name)
            item.size = len(value)
            item.mode = 0o644
            item.uid = item.gid = 0
            archive.addfile(item, io.BytesIO(value))
    return output.getvalue()


def signed_envelope(tmp_path: Path, version="1.0.0"):
    artifact = production_archive(version)
    private = Ed25519PrivateKey.generate()
    public_path = tmp_path / "public.pem"
    public_path.write_bytes(
        private.public_key().public_bytes(
            serialization.Encoding.PEM,
            serialization.PublicFormat.SubjectPublicKeyInfo,
        )
    )
    signed = {
        "artifact_bytes": len(artifact),
        "artifact_sha256": hashlib.sha256(artifact).hexdigest(),
        "artifact_url": "https://example.invalid/release.tar.gz",
        "platform": PLATFORM,
        "product": "studica-robot",
        "release_root": f"opt/studica/releases/{version}",
        "version": version,
    }
    envelope = {
        "schema_version": 1,
        "signed": signed,
        "signature": base64.b64encode(private.sign(canonical_json(signed))).decode(),
    }
    return artifact, public_path, envelope


def test_signature_tampering_is_rejected(tmp_path: Path):
    _, public, envelope = signed_envelope(tmp_path)
    assert verify_manifest(envelope, public)["version"] == "1.0.0"
    envelope["signed"]["version"] = "2.0.0"
    with pytest.raises(UpdateError):
        verify_manifest(envelope, public)


def test_verified_release_extract_and_atomic_pointer(tmp_path: Path):
    artifact, public, envelope = signed_envelope(tmp_path)
    staged = tmp_path / "state/staged/1.0.0"
    staged.mkdir(parents=True)
    (staged / "release.tar.gz").write_bytes(artifact)
    (staged / "manifest.json").write_text(json.dumps(envelope))
    release = extract_verified_release(
        "1.0.0", tmp_path / "state", tmp_path / "releases", public
    )
    current = tmp_path / "current"
    assert switch_current(current, release) is None
    assert current.resolve() == release.resolve()


def test_interrupted_activation_rolls_back_from_boot_journal(tmp_path: Path):
    state = tmp_path / "state"
    staged = state / "staged/2.0.0"
    staged.mkdir(parents=True)
    (staged / "status.json").write_text(
        json.dumps(
            {
                "version": "2.0.0",
                "state": "VERIFIED_PENDING_APPROVAL",
                "activation_approved": False,
            }
        ),
        encoding="utf-8",
    )
    releases = tmp_path / "releases"
    previous = releases / "1.0.0"
    selected = releases / "2.0.0"
    previous.mkdir(parents=True)
    selected.mkdir()
    current = tmp_path / "current"
    current.symlink_to(previous)

    assert prepare_activation(
        "2.0.0", state, releases, current, selected
    ) == previous
    switch_current(current, selected)
    result = recover_interrupted_activation(state, releases, current)

    assert "1.0.0" in result
    assert current.resolve() == previous.resolve()
    assert not (state / "activation.json").exists()
    history = list((state / "history").glob("*.json"))
    assert len(history) == 1
    assert json.loads(history[0].read_text())["state"] == "ROLLED_BACK_INTERRUPTED"
    status = json.loads((staged / "status.json").read_text())
    assert status["state"] == "ROLLED_BACK_INTERRUPTED"
    assert status["activation_approved"] is False
