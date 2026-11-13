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


def production_archive(version: str, development=False) -> bytes:
    files = {
        f"opt/studica/releases/{version}/install/setup.bash": b"#!/bin/bash\n",
        f"opt/studica/releases/{version}/metadata/release.json": json.dumps(
            {
                "product": "studica-robot",
                "release_version": version,
                "channel": "development" if development else "production",
                "activation_authorized": not development,
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


def signed_envelope(tmp_path: Path, version="1.0.0", development=False, qualified=False):
    artifact = production_archive(version, development)
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
    if qualified:
        signed["qualification"] = {
            "schema_version": 1, "tested_release_sha256": signed["artifact_sha256"],
            "cold_boots_passed": 50, "independent_torque_removal": True,
            "failure_injection_passed": True, "zero_motion_all_boots": True}
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


def test_existing_release_is_not_trusted(tmp_path):
    artifact, public, envelope = signed_envelope(tmp_path)
    staged = tmp_path / "state/staged/1.0.0"
    staged.mkdir(parents=True)
    (staged / "release.tar.gz").write_bytes(artifact)
    (staged / "manifest.json").write_text(json.dumps(envelope))
    (tmp_path / "releases/1.0.0").mkdir(parents=True)
    with pytest.raises(UpdateError, match="already exists"):
        extract_verified_release("1.0.0", tmp_path / "state", tmp_path / "releases", public)


def test_qualification_is_bound_to_digest_and_all_acceptance_gates():
    from studica_robot_platform.updates import validate_signed_qualification
    report = {"schema_version": 1, "tested_release_sha256": "a" * 64, "cold_boots_passed": 50,
              "independent_torque_removal": True, "failure_injection_passed": True,
              "zero_motion_all_boots": True}
    validate_signed_qualification(report, "a" * 64)
    with pytest.raises(UpdateError):
        validate_signed_qualification(report, "b" * 64)
    for field in ("independent_torque_removal", "failure_injection_passed", "zero_motion_all_boots"):
        with pytest.raises(UpdateError):
            validate_signed_qualification(dict(report, **{field: False}), "a" * 64)


def test_archive_links_cannot_escape_or_replace_ancestors():
    from studica_robot_platform.updates import _safe_member
    root = "opt/studica/releases/1.0.0"
    member = tarfile.TarInfo("opt")
    member.type = tarfile.SYMTYPE
    member.linkname = "elsewhere"
    with pytest.raises(UpdateError):
        _safe_member(member, root)
    member.name = root + "/install/link"
    member.linkname = "../../../../outside"
    with pytest.raises(UpdateError):
        _safe_member(member, root)


@pytest.mark.parametrize("qualified", [False, True])
def test_development_bundle_requires_signed_qualification(tmp_path, qualified):
    artifact, public, envelope = signed_envelope(tmp_path, development=True, qualified=qualified)
    staged = tmp_path / "state/staged/1.0.0"
    staged.mkdir(parents=True)
    (staged / "release.tar.gz").write_bytes(artifact)
    (staged / "manifest.json").write_text(json.dumps(envelope))
    if qualified:
        release = extract_verified_release("1.0.0", tmp_path / "state", tmp_path / "releases", public)
        assert (release / "install/setup.bash").is_file()
    else:
        with pytest.raises(UpdateError, match="does not authorize"):
            extract_verified_release("1.0.0", tmp_path / "state", tmp_path / "releases", public)


def test_corrupt_staged_archive_is_rejected(tmp_path):
    artifact, public, envelope = signed_envelope(tmp_path)
    staged = tmp_path / "state/staged/1.0.0"
    staged.mkdir(parents=True)
    (staged / "release.tar.gz").write_bytes(artifact[:-1] + bytes([artifact[-1] ^ 1]))
    (staged / "manifest.json").write_text(json.dumps(envelope))
    with pytest.raises(UpdateError, match="changed after verification"):
        extract_verified_release("1.0.0", tmp_path / "state", tmp_path / "releases", public)
    assert not (tmp_path / "releases/1.0.0").exists()


@pytest.mark.parametrize('document', [[], None, {'schema_version': True, 'state': 'PREPARED'}])
def test_malformed_recovery_journal_never_changes_pointer(tmp_path, document):
    state = tmp_path / 'state'
    state.mkdir()
    journal = state / 'activation.json'
    journal.write_text(json.dumps(document))
    current = tmp_path / 'current'
    current.symlink_to(tmp_path / 'unchanged')
    with pytest.raises(UpdateError, match='journal is invalid'):
        recover_interrupted_activation(state, tmp_path / 'releases', current)
    assert current.readlink() == tmp_path / 'unchanged'
    assert journal.exists()


def test_dangling_journal_blocks_recovery_and_new_activation(tmp_path):
    state = tmp_path / 'state'
    state.mkdir()
    journal = state / 'activation.json'
    journal.symlink_to(tmp_path / 'missing')
    with pytest.raises(UpdateError, match='unsafe'):
        recover_interrupted_activation(state, tmp_path / 'releases', tmp_path / 'current')
    with pytest.raises(UpdateError, match='earlier activation'):
        prepare_activation('2.0.0', state, tmp_path / 'releases',
                           tmp_path / 'current', tmp_path / 'releases/2.0.0')
    assert journal.is_symlink()


@pytest.mark.parametrize('version', [None, 207, '../escape', '1.0.0', ''])
def test_invalid_journal_version_rejected_before_pointer_change(tmp_path, version):
    from studica_robot_platform.updates import finalize_activation
    state, releases = tmp_path / 'state', tmp_path / 'releases'
    previous, selected = releases / '1.0.0', releases / '2.0.0'
    previous.mkdir(parents=True)
    selected.mkdir()
    current = tmp_path / 'current'
    current.symlink_to(previous)
    prepare_activation('2.0.0', state, releases, current, selected)
    switch_current(current, selected)
    journal = state / 'activation.json'
    document = json.loads(journal.read_text())
    document['version'] = version
    journal.write_text(json.dumps(document))
    evidence = journal.read_bytes()
    with pytest.raises(UpdateError, match='journal is invalid'):
        recover_interrupted_activation(state, releases, current)
    with pytest.raises(UpdateError, match='journal is invalid'):
        finalize_activation(state, 'ACTIVE_HEALTHY')
    assert current.resolve() == selected
    assert journal.read_bytes() == evidence
    assert not (state / 'history').exists()


@pytest.mark.parametrize('boundary', ['prepared', 'switched', 'history', 'finalized'])
def test_process_exit_recovery_boundaries(tmp_path, boundary):
    """Real child exits and filesystem journals; no ROS or service operations."""
    import os
    import subprocess
    import sys

    state, releases = tmp_path / 'state', tmp_path / 'releases'
    previous, selected = releases / '1.0.0', releases / '2.0.0'
    previous.mkdir(parents=True)
    selected.mkdir()
    current = tmp_path / 'current'
    current.symlink_to(previous)
    source = Path(__file__).resolve().parents[1]
    child = '''
import os
from pathlib import Path
import sys
from studica_robot_platform import updates as u
root, boundary = Path(sys.argv[1]), sys.argv[2]
state, releases, current = root / 'state', root / 'releases', root / 'current'
u.prepare_activation('2.0.0', state, releases, current, releases / '2.0.0')
if boundary == 'prepared':
    os._exit(73)
u.switch_current(current, releases / '2.0.0')
if boundary == 'switched':
    os._exit(73)
write = u._atomic_json
def interrupted_write(path, *args, **kwargs):
    write(path, *args, **kwargs)
    if path.parent.name == 'history':
        os._exit(73)
if boundary == 'history':
    u._atomic_json = interrupted_write
u.finalize_activation(state, 'ACTIVE_HEALTHY')
os._exit(73)
'''
    result = subprocess.run(
        [sys.executable, '-c', child, str(tmp_path), boundary],
        env=dict(os.environ, PYTHONPATH=str(source)), capture_output=True, timeout=10)
    assert result.returncode == 73, result.stderr.decode()
    outcome = recover_interrupted_activation(state, releases, current)
    expected = selected if boundary == 'finalized' else previous
    assert current.resolve() == expected
    assert not (state / 'activation.json').exists()
    history = [json.loads(path.read_text()) for path in (state / 'history').glob('*.json')]
    if boundary == 'finalized':
        assert outcome == 'no interrupted activation'
        assert [item['state'] for item in history] == ['ACTIVE_HEALTHY']
    else:
        assert 'rolled back interrupted activation' in outcome
        assert any(item['state'] == 'ROLLED_BACK_INTERRUPTED' for item in history)
    before = {path.name: path.read_bytes() for path in (state / 'history').glob('*.json')}
    assert recover_interrupted_activation(state, releases, current) == 'no interrupted activation'
    assert before == {path.name: path.read_bytes() for path in (state / 'history').glob('*.json')}


@pytest.mark.parametrize('status', [[], None])
def test_invalid_staged_status_retains_activation_journal(tmp_path, status):
    from studica_robot_platform.updates import finalize_activation
    state, releases = tmp_path / 'state', tmp_path / 'releases'
    previous, selected = releases / '1.0.0', releases / '2.0.0'
    previous.mkdir(parents=True)
    selected.mkdir()
    current = tmp_path / 'current'
    current.symlink_to(previous)
    prepare_activation('2.0.0', state, releases, current, selected)
    status_path = state / 'staged/2.0.0/status.json'
    status_path.parent.mkdir(parents=True)
    status_path.write_text(json.dumps(status))
    with pytest.raises(UpdateError, match='staged activation status is invalid'):
        finalize_activation(state, 'ACTIVE_HEALTHY')
    assert (state / 'activation.json').exists()
    assert not (state / 'history').exists()
    assert json.loads(status_path.read_text()) == status


@pytest.mark.parametrize('outcome', ['ACTIVE_HEALTHY', 'ROLLED_BACK_FAILED',
                                     'ROLLED_BACK_INTERRUPTED'])
@pytest.mark.parametrize('boundary', ['terminal', 'unlock', 'unlock_synced', 'receipt_removed'])
def test_completion_receipt_survives_process_exit(tmp_path, outcome, boundary):
    import os
    import subprocess
    import sys
    from studica_robot_platform.updates import (
        complete_activation_cleanup, validate_maintenance_lock)

    source = Path(__file__).resolve().parents[1]
    child = '''
import os
from pathlib import Path
import sys
from studica_robot_platform import updates as u
root, outcome, boundary = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
state, releases, current = root / 'state', root / 'releases', root / 'current'
old, new, lock = releases / 'old', releases / 'new', root / 'maintenance'
old.mkdir(parents=True)
new.mkdir()
current.symlink_to(old)
u.prepare_activation('new', state, releases, current, new)
u.create_maintenance_lock(state, lock)
if outcome == 'ACTIVE_HEALTHY':
    u.switch_current(current, new)
u.finalize_activation(state, outcome, retain_completion=True)
if boundary == 'terminal':
    os._exit(73)
unlink, sync = Path.unlink, u._sync_directory
def interrupted_unlink(path, *args, **kwargs):
    unlink(path, *args, **kwargs)
    if ((boundary == 'unlock' and path == lock)
            or (boundary == 'receipt_removed' and path == state / 'activation.json')):
        os._exit(73)
def interrupted_sync(path):
    sync(path)
    if boundary == 'unlock_synced' and path == lock.parent:
        os._exit(73)
Path.unlink, u._sync_directory = interrupted_unlink, interrupted_sync
u.complete_activation_cleanup(state, releases, current, lock)
raise AssertionError('boundary not reached')
'''
    result = subprocess.run(
        [sys.executable, '-c', child, str(tmp_path), outcome, boundary],
        env=dict(os.environ, PYTHONPATH=str(source)), capture_output=True, timeout=10)
    assert result.returncode == 73, result.stderr.decode()
    state, releases = tmp_path / 'state', tmp_path / 'releases'
    current, lock = tmp_path / 'current', tmp_path / 'maintenance'
    history_before = {p.name: p.read_bytes() for p in (state / 'history').glob('*.json')}
    validate_maintenance_lock(state, lock)
    recovered = recover_interrupted_activation(state, releases, current, retain_completion=True)
    if recovered != 'no interrupted activation':
        complete_activation_cleanup(state, releases, current, lock)
    assert current.resolve() == releases / ('new' if outcome == 'ACTIVE_HEALTHY' else 'old')
    assert not lock.exists()
    assert not (state / 'activation.json').exists()
    assert history_before == {p.name: p.read_bytes() for p in (state / 'history').glob('*.json')}
    assert recover_interrupted_activation(
        state, releases, current, retain_completion=True) == 'no interrupted activation'


@pytest.mark.parametrize('damage', ['wrong_lock', 'empty_lock', 'wrong_pointer', 'bad_receipt',
                                    'symlink_receipt', 'symlink_lock'])
def test_completion_cleanup_rejects_unbound_evidence(tmp_path, damage):
    from studica_robot_platform.updates import (
        create_maintenance_lock, complete_activation_cleanup, finalize_activation)
    state, releases = tmp_path / 'state', tmp_path / 'releases'
    old, new = releases / 'old', releases / 'new'
    old.mkdir(parents=True)
    new.mkdir()
    current, lock = tmp_path / 'current', tmp_path / 'maintenance'
    current.symlink_to(old)
    prepare_activation('new', state, releases, current, new)
    create_maintenance_lock(state, lock)
    switch_current(current, new)
    finalize_activation(state, 'ACTIVE_HEALTHY', retain_completion=True)
    journal = state / 'activation.json'
    if damage == 'wrong_lock':
        lock.write_text(json.dumps({'transaction_id': '0' * 32}))
    elif damage == 'empty_lock':
        lock.write_text('')
    elif damage == 'wrong_pointer':
        switch_current(current, old)
    elif damage == 'bad_receipt':
        journal.write_text('{}')
    else:
        target = journal if damage == 'symlink_receipt' else lock
        saved = tmp_path / 'saved'
        target.rename(saved)
        target.symlink_to(saved)
    before = current.readlink()
    with pytest.raises(UpdateError):
        complete_activation_cleanup(state, releases, current, lock)
    assert current.readlink() == before
    assert lock.exists()
    assert journal.exists()
