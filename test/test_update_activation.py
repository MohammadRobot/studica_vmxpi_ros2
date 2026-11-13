"""Fault injection for managed activation without touching host services."""

import importlib.util
from pathlib import Path

import pytest

from studica_robot_platform.updates import UpdateError

SPEC = importlib.util.spec_from_file_location(
    "update_agent_test", Path(__file__).resolve().parents[1] / "scripts/studica_update_agent.py")
AGENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AGENT)


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    releases = tmp_path / "releases"
    old, new = releases / "old", releases / "new"
    old.mkdir(parents=True)
    new.mkdir()
    current = tmp_path / "current"
    current.symlink_to(old)
    lock = tmp_path / "maintenance"
    monkeypatch.setattr(AGENT, "CURRENT_RELEASE", current)
    monkeypatch.setattr(AGENT, "RELEASES_ROOT", releases)
    monkeypatch.setattr(AGENT, "MAINTENANCE_LOCK", lock)
    monkeypatch.setattr(AGENT, "extract_verified_release", lambda *args: new)
    events = []
    monkeypatch.setattr(AGENT, "_systemctl", lambda verb, unit: events.append(verb))
    finalize = AGENT.finalize_activation

    def record_finalize(*args, **kwargs):
        result = finalize(*args, **kwargs)
        events.append(args[1])
        return result

    monkeypatch.setattr(AGENT, "finalize_activation", record_finalize)
    return current, old, new, lock, events


def test_armed_activation_is_rejected_before_stopping(runtime, monkeypatch, tmp_path):
    current, old, _, lock, events = runtime
    monkeypatch.setattr(AGENT, "_wait_disarmed", lambda *args, **kwargs: False)
    with pytest.raises(UpdateError, match="requires IDLE"):
        AGENT.activate("new", tmp_path, tmp_path / "key")
    assert current.resolve() == old
    assert not lock.exists()
    assert events == []


def test_candidate_health_failure_rolls_back_disarmed(runtime, monkeypatch, tmp_path):
    current, old, _, lock, events = runtime
    health = iter([True, True, False, True])
    monkeypatch.setattr(AGENT, "_wait_disarmed", lambda *args, **kwargs: next(health))
    with pytest.raises(UpdateError):
        AGENT.activate("new", tmp_path, tmp_path / "key")
    assert current.resolve() == old
    assert events == ["stop", "start", "stop", "start", "ROLLED_BACK_FAILED"]
    assert not lock.exists()


def test_failed_candidate_stop_retains_pointer_and_maintenance(runtime, monkeypatch, tmp_path):
    current, _, new, lock, events = runtime
    health = iter([True, True, False])
    monkeypatch.setattr(AGENT, "_wait_disarmed", lambda *args, **kwargs: next(health))

    def systemctl(verb, unit):
        events.append(verb)
        if events == ["stop", "start", "stop"]:
            raise UpdateError("stop failed")

    monkeypatch.setattr(AGENT, "_systemctl", systemctl)
    with pytest.raises(UpdateError, match="journal retained"):
        AGENT.activate("new", tmp_path, tmp_path / "key")
    assert current.resolve() == new
    assert lock.exists()
    assert events == ["stop", "start", "stop"]


def test_success_removes_maintenance_only_after_health(runtime, monkeypatch, tmp_path):
    current, _, new, lock, events = runtime
    calls = []

    def health(*args, **kwargs):
        calls.append(lock.exists())
        return True

    monkeypatch.setattr(AGENT, "_wait_disarmed", health)
    AGENT.activate("new", tmp_path, tmp_path / "key")
    assert calls == [False, True, True]
    assert current.resolve() == new
    assert events == ["stop", "start", "ACTIVE_HEALTHY"]
    assert not lock.exists()


def test_minimal_activation_uses_only_minimal_service(runtime, monkeypatch, tmp_path):
    _, _, _, _, events = runtime
    checks = []
    units = []

    def health(*args, **kwargs):
        checks.append(kwargs['runtime'])
        return True

    monkeypatch.setattr(AGENT, '_wait_disarmed', health)
    monkeypatch.setattr(AGENT, '_systemctl', lambda verb, unit: units.append((verb, unit)))
    AGENT.activate('new', tmp_path, tmp_path / 'key', runtime='minimal')
    assert checks == ['minimal'] * 3
    assert units == [('stop', 'studica-minimal.service'), ('start', 'studica-minimal.service')]
    assert events == ['ACTIVE_HEALTHY']


def test_existing_lock_is_not_removed_or_services_stopped(runtime, monkeypatch, tmp_path):
    _, _, _, lock, events = runtime
    lock.touch()
    monkeypatch.setattr(AGENT, '_wait_disarmed', lambda *args, **kwargs: True)
    with pytest.raises(UpdateError, match='existing maintenance'):
        AGENT.activate('new', tmp_path, tmp_path / 'key', runtime='minimal')
    assert lock.exists()
    assert events == []


def test_boot_recovery_does_not_source_the_candidate_release():
    # Regression for recovery failing to import when the selected release is broken.
    root = Path(__file__).resolve().parents[1]
    unit = (root / 'deployment/minimal/studica-minimal-recovery.service').read_text()
    command = next(line for line in unit.splitlines() if line.startswith('ExecStart='))
    assert '/opt/studica/maintenance/install/setup.bash' in command
    assert '/opt/studica/current' not in command


@pytest.mark.parametrize('active', ['active', 'activating', 'deactivating', 'unknown', ''])
def test_recovery_never_changes_running_or_unknown_runtime(runtime, monkeypatch, tmp_path, active):
    from types import SimpleNamespace
    current, old, _, lock, _ = runtime
    lock.touch()
    monkeypatch.setattr(AGENT.subprocess, 'run', lambda *a, **kw: SimpleNamespace(stdout=active))
    with pytest.raises(UpdateError, match='runtime to be stopped'):
        AGENT.recover(tmp_path, 'minimal')
    assert lock.exists()
    assert current.resolve() == old


@pytest.mark.parametrize('symlink', [False, True])
def test_recovery_preserves_unexplained_maintenance(runtime, monkeypatch, tmp_path, symlink):
    from types import SimpleNamespace
    current, old, _, lock, _ = runtime
    if symlink:
        lock.symlink_to(tmp_path / 'missing')
    else:
        lock.touch()
    monkeypatch.setattr(AGENT.subprocess, 'run', lambda *a, **kw: SimpleNamespace(stdout='inactive'))
    with pytest.raises(UpdateError, match='manual investigation'):
        AGENT.recover(tmp_path, 'minimal')
    assert lock.is_symlink() if symlink else lock.exists()
    assert current.resolve() == old


def test_recovery_clears_lock_only_after_real_journal_recovery(runtime, monkeypatch, tmp_path):
    from types import SimpleNamespace
    from studica_robot_platform.updates import prepare_activation, switch_current
    current, old, new, lock, _ = runtime
    prepare_activation('new', tmp_path, old.parent, current, new)
    switch_current(current, new)
    AGENT.create_maintenance_lock(tmp_path, lock)
    calls = []

    def stopped(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(stdout='inactive')

    monkeypatch.setattr(AGENT.subprocess, 'run', stopped)
    assert 'rolled back' in AGENT.recover(tmp_path, 'minimal')
    assert current.resolve() == old
    assert not lock.exists()
    assert not (tmp_path / 'activation.json').exists()
    assert calls == [['/usr/bin/systemctl', 'show', '--property=ActiveState', '--value',
                      'studica-minimal.service']]


def test_cleanup_failure_never_rolls_back_committed_activation(runtime, monkeypatch, tmp_path):
    from types import SimpleNamespace
    current, _, new, lock, events = runtime
    monkeypatch.setattr(AGENT, '_wait_disarmed', lambda *a, **kw: True)
    cleanup = AGENT.complete_activation_cleanup

    def fail(*args):
        raise OSError('injected cleanup failure')

    monkeypatch.setattr(AGENT, 'complete_activation_cleanup', fail)
    with pytest.raises(OSError, match='injected cleanup'):
        AGENT.activate('new', tmp_path, tmp_path / 'key', runtime='minimal')
    assert current.resolve() == new
    assert lock.exists()
    assert events == ['stop', 'start', 'ACTIVE_HEALTHY']
    monkeypatch.setattr(AGENT, 'complete_activation_cleanup', cleanup)
    monkeypatch.setattr(AGENT.subprocess, 'run', lambda *a, **kw: SimpleNamespace(stdout='inactive'))
    assert AGENT.recover(tmp_path, 'minimal') == 'completed activation cleanup pending'
    assert current.resolve() == new
    assert not lock.exists()
    assert not (tmp_path / 'activation.json').exists()


def test_error_after_terminal_write_does_not_rollback(runtime, monkeypatch, tmp_path):
    current, _, new, lock, events = runtime
    monkeypatch.setattr(AGENT, '_wait_disarmed', lambda *a, **kw: True)
    finalize = AGENT.finalize_activation

    def committed_then_failed(*args, **kwargs):
        finalize(*args, **kwargs)
        raise OSError('injected finalization sync failure')

    monkeypatch.setattr(AGENT, 'finalize_activation', committed_then_failed)
    with pytest.raises(UpdateError, match='cleanup requires recovery'):
        AGENT.activate('new', tmp_path, tmp_path / 'key', runtime='minimal')
    assert events == ['stop', 'start', 'ACTIVE_HEALTHY']
    assert current.resolve() == new
    assert lock.exists()


def test_failed_unlock_sync_retains_receipt_and_retries(runtime, monkeypatch, tmp_path):
    from types import SimpleNamespace
    from studica_robot_platform import updates
    current, _, new, lock, _ = runtime
    monkeypatch.setattr(AGENT, '_wait_disarmed', lambda *a, **kw: True)
    sync = updates._sync_directory
    observed = []

    def fail_unlock_sync(path):
        if path == lock.parent and not lock.exists():
            observed.append((path, (tmp_path / 'activation.json').exists()))
            raise OSError('injected unlock sync failure')
        sync(path)

    # Inject only during cleanup; preparation also syncs this parent directory.
    cleanup = AGENT.complete_activation_cleanup

    def interrupted_cleanup(*args):
        with monkeypatch.context() as patch:
            patch.setattr(updates, '_sync_directory', fail_unlock_sync)
            cleanup(*args)

    monkeypatch.setattr(AGENT, 'complete_activation_cleanup', interrupted_cleanup)
    with pytest.raises(OSError, match='unlock sync failure'):
        AGENT.activate('new', tmp_path, tmp_path / 'key', runtime='minimal')
    assert observed == [(lock.parent, True)]
    assert current.resolve() == new
    assert (tmp_path / 'activation.json').exists()
    monkeypatch.setattr(AGENT, 'complete_activation_cleanup', cleanup)
    monkeypatch.setattr(AGENT.subprocess, 'run', lambda *a, **kw: SimpleNamespace(stdout='inactive'))
    AGENT.recover(tmp_path, 'minimal')
    assert not (tmp_path / 'activation.json').exists()
    assert not lock.exists()
