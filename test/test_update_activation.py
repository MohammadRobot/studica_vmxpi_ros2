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
    monkeypatch.setattr(AGENT, "prepare_activation", lambda *args: old)
    events = []
    monkeypatch.setattr(AGENT, "_systemctl", lambda verb, unit: events.append(verb))
    monkeypatch.setattr(AGENT, "finalize_activation", lambda *args: events.append(args[1]))
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
