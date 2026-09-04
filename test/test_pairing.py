from pathlib import Path

import pytest

from studica_robot_platform.pairing import CompanionPairingStore, PairingError


def test_pairing_code_is_one_time_and_token_is_hash_only(tmp_path: Path):
    store = CompanionPairingStore(tmp_path / "pairing")
    issued = store.issue_code()
    assert len(issued["code"]) == 8
    token = store.redeem(issued["code"], "office-pc", "192.0.2.10")
    assert store.validate_token(token)
    assert "192.0.2.10" in (
        tmp_path / "pairing/companion-tokens.json"
    ).read_text()
    assert token not in (tmp_path / "pairing/companion-tokens.json").read_text()
    with pytest.raises(PairingError):
        store.redeem(issued["code"], "second-pc", "192.0.2.10")
    replacement = store.issue_code()
    new_token = store.redeem(
        replacement["code"], "second-pc", "192.0.2.11"
    )
    assert store.validate_token(new_token)
    assert not store.validate_token(token)


def test_pairing_rejects_invalid_code_and_companion_id(tmp_path: Path):
    store = CompanionPairingStore(tmp_path / "pairing")
    issued = store.issue_code()
    with pytest.raises(PairingError):
        store.redeem("00000000", "office-pc", "192.0.2.10")
    with pytest.raises(PairingError):
        store.redeem(issued["code"], "../unsafe", "192.0.2.10")


def test_token_can_be_revoked_after_peer_configuration_failure(tmp_path: Path):
    store = CompanionPairingStore(tmp_path / "pairing")
    code = store.issue_code()["code"]
    token = store.redeem(code, "companion", "192.0.2.10")
    assert store.validate_token(token)
    store.revoke_token(token)
    assert not store.validate_token(token)
