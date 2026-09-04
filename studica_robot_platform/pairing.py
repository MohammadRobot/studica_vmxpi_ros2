"""One-time companion pairing and scoped token persistence."""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import time
from typing import Any


COMPANION_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")
PAIRING_CODE = re.compile(r"[0-9]{8}")


class PairingError(ValueError):
    """A companion pairing request is invalid, expired, or already used."""


def _atomic_json(path: Path, document: Any) -> None:
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        os.chmod(temporary, 0o600)
        json.dump(document, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class CompanionPairingStore:
    """Issue one-use codes and validate hash-only companion credentials."""

    def __init__(self, root: Path) -> None:
        if not root.is_absolute():
            raise PairingError("pairing root must be absolute")
        root.mkdir(parents=True, exist_ok=True)
        if root.is_symlink() or not root.is_dir():
            raise PairingError("pairing root must be a real directory")
        os.chmod(root, 0o700)
        self.root = root.resolve()
        self._code_path = self.root / "one-time-code.json"
        self._tokens_path = self.root / "companion-tokens.json"

    def issue_code(self, lifetime_sec: int = 600) -> dict[str, Any]:
        """Replace any previous code and return a short-lived plaintext code."""
        if not 60 <= lifetime_sec <= 1800:
            raise PairingError("pairing code lifetime must be 60 to 1800 seconds")
        code = f"{secrets.randbelow(100_000_000):08d}"
        expires_at = int(time.time()) + lifetime_sec
        _atomic_json(
            self._code_path,
            {
                "schema_version": 1,
                "code_sha256": hashlib.sha256(code.encode("ascii")).hexdigest(),
                "expires_at": expires_at,
            },
        )
        return {"code": code, "expires_at": expires_at}

    def _tokens(self) -> list[dict[str, Any]]:
        if not self._tokens_path.exists():
            return []
        try:
            value = json.loads(self._tokens_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise PairingError("companion token store is invalid") from error
        if not isinstance(value, dict) or not isinstance(value.get("tokens"), list):
            raise PairingError("companion token store is invalid")
        return [item for item in value["tokens"] if isinstance(item, dict)]

    def validate_code(self, code: str, companion_id: str) -> None:
        """Validate a pending code without consuming it."""
        if PAIRING_CODE.fullmatch(code) is None:
            raise PairingError("pairing code is invalid")
        if COMPANION_ID.fullmatch(companion_id) is None:
            raise PairingError("companion_id is invalid")
        try:
            value = json.loads(self._code_path.read_text(encoding="utf-8"))
            expected = str(value["code_sha256"])
            expires_at = int(value["expires_at"])
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise PairingError("pairing code is unavailable") from error
        actual = hashlib.sha256(code.encode("ascii")).hexdigest()
        if time.time() > expires_at or not hmac.compare_digest(actual, expected):
            raise PairingError("pairing code is invalid or expired")

    def redeem(self, code: str, companion_id: str, peer_address: str) -> str:
        """Consume a valid code and return the companion token exactly once."""
        self.validate_code(code, companion_id)
        try:
            peer_address = str(ipaddress.ip_address(peer_address))
        except ValueError as error:
            raise PairingError("companion peer address is invalid") from error

        token = secrets.token_urlsafe(32)
        tokens = [
            {
                "companion_id": companion_id,
                "created_at": int(time.time()),
                "peer_address": peer_address,
                "token_sha256": hashlib.sha256(token.encode("ascii")).hexdigest(),
            }
        ]
        _atomic_json(
            self._tokens_path,
            {"schema_version": 1, "tokens": tokens},
        )
        self._code_path.unlink(missing_ok=True)
        return token

    def validate_token(self, token: str) -> bool:
        """Validate without persisting or exposing a plaintext credential."""
        if len(token) < 24:
            return False
        digest = hashlib.sha256(token.encode("utf-8")).hexdigest()
        try:
            tokens = self._tokens()
        except PairingError:
            return False
        return any(
            hmac.compare_digest(str(item.get("token_sha256", "")), digest)
            for item in tokens
        )

    def revoke_token(self, token: str) -> None:
        """Remove a just-issued token if peer provisioning does not finish."""
        digest = hashlib.sha256(token.encode("utf-8")).hexdigest()
        tokens = self._tokens()
        retained = [
            item
            for item in tokens
            if not hmac.compare_digest(
                str(item.get("token_sha256", "")), digest
            )
        ]
        if len(retained) != len(tokens):
            _atomic_json(
                self._tokens_path,
                {"schema_version": 1, "tokens": retained},
            )
