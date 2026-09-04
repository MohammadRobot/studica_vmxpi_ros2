"""Narrow Unix-socket client for privileged unit orchestration."""

from __future__ import annotations

import json
import socket
from typing import Any, Dict


class OrchestratorError(RuntimeError):
    """The local privileged orchestrator rejected or lost a request."""


def request(
    payload: Dict[str, Any],
    socket_path: str = "/run/studica/orchestrator.sock",
    timeout_sec: float = 1.0,
) -> Dict[str, Any]:
    """Send one bounded JSON request and return its bounded JSON response."""
    encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
    if len(encoded) > 4096:
        raise OrchestratorError("orchestrator request is too large")
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(timeout_sec)
            client.connect(socket_path)
            client.sendall(encoded)
            data = bytearray()
            while b"\n" not in data:
                chunk = client.recv(4096)
                if not chunk:
                    break
                data.extend(chunk)
                if len(data) > 16384:
                    raise OrchestratorError("orchestrator response is too large")
    except OSError as error:
        raise OrchestratorError(f"orchestrator unavailable: {error}") from error
    try:
        response = json.loads(bytes(data).split(b"\n", 1)[0])
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise OrchestratorError("invalid orchestrator response") from error
    if not isinstance(response, dict):
        raise OrchestratorError("invalid orchestrator response type")
    return response
