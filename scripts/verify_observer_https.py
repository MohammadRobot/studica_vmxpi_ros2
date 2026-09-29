#!/usr/bin/env python3
"""Verify real observation HTTPS using device CA and credential, never print secrets."""

import json
from pathlib import Path
import ssl
import urllib.error
import urllib.request


def main():
    context = ssl.create_default_context(cafile="/var/lib/studica/tls/robot.crt")
    token = Path("/var/lib/studica/secrets/api-token").read_text().strip()
    base = "https://192.168.1.173"

    def request(path, *, authenticated=False, method="GET", data=None):
        headers = {"Authorization": f"Bearer {token}"} if authenticated else {}
        if data is not None:
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(base + path, headers=headers, method=method,
                                     data=json.dumps(data).encode() if data is not None else None)
        try:
            with urllib.request.urlopen(req, context=context, timeout=8) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    status, body = request("/api/v1/health")
    health = json.loads(body)
    assert status == 200 and health["read_only"] and not health["ros_ready"]
    assert request("/api/v1/status")[0] == 401
    status, body = request("/api/v1/status", authenticated=True)
    state = json.loads(body)
    assert status == 200 and state["read_only"] and state["armed"] is None and not state["ready"]
    assert state["safety_state"] == "NOT_MONITORED"
    assert state["sensors"]["lidar"]["healthy"]
    for method, path in (
        ("PUT", "/api/v1/mode"), ("PUT", "/api/v1/sensors/camera"),
        ("GET", "/api/v1/teleop"), ("POST", "/api/v1/navigation/goal"),
        ("POST", "/api/v1/updates/activate"),
    ):
        assert request(path, authenticated=True, method=method,
                       data={} if method != "GET" else None)[0] == 403
    status, body = request("/api/v1/maps", authenticated=True)
    assert status == 200
    maps = json.loads(body)
    print(json.dumps({"passed": True, "tls_chain_and_ip_verified": True,
                      "unauthenticated_status": 401, "control_routes": 403,
                      "status": state, "maps": maps}, indent=2))


if __name__ == "__main__":
    main()
