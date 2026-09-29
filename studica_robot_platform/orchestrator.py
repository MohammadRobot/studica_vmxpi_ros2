"""Privileged, local-only systemd orchestrator with a strict allowlist."""

from __future__ import annotations

import argparse
import grp
import ipaddress
import json
import os
from pathlib import Path
import pwd
import re
import signal
import socket
import struct
import subprocess
import tempfile
from typing import Any, Callable, Dict, Iterable, Optional

from .provisioning import private_write, render_cyclonedds, dds_port_range


SENSOR_UNITS = {
    "lidar": "studica-lidar.service",
    "camera": "studica-camera.service",
}
MANAGED_UNITS = frozenset(SENSOR_UNITS.values())
BLUETOOTH_ADDRESS = re.compile(r"(?:[0-9A-F]{2}:){5}[0-9A-F]{2}")
RELEASE_VERSION = re.compile(r"[0-9A-Za-z][0-9A-Za-z.+-]{0,63}")
WIFI_SSID = re.compile(r"[^\x00-\x1f\x7f]{1,32}")


def camera_resources_available() -> tuple[bool, str]:
    """Apply a conservative preflight before a camera workload starts."""
    try:
        cpu_count = max(1, os.cpu_count() or 1)
        normalized_load = os.getloadavg()[0] / cpu_count
        memory = {}
        for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
            key, _, value = line.partition(":")
            if key in {"MemTotal", "MemAvailable"}:
                memory[key] = int(value.strip().split()[0])
        available_ratio = memory["MemAvailable"] / memory["MemTotal"]
    except (OSError, KeyError, ValueError, ZeroDivisionError) as error:
        return False, f"resource preflight unavailable: {error}"
    if normalized_load > 0.80:
        return False, "camera blocked because normalized compute load exceeds 80%"
    if available_ratio < 0.20:
        return False, "camera blocked because available memory is below 20%"
    return True, "camera resource preflight passed"


class UnitController:
    """Run fixed systemctl operations without accepting command fragments."""

    def __init__(
        self,
        runner: Callable[..., subprocess.CompletedProcess] = subprocess.run,
        companion_address_path: Path = Path("/etc/studica/companion-address"),
        cyclonedds_path: Path = Path("/etc/studica/cyclonedds.xml"),
        update_state_root: Path = Path("/var/lib/studica/updates"),
        transport_state_root: Path = Path("/run/studica/dds"),
        camera_resource_probe: Callable[[], tuple[bool, str]] = (
            camera_resources_available
        ),
        domain_id: Optional[int] = None,
    ):
        self._runner = runner
        self._companion_address_path = companion_address_path
        self._cyclonedds_path = cyclonedds_path
        self._update_state_root = update_state_root
        self._transport_state_root = transport_state_root
        self._camera_resource_probe = camera_resource_probe
        self._dds_ports = dds_port_range(
            int(os.environ.get("ROS_DOMAIN_ID", "42")) if domain_id is None else domain_id)

    def set_active(self, unit: str, enabled: bool) -> Dict[str, Any]:
        if unit not in MANAGED_UNITS:
            return {"ok": False, "error": "unit is not managed"}
        if unit == SENSOR_UNITS["camera"] and enabled:
            available, reason = self._camera_resource_probe()
            if not available:
                return {"ok": False, "error": reason}
        verb = "start" if enabled else "stop"
        result = self._runner(
            ["/usr/bin/systemctl", verb, unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=30,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()[:512]
            return {"ok": False, "error": detail or f"systemctl {verb} failed"}
        return {"ok": True, "message": f"{unit} {verb} requested"}

    def status(self, units: Iterable[str] = MANAGED_UNITS) -> Dict[str, Any]:
        states: Dict[str, str] = {}
        for unit in units:
            if unit not in MANAGED_UNITS:
                continue
            result = self._runner(
                ["/usr/bin/systemctl", "is-active", unit],
                check=False,
                capture_output=True,
                text=True,
                timeout=5,
            )
            states[unit] = (result.stdout.strip() or "unknown")[:64]
        return {"ok": True, "units": states}

    def bluetooth_devices(self, scan: bool) -> Dict[str, Any]:
        if scan:
            self._runner(
                ["/usr/bin/bluetoothctl", "--timeout", "8", "scan", "on"],
                check=False,
                capture_output=True,
                text=True,
                timeout=12,
            )
        result = self._runner(
            ["/usr/bin/bluetoothctl", "devices"],
            check=False,
            capture_output=True,
            text=True,
            timeout=5,
        )
        devices = []
        for line in result.stdout.splitlines():
            match = re.fullmatch(
                r"Device ((?:[0-9A-F]{2}:){5}[0-9A-F]{2}) (.{1,128})", line
            )
            if match:
                devices.append({"address": match.group(1), "name": match.group(2)})
        return {"ok": result.returncode == 0, "devices": devices}

    def bluetooth_pair(self, address: str) -> Dict[str, Any]:
        address = address.upper()
        if BLUETOOTH_ADDRESS.fullmatch(address) is None:
            return {"ok": False, "error": "invalid Bluetooth address"}
        for verb, timeout in (("pair", 25), ("trust", 8), ("connect", 15)):
            result = self._runner(
                ["/usr/bin/bluetoothctl", "--timeout", str(timeout), verb, address],
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout + 5,
            )
            if result.returncode != 0:
                detail = (result.stderr or result.stdout).strip()[-512:]
                return {"ok": False, "error": detail or f"Bluetooth {verb} failed"}
        return {"ok": True, "message": f"paired and connected {address}"}

    @staticmethod
    def _nmcli_fields(line: str) -> list[str]:
        fields = []
        current = []
        escaped = False
        for character in line:
            if escaped:
                current.append(character)
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == ":":
                fields.append("".join(current))
                current = []
            else:
                current.append(character)
        fields.append("".join(current))
        return fields

    def wifi_networks(self) -> Dict[str, Any]:
        result = self._runner(
            [
                "/usr/bin/nmcli",
                "--terse",
                "--escape",
                "yes",
                "--fields",
                "SSID,SIGNAL,SECURITY",
                "device",
                "wifi",
                "list",
                "ifname",
                "wlan0",
                "--rescan",
                "yes",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=20,
        )
        if result.returncode != 0:
            return {"ok": False, "error": (result.stderr or result.stdout).strip()[-512:]}
        unique: Dict[str, Dict[str, Any]] = {}
        for line in result.stdout.splitlines():
            fields = self._nmcli_fields(line)
            if len(fields) != 3 or not fields[0]:
                continue
            try:
                signal_strength = max(0, min(100, int(fields[1])))
            except ValueError:
                signal_strength = 0
            candidate = {
                "ssid": fields[0],
                "signal": signal_strength,
                "security": fields[2] or "open",
            }
            if signal_strength >= unique.get(fields[0], {}).get("signal", -1):
                unique[fields[0]] = candidate
        return {"ok": True, "networks": sorted(unique.values(), key=lambda item: -item["signal"])}

    def wifi_connect(self, ssid: str, password: str) -> Dict[str, Any]:
        if WIFI_SSID.fullmatch(ssid) is None:
            return {"ok": False, "error": "invalid Wi-Fi SSID"}
        if not 8 <= len(password) <= 63 or any(
            ord(character) < 32 or ord(character) > 126 for character in password
        ):
            return {"ok": False, "error": "Wi-Fi password must contain 8 to 63 characters"}
        credential = None
        try:
            descriptor, name = tempfile.mkstemp(prefix="wifi-", dir="/run/studica")
            credential = Path(name)
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(f"802-11-wireless-security.psk:{password}\n")
            result = self._runner(
                [
                    "/usr/bin/nmcli",
                    "device",
                    "wifi",
                    "connect",
                    ssid,
                    "ifname",
                    "wlan0",
                    "passwd-file",
                    str(credential),
                    "name",
                    "Studica WiFi",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=45,
            )
        finally:
            if credential is not None:
                credential.unlink(missing_ok=True)
        if result.returncode != 0:
            return {"ok": False, "error": (result.stderr or result.stdout).strip()[-512:]}
        # Robot control Wi-Fi must never inherit power-saving defaults.
        self._runner(
            [
                "/usr/bin/nmcli",
                "connection",
                "modify",
                "Studica WiFi",
                "802-11-wireless.powersave",
                "2",
                "connection.autoconnect-priority",
                "100",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        return {"ok": True, "message": f"connected to Wi-Fi network {ssid}"}

    def _companion_address(self) -> str:
        try:
            return str(
                ipaddress.ip_address(
                    self._companion_address_path.read_text(
                        encoding="utf-8"
                    ).strip()
                )
            )
        except (OSError, ValueError) as error:
            raise ValueError(f"companion peer is not configured: {error}") from error

    def _dds_firewall(
        self, enabled: bool, label: str, state_name: str
    ) -> Dict[str, Any]:
        state_path = self._transport_state_root / state_name
        try:
            address = self._companion_address()
        except ValueError as error:
            if not enabled:
                return {"ok": True, "message": f"{label} already disabled"}
            return {"ok": False, "error": str(error)}
        allow_rule = [
            "allow",
            "in",
            "from",
            address,
            "to",
            "any",
            "port",
            self._dds_ports,
            "proto",
            "udp",
            "comment",
            label,
        ]
        deny_rule = [
            "deny",
            "out",
            "to",
            address,
            "port",
            self._dds_ports,
            "proto",
            "udp",
            "comment",
            "Studica DDS closed",
        ]

        def apply(rule: list[str]) -> Optional[str]:
            result = self._runner(
                ["/usr/sbin/ufw", "--force", *rule],
                check=False,
                capture_output=True,
                text=True,
                timeout=20,
            )
            if result.returncode == 0:
                return None
            return (result.stderr or result.stdout).strip()[-512:]

        if enabled:
            other = "developer" if state_name == "companion" else "companion"
            if (self._transport_state_root / other).is_file():
                return {"ok": False, "error": "another DDS owner is active"}
            self._transport_state_root.mkdir(parents=True, exist_ok=True)
            # Keep egress closed while preparing the ingress exception.  The
            # marker is committed before egress opens, and a repeated enable
            # reconciles an interrupted prior attempt instead of trusting the
            # marker alone.
            detail = apply(deny_rule)
            if detail is not None:
                return {"ok": False, "error": detail}
            detail = apply(allow_rule)
            if detail is not None:
                return {"ok": False, "error": detail}
            try:
                private_write(state_path, address + "\n", mode=0o600)
            except OSError as error:
                apply(["delete", *allow_rule])
                return {"ok": False, "error": f"cannot record DDS owner: {error}"}
            detail = apply(["delete", *deny_rule])
            if detail is not None:
                state_path.unlink(missing_ok=True)
                apply(["delete", *allow_rule])
                apply(deny_rule)
                return {"ok": False, "error": detail}
        else:
            detail = apply(deny_rule)
            if detail is not None:
                return {"ok": False, "error": detail}
            if not state_path.is_file():
                return {"ok": True, "message": f"{label} already disabled"}
            detail = apply(["delete", *allow_rule])
            if detail is not None:
                return {"ok": False, "error": detail}
            state_path.unlink(missing_ok=True)
        return {
            "ok": True,
            "message": f"{label} {'enabled' if enabled else 'disabled'}",
        }

    def developer_firewall(self, enabled: bool) -> Dict[str, Any]:
        return self._dds_firewall(
            enabled, "Studica developer DDS", "developer"
        )

    def companion_transport(self, enabled: bool) -> Dict[str, Any]:
        return self._dds_firewall(
            enabled, "Studica companion DDS", "companion"
        )

    def configure_companion_peer(self, address: str) -> Dict[str, Any]:
        try:
            peer = ipaddress.ip_address(address)
        except ValueError:
            return {"ok": False, "error": "invalid companion peer address"}
        if peer.is_unspecified or peer.is_multicast or peer.is_loopback:
            return {"ok": False, "error": "companion peer address is not routable"}
        normalized = str(peer)
        private_write(
            self._companion_address_path, normalized + "\n", mode=0o640
        )
        private_write(
            self._cyclonedds_path,
            render_cyclonedds(normalized),
            mode=0o644,
        )
        result = self._runner(
            [
                "/usr/bin/systemctl",
                "start",
                "--no-block",
                "studica-peer-apply.timer",
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            return {
                "ok": False,
                "error": (result.stderr or result.stdout).strip()[-512:],
            }
        return {
            "ok": True,
            "message": "companion peer configured; platform restart scheduled",
            "peer_address": normalized,
            "restart_scheduled": True,
        }

    def update_status(self) -> Dict[str, Any]:
        staged = self._update_state_root / "staged"
        updates = []
        if staged.is_dir() and not staged.is_symlink():
            for entry in sorted(staged.iterdir()):
                try:
                    value = json.loads(
                        (entry / "status.json").read_text(encoding="utf-8")
                    )
                except (OSError, json.JSONDecodeError):
                    continue
                if isinstance(value, dict):
                    updates.append(value)
        return {"ok": True, "updates": updates}

    def activate_update(self, version: str) -> Dict[str, Any]:
        if RELEASE_VERSION.fullmatch(version) is None:
            return {"ok": False, "error": "invalid release version"}
        status_path = self._update_state_root / "staged" / version / "status.json"
        try:
            status = json.loads(status_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            return {"ok": False, "error": f"staged update is unavailable: {error}"}
        if status.get("state") != "VERIFIED_PENDING_APPROVAL":
            return {"ok": False, "error": "update is not pending approval"}
        unit = f"studica-update-activate@{version}.service"
        result = self._runner(
            ["/usr/bin/systemctl", "start", "--no-block", unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            return {
                "ok": False,
                "error": (result.stderr or result.stdout).strip()[-512:],
            }
        return {"ok": True, "message": f"activation scheduled for {version}"}

    def support_bundle(self) -> Dict[str, Any]:
        executable = Path(
            "/opt/studica/current/install/lib/studica_vmxpi_ros2/"
            "create_support_bundle.py"
        )
        result = self._runner(
            [str(executable)],
            check=False,
            capture_output=True,
            text=True,
            timeout=40,
        )
        if result.returncode != 0:
            return {
                "ok": False,
                "error": (result.stderr or result.stdout).strip()[-512:],
            }
        path = Path(result.stdout.strip().splitlines()[-1]).resolve()
        support_root = Path("/var/lib/studica/support").resolve()
        try:
            path.relative_to(support_root)
        except ValueError:
            return {"ok": False, "error": "support tool returned an unsafe path"}
        if not path.is_file() or path.is_symlink():
            return {"ok": False, "error": "support bundle was not created"}
        return {"ok": True, "path": str(path), "name": path.name}


def handle_request(payload: Any, controller: UnitController) -> Dict[str, Any]:
    """Validate one request and dispatch only hard-coded operations."""
    if not isinstance(payload, dict):
        return {"ok": False, "error": "request must be an object"}
    action = payload.get("action")
    if action == "status":
        return controller.status()
    if action == "set_sensor":
        sensor = payload.get("sensor")
        enabled = payload.get("enabled")
        if sensor not in SENSOR_UNITS or not isinstance(enabled, bool):
            return {"ok": False, "error": "invalid sensor request"}
        return controller.set_active(SENSOR_UNITS[sensor], enabled)
    if action == "stop_optional":
        results = [
            controller.set_active(unit, False)
            for unit in sorted(MANAGED_UNITS)
            if unit != SENSOR_UNITS["lidar"]
        ]
        return {"ok": all(item["ok"] for item in results), "results": results}
    if action == "bluetooth_devices":
        return controller.bluetooth_devices(payload.get("scan") is True)
    if action == "bluetooth_pair":
        return controller.bluetooth_pair(str(payload.get("address", "")))
    if action == "wifi_networks":
        return controller.wifi_networks()
    if action == "wifi_connect":
        return controller.wifi_connect(
            str(payload.get("ssid", "")), str(payload.get("password", ""))
        )
    if action == "set_developer_firewall":
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            return {"ok": False, "error": "enabled must be boolean"}
        return controller.developer_firewall(enabled)
    if action == "set_companion_transport":
        enabled = payload.get("enabled")
        if not isinstance(enabled, bool):
            return {"ok": False, "error": "enabled must be boolean"}
        return controller.companion_transport(enabled)
    if action == "configure_companion_peer":
        return controller.configure_companion_peer(
            str(payload.get("address", ""))
        )
    if action == "update_status":
        return controller.update_status()
    if action == "activate_update":
        return controller.activate_update(str(payload.get("version", "")))
    if action == "support_bundle":
        return controller.support_bundle()
    return {"ok": False, "error": "unsupported action"}


class OrchestratorServer:
    """Serve authenticated local requests using Unix peer credentials."""

    def __init__(
        self,
        socket_path: Path,
        allowed_uid: int,
        socket_gid: int,
        controller: Optional[UnitController] = None,
    ) -> None:
        self.socket_path = socket_path
        self.allowed_uid = allowed_uid
        self.socket_gid = socket_gid
        self.controller = controller or UnitController()
        self._running = True
        self._server: Optional[socket.socket] = None

    def stop(self, signum=None, frame=None) -> None:
        del signum, frame
        self._running = False
        if self._server is not None:
            self._server.close()

    def _authorized(self, connection: socket.socket) -> bool:
        credentials = connection.getsockopt(
            socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i")
        )
        _, uid, _ = struct.unpack("3i", credentials)
        return uid in {0, self.allowed_uid}

    def _serve_connection(self, connection: socket.socket) -> None:
        connection.settimeout(2.0)
        if not self._authorized(connection):
            response = {"ok": False, "error": "unauthorized local peer"}
        else:
            data = bytearray()
            while b"\n" not in data and len(data) <= 4096:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                data.extend(chunk)
            try:
                if len(data) > 4096:
                    raise ValueError("request too large")
                payload = json.loads(bytes(data).split(b"\n", 1)[0])
                response = handle_request(payload, self.controller)
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
                response = {"ok": False, "error": str(error)}
            except Exception as error:
                response = {
                    "ok": False,
                    "error": f"orchestrator operation failed: {type(error).__name__}",
                }
        encoded = json.dumps(response, separators=(",", ":")).encode("utf-8") + b"\n"
        connection.sendall(encoded)

    def run(self) -> None:
        """Create the protected socket and serve until SIGTERM/SIGINT."""
        self.socket_path.parent.mkdir(mode=0o755, parents=True, exist_ok=True)
        try:
            self.socket_path.unlink(missing_ok=True)
            server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self._server = server
            server.bind(str(self.socket_path))
            os.chown(self.socket_path, 0, self.socket_gid)
            os.chmod(self.socket_path, 0o660)
            server.listen(8)
            server.settimeout(1.0)
            while self._running:
                try:
                    connection, _ = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._running:
                        raise
                    break
                with connection:
                    self._serve_connection(connection)
        finally:
            if self._server is not None:
                self._server.close()
            self.socket_path.unlink(missing_ok=True)


def main(args=None) -> None:
    """Run the root orchestrator for one configured service account."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/run/studica/orchestrator.sock")
    parser.add_argument("--allowed-user", default="studica")
    parser.add_argument("--socket-group", default="studica")
    options = parser.parse_args(args)
    allowed_uid = pwd.getpwnam(options.allowed_user).pw_uid
    socket_gid = grp.getgrnam(options.socket_group).gr_gid
    server = OrchestratorServer(Path(options.socket), allowed_uid, socket_gid)
    signal.signal(signal.SIGTERM, server.stop)
    signal.signal(signal.SIGINT, server.stop)
    server.run()


if __name__ == "__main__":
    main()
