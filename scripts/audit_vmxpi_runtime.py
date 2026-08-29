#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Read-only audit of a VMXPi against the production runtime profile."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import glob
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import socket
import subprocess
import sys
from typing import Any, Iterable


PACKAGE_NAME = "studica_vmxpi_ros2"
DEFAULT_PROFILE_NAME = "vmxpi-production-v1.json"
SSHD_DEFAULTS = {
    "passwordauthentication": "yes",
    "permitrootlogin": "prohibit-password",
    "pubkeyauthentication": "yes",
    "x11forwarding": "yes",
}


@dataclass(frozen=True)
class Listener:
    protocol: str
    address: str
    port: int

    @property
    def public(self) -> bool:
        address = self.address.split("%", maxsplit=1)[0].strip("[]")
        if address in {"", "*"}:
            return True
        try:
            parsed = ipaddress.ip_address(address)
        except ValueError:
            return True
        return parsed.is_unspecified or not parsed.is_loopback


@dataclass(frozen=True)
class Snapshot:
    timestamp_utc: str
    os_id: str
    version_id: str
    architecture: str
    hostname: str
    system_state: str
    cpu_count: int
    load_1: float
    load_5: float
    load_15: float
    memory_total_bytes: int
    memory_available_bytes: int
    swap_total_bytes: int
    swap_free_bytes: int
    root_total_bytes: int
    root_free_bytes: int
    temperature_c: float | None
    journal_bytes: int | None
    unit_active: dict[str, bool]
    failed_units: tuple[str, ...]
    installed_packages: tuple[str, ...]
    listeners: tuple[Listener, ...]
    ufw_enabled: bool | None
    sshd_settings: dict[str, str]
    process_markers: tuple[str, ...]
    command_errors: tuple[str, ...]


@dataclass(frozen=True)
class Finding:
    finding_id: str
    severity: str
    detail: str


def default_profile_path(script_path: Path | None = None) -> Path:
    script = (script_path or Path(__file__)).resolve()
    source_candidate = script.parents[1] / "deployment" / DEFAULT_PROFILE_NAME
    if source_candidate.is_file():
        return source_candidate
    if len(script.parents) >= 3:
        installed_candidate = (
            script.parents[2]
            / "share"
            / PACKAGE_NAME
            / "deployment"
            / DEFAULT_PROFILE_NAME
        )
        if installed_candidate.is_file():
            return installed_candidate
    return Path("/etc/studica") / DEFAULT_PROFILE_NAME


def load_profile(path: Path) -> dict[str, Any]:
    try:
        profile = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot load runtime profile {path}: {error}") from error
    required_keys = {
        "schema_version",
        "profile",
        "platform",
        "required_active_units",
        "prohibited_active_units",
        "prohibited_installed_packages",
        "prohibited_process_markers",
        "allowed_public_tcp_ports",
        "allowed_public_udp_ports",
        "generic_hostnames",
        "required_sshd_settings",
        "thresholds",
    }
    missing = sorted(required_keys - profile.keys())
    if missing:
        raise ValueError(f"runtime profile is missing keys: {', '.join(missing)}")
    if profile["schema_version"] != 1:
        raise ValueError(
            f"unsupported runtime profile schema {profile['schema_version']!r}"
        )
    return profile


def run_command(arguments: list[str], timeout: float = 5.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        arguments,
        check=False,
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def parse_os_release(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", maxsplit=1)
        result[key] = value.strip().strip('"').strip("'")
    return result


def parse_meminfo(text: str) -> dict[str, int]:
    result: dict[str, int] = {}
    for line in text.splitlines():
        match = re.fullmatch(r"([^:]+):\s+(\d+)\s+kB", line.strip())
        if match:
            result[match.group(1)] = int(match.group(2)) * 1024
    return result


def parse_ss_listeners(text: str, protocol: str = "tcp") -> tuple[Listener, ...]:
    listeners: set[Listener] = set()
    for line in text.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        local = fields[3]
        if ":" not in local:
            continue
        address, port_text = local.rsplit(":", maxsplit=1)
        try:
            port = int(port_text)
        except ValueError:
            continue
        listeners.add(Listener(protocol, address.strip("[]"), port))
    return tuple(
        sorted(listeners, key=lambda item: (item.protocol, item.port, item.address))
    )


def parse_failed_units(text: str) -> tuple[str, ...]:
    failed: set[str] = set()
    for line in text.splitlines():
        fields = line.split()
        if fields and fields[0] == "●":
            fields = fields[1:]
        if fields and fields[0].endswith((".service", ".socket", ".timer", ".path")):
            failed.add(fields[0])
    return tuple(sorted(failed))


def parse_sshd_settings(text: str) -> dict[str, str]:
    settings: dict[str, str] = {}
    for line in text.splitlines():
        fields = line.split(maxsplit=1)
        if len(fields) == 2:
            settings[fields[0].lower()] = fields[1].strip().lower()
    return settings


def parse_sshd_config(path: Path, requested_settings: Iterable[str]) -> dict[str, str]:
    """Resolve global sshd settings when private host keys make ``sshd -T`` unavailable."""
    requested = {setting.lower() for setting in requested_settings}
    settings: dict[str, str] = {}
    visited: set[Path] = set()

    def visit(config_path: Path) -> bool:
        resolved = config_path.resolve()
        if resolved in visited:
            return False
        visited.add(resolved)
        try:
            lines = resolved.read_text(encoding="utf-8").splitlines()
        except OSError:
            return False
        for line in lines:
            try:
                fields = shlex.split(line, comments=True, posix=True)
            except ValueError:
                continue
            if not fields:
                continue
            keyword = fields[0].lower()
            if keyword == "match":
                return True
            if keyword == "include":
                for pattern in fields[1:]:
                    if not pattern.startswith("/"):
                        pattern = str(resolved.parent / pattern)
                    for included in sorted(glob.glob(pattern)):
                        if visit(Path(included)):
                            return True
                continue
            if keyword in requested and keyword not in settings and len(fields) >= 2:
                settings[keyword] = fields[1].lower()
        return False

    visit(path)
    for setting in requested:
        if setting not in settings and setting in SSHD_DEFAULTS:
            settings[setting] = SSHD_DEFAULTS[setting]
    return settings


def parse_size_bytes(text: str) -> int | None:
    match = re.search(r"([0-9]+(?:\.[0-9]+)?)\s*([KMGT]?)i?B?", text)
    if not match:
        return None
    scales = {"": 1, "K": 1024, "M": 1024**2, "G": 1024**3, "T": 1024**4}
    return int(float(match.group(1)) * scales[match.group(2)])


def read_temperature_c() -> float | None:
    candidates = sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp"))
    for candidate in candidates:
        try:
            value = float(candidate.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            continue
        if value > 1000.0:
            value /= 1000.0
        if -20.0 <= value <= 150.0:
            return value
    return None


def collect_snapshot(profile: dict[str, Any], required_units: Iterable[str]) -> Snapshot:
    errors: list[str] = []
    try:
        os_release = parse_os_release(Path("/etc/os-release").read_text(encoding="utf-8"))
    except OSError as error:
        os_release = {}
        errors.append(f"/etc/os-release: {error}")

    try:
        architecture_result = run_command(["dpkg", "--print-architecture"])
        architecture = architecture_result.stdout.strip()
        if architecture_result.returncode != 0:
            errors.append("dpkg --print-architecture failed")
    except (OSError, subprocess.TimeoutExpired) as error:
        architecture = "unknown"
        errors.append(f"architecture probe: {error}")

    try:
        system_state_result = run_command(["systemctl", "is-system-running"])
        system_state = system_state_result.stdout.strip() or "unknown"
    except (OSError, subprocess.TimeoutExpired) as error:
        system_state = "unknown"
        errors.append(f"system state probe: {error}")

    all_units = sorted(
        set(profile["required_active_units"])
        | set(profile["prohibited_active_units"])
        | set(required_units)
    )
    unit_active: dict[str, bool] = {}
    for unit in all_units:
        try:
            result = run_command(["systemctl", "is-active", unit], timeout=3.0)
            unit_active[unit] = result.stdout.strip() == "active"
        except (OSError, subprocess.TimeoutExpired) as error:
            unit_active[unit] = False
            errors.append(f"unit probe {unit}: {error}")

    try:
        failed_result = run_command(
            ["systemctl", "--failed", "--plain", "--no-legend", "--no-pager"]
        )
        failed_units = parse_failed_units(failed_result.stdout)
    except (OSError, subprocess.TimeoutExpired) as error:
        failed_units = ()
        errors.append(f"failed-unit probe: {error}")

    installed_packages: list[str] = []
    for package in profile["prohibited_installed_packages"]:
        try:
            result = run_command(
                ["dpkg-query", "-W", "-f=${db:Status-Abbrev}", package], timeout=3.0
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            errors.append(f"package probe {package}: {error}")
            continue
        if result.returncode == 0 and result.stdout.startswith("ii"):
            installed_packages.append(package)

    try:
        tcp_result = run_command(["ss", "-H", "-lnt"])
        udp_result = run_command(["ss", "-H", "-lnu"])
        listeners = tuple(
            sorted(
                set(parse_ss_listeners(tcp_result.stdout, "tcp"))
                | set(parse_ss_listeners(udp_result.stdout, "udp")),
                key=lambda item: (item.protocol, item.port, item.address),
            )
        )
        if tcp_result.returncode != 0:
            errors.append("TCP listener probe failed")
        if udp_result.returncode != 0:
            errors.append("UDP listener probe failed")
    except (OSError, subprocess.TimeoutExpired) as error:
        listeners = ()
        errors.append(f"network listener probe: {error}")

    try:
        ufw_config = parse_os_release(
            Path("/etc/ufw/ufw.conf").read_text(encoding="utf-8")
        )
        ufw_value = ufw_config.get("ENABLED", "").lower()
        ufw_enabled = ufw_value == "yes"
        if ufw_value not in {"yes", "no"}:
            errors.append("UFW enabled state is missing or invalid")
    except OSError as error:
        ufw_enabled = None
        errors.append(f"UFW configuration probe: {error}")

    sshd_binary = shutil.which("sshd") or "/usr/sbin/sshd"
    try:
        sshd_result = run_command([sshd_binary, "-T"])
        sshd_settings = parse_sshd_settings(sshd_result.stdout)
        if sshd_result.returncode != 0:
            sshd_settings = parse_sshd_config(
                Path("/etc/ssh/sshd_config"),
                profile["required_sshd_settings"],
            )
            if not sshd_settings:
                errors.append("sshd effective-configuration probe failed")
    except (OSError, subprocess.TimeoutExpired) as error:
        sshd_settings = {}
        errors.append(f"sshd configuration probe: {error}")

    process_markers: set[str] = set()
    try:
        processes_result = run_command(["ps", "-eo", "comm="], timeout=5.0)
        process_names = {
            line.strip().lower()
            for line in processes_result.stdout.splitlines()
            if line.strip()
        }
        for marker in profile["prohibited_process_markers"]:
            if marker.lower() in process_names:
                process_markers.add(marker)
    except (OSError, subprocess.TimeoutExpired) as error:
        errors.append(f"process probe: {error}")

    try:
        journal_result = run_command(["journalctl", "--disk-usage"])
        journal_bytes = parse_size_bytes(journal_result.stdout)
        if journal_result.returncode != 0:
            errors.append("journal size probe failed")
    except (OSError, subprocess.TimeoutExpired) as error:
        journal_bytes = None
        errors.append(f"journal size probe: {error}")

    try:
        meminfo = parse_meminfo(Path("/proc/meminfo").read_text(encoding="utf-8"))
    except OSError as error:
        meminfo = {}
        errors.append(f"memory probe: {error}")
    memory_total = meminfo.get("MemTotal", 0)
    memory_available = meminfo.get("MemAvailable", 0)
    swap_total = meminfo.get("SwapTotal", 0)
    swap_free = meminfo.get("SwapFree", 0)

    load_1, load_5, load_15 = os.getloadavg()
    root_disk = shutil.disk_usage("/")
    return Snapshot(
        timestamp_utc=datetime.now(timezone.utc).replace(microsecond=0).isoformat(),
        os_id=os_release.get("ID", "unknown"),
        version_id=os_release.get("VERSION_ID", "unknown"),
        architecture=architecture,
        hostname=socket.gethostname(),
        system_state=system_state,
        cpu_count=os.cpu_count() or 1,
        load_1=load_1,
        load_5=load_5,
        load_15=load_15,
        memory_total_bytes=memory_total,
        memory_available_bytes=memory_available,
        swap_total_bytes=swap_total,
        swap_free_bytes=swap_free,
        root_total_bytes=root_disk.total,
        root_free_bytes=root_disk.free,
        temperature_c=read_temperature_c(),
        journal_bytes=journal_bytes,
        unit_active=unit_active,
        failed_units=failed_units,
        installed_packages=tuple(sorted(installed_packages)),
        listeners=listeners,
        ufw_enabled=ufw_enabled,
        sshd_settings=sshd_settings,
        process_markers=tuple(sorted(process_markers)),
        command_errors=tuple(errors),
    )


def _percent(part: int, total: int) -> float:
    return 100.0 * part / total if total > 0 else 0.0


def evaluate_snapshot(
    snapshot: Snapshot,
    profile: dict[str, Any],
    extra_required_units: Iterable[str] = (),
    allowed_units: Iterable[str] = (),
    allowed_packages: Iterable[str] = (),
    allowed_ports: Iterable[int] = (),
    allowed_udp_ports: Iterable[int] = (),
    skip_platform_check: bool = False,
) -> list[Finding]:
    findings: list[Finding] = []
    platform = profile["platform"]
    if not skip_platform_check:
        observed_platform = (snapshot.os_id, snapshot.version_id, snapshot.architecture)
        required_platform = (
            platform["os_id"],
            platform["version_id"],
            platform["architecture"],
        )
        if observed_platform != required_platform:
            findings.append(
                Finding(
                    "platform",
                    "ERROR",
                    "expected " + "/".join(required_platform) + ", found " + "/".join(observed_platform),
                )
            )

    if snapshot.hostname.lower() in {name.lower() for name in profile["generic_hostnames"]}:
        findings.append(
            Finding(
                "generic-hostname",
                "ERROR",
                f"hostname {snapshot.hostname!r} must be unique per production robot",
            )
        )

    if snapshot.system_state not in {"running"}:
        findings.append(
            Finding(
                "system-state",
                "ERROR",
                f"systemd state is {snapshot.system_state!r}, not 'running'",
            )
        )
    for unit in snapshot.failed_units:
        findings.append(Finding(f"failed-unit:{unit}", "ERROR", f"systemd unit {unit} is failed"))

    required_units = set(profile["required_active_units"]) | set(extra_required_units)
    for unit in sorted(required_units):
        if not snapshot.unit_active.get(unit, False):
            findings.append(
                Finding(f"required-unit:{unit}", "ERROR", f"required unit {unit} is not active")
            )

    unit_exceptions = set(allowed_units)
    for unit in profile["prohibited_active_units"]:
        if snapshot.unit_active.get(unit, False) and unit not in unit_exceptions:
            findings.append(
                Finding(f"unneeded-unit:{unit}", "ERROR", f"non-runtime unit {unit} is active")
            )

    package_exceptions = set(allowed_packages)
    for package in snapshot.installed_packages:
        if package not in package_exceptions:
            findings.append(
                Finding(
                    f"nonruntime-package:{package}",
                    "WARN",
                    f"non-runtime package {package} is installed in the product image",
                )
            )

    if snapshot.process_markers:
        findings.append(
            Finding(
                "nonruntime-processes",
                "ERROR",
                "non-runtime process markers are present: " + ", ".join(snapshot.process_markers),
            )
        )

    permitted_tcp_ports = set(profile["allowed_public_tcp_ports"]) | set(allowed_ports)
    permitted_udp_ports = set(profile["allowed_public_udp_ports"]) | set(allowed_udp_ports)
    for listener in snapshot.listeners:
        permitted_ports = permitted_tcp_ports if listener.protocol == "tcp" else permitted_udp_ports
        if listener.public and listener.port not in permitted_ports:
            findings.append(
                Finding(
                    f"public-{listener.protocol}:{listener.port}",
                    "ERROR",
                    f"unexpected public {listener.protocol.upper()} listener "
                    f"{listener.address}:{listener.port}",
                )
            )

    if snapshot.ufw_enabled is not True:
        findings.append(
            Finding(
                "firewall-disabled",
                "ERROR",
                "UFW must be enabled with the approved default-deny product policy",
            )
        )

    for setting, expected in profile["required_sshd_settings"].items():
        observed = snapshot.sshd_settings.get(setting)
        if observed != expected:
            findings.append(
                Finding(
                    f"sshd:{setting}",
                    "ERROR",
                    f"sshd {setting} must be {expected!r}, found {observed or 'unavailable'!r}",
                )
            )

    thresholds = profile["thresholds"]
    normalized_load = snapshot.load_1 / max(snapshot.cpu_count, 1)
    if normalized_load > thresholds["max_normalized_load_1"]:
        findings.append(
            Finding(
                "load",
                "WARN",
                f"one-minute load per CPU is {normalized_load:.2f}",
            )
        )
    available_memory = _percent(snapshot.memory_available_bytes, snapshot.memory_total_bytes)
    if available_memory < thresholds["min_available_memory_percent"]:
        findings.append(
            Finding("memory", "WARN", f"available memory is {available_memory:.1f}%")
        )
    swap_used = _percent(
        snapshot.swap_total_bytes - snapshot.swap_free_bytes,
        snapshot.swap_total_bytes,
    )
    if swap_used > thresholds["max_swap_used_percent"]:
        findings.append(Finding("swap", "WARN", f"swap use is {swap_used:.1f}%"))
    root_used = 100.0 - _percent(snapshot.root_free_bytes, snapshot.root_total_bytes)
    if root_used > thresholds["max_root_used_percent"]:
        findings.append(Finding("root-disk", "WARN", f"root filesystem use is {root_used:.1f}%"))
    if snapshot.temperature_c is not None and snapshot.temperature_c > thresholds["max_temperature_c"]:
        findings.append(
            Finding("temperature", "WARN", f"CPU temperature is {snapshot.temperature_c:.1f} C")
        )
    if snapshot.journal_bytes is None:
        findings.append(Finding("journal-size", "WARN", "journald disk use could not be measured"))
    elif snapshot.journal_bytes > thresholds["max_journal_bytes"]:
        findings.append(
            Finding(
                "journal-size",
                "WARN",
                f"journald uses {snapshot.journal_bytes / 1024**2:.1f} MiB",
            )
        )

    for error in snapshot.command_errors:
        findings.append(Finding("probe-error", "WARN", error))
    return findings


def report_dict(snapshot: Snapshot, profile: dict[str, Any], findings: list[Finding]) -> dict[str, Any]:
    report = {
        "schema_version": 1,
        "profile": profile["profile"],
        "compliant": not findings,
        "snapshot": asdict(snapshot),
        "findings": [asdict(finding) for finding in findings],
    }
    return report


def print_human(snapshot: Snapshot, profile: dict[str, Any], findings: list[Finding]) -> None:
    memory_available = _percent(snapshot.memory_available_bytes, snapshot.memory_total_bytes)
    root_used = 100.0 - _percent(snapshot.root_free_bytes, snapshot.root_total_bytes)
    swap_used = _percent(
        snapshot.swap_total_bytes - snapshot.swap_free_bytes,
        snapshot.swap_total_bytes,
    )
    temperature = "unavailable" if snapshot.temperature_c is None else f"{snapshot.temperature_c:.1f} C"
    journal = "unavailable" if snapshot.journal_bytes is None else f"{snapshot.journal_bytes / 1024**2:.1f} MiB"
    print(f"VMXPi runtime audit: {profile['profile']}")
    print(
        f"Platform: {snapshot.os_id} {snapshot.version_id} {snapshot.architecture}; "
        f"hostname={snapshot.hostname}; systemd={snapshot.system_state}"
    )
    print(
        f"Load: {snapshot.load_1:.2f} {snapshot.load_5:.2f} {snapshot.load_15:.2f} "
        f"on {snapshot.cpu_count} CPUs; available memory={memory_available:.1f}%; "
        f"swap used={swap_used:.1f}%"
    )
    print(f"Storage: root used={root_used:.1f}%; journal={journal}; CPU temperature={temperature}")
    public_listeners = [
        f"{listener.protocol}://{listener.address}:{listener.port}"
        for listener in snapshot.listeners
        if listener.public
    ]
    firewall = "enabled" if snapshot.ufw_enabled else "disabled or unavailable"
    print("Public network listeners: " + (", ".join(public_listeners) or "none"))
    print(f"Host firewall: {firewall}")
    if findings:
        print("Findings:")
        for finding in findings:
            print(f"  [{finding.severity}] {finding.finding_id}: {finding.detail}")
        print(f"RESULT: FAIL ({len(findings)} findings; no changes made)")
    else:
        print("RESULT: PASS (production runtime profile satisfied; no changes made)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        type=Path,
        default=default_profile_path(),
        help="JSON runtime profile (default: %(default)s)",
    )
    parser.add_argument(
        "--require-unit",
        action="append",
        default=[],
        metavar="UNIT",
        help="require an additional systemd unit; repeat as needed",
    )
    parser.add_argument(
        "--allow-unit",
        action="append",
        default=[],
        metavar="UNIT",
        help="temporarily allow one profile-prohibited active unit",
    )
    parser.add_argument(
        "--allow-package",
        action="append",
        default=[],
        metavar="PACKAGE",
        help="temporarily allow one profile-prohibited installed package",
    )
    parser.add_argument(
        "--allow-public-tcp-port",
        action="append",
        default=[],
        type=int,
        metavar="PORT",
        help="temporarily allow one additional public TCP port",
    )
    parser.add_argument(
        "--allow-public-udp-port",
        action="append",
        default=[],
        type=int,
        metavar="PORT",
        help="temporarily allow one additional public UDP port",
    )
    parser.add_argument(
        "--skip-platform-check",
        action="store_true",
        help="audit a development host without enforcing Ubuntu/arm64 identity",
    )
    parser.add_argument("--json", action="store_true", help="emit a JSON report")
    arguments = parser.parse_args()
    try:
        profile = load_profile(arguments.profile.resolve())
        snapshot = collect_snapshot(profile, arguments.require_unit)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    findings = evaluate_snapshot(
        snapshot,
        profile,
        extra_required_units=arguments.require_unit,
        allowed_units=arguments.allow_unit,
        allowed_packages=arguments.allow_package,
        allowed_ports=arguments.allow_public_tcp_port,
        allowed_udp_ports=arguments.allow_public_udp_port,
        skip_platform_check=arguments.skip_platform_check,
    )
    if arguments.json:
        print(json.dumps(report_dict(snapshot, profile, findings), indent=2, sort_keys=True))
    else:
        print_human(snapshot, profile, findings)
    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main())
