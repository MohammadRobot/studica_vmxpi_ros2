#!/usr/bin/env python3
"""Install the Ubuntu 22.04/Humble companion as a user service."""

from __future__ import annotations

import argparse
import ipaddress
import json
import os
import re
from pathlib import Path
import shlex
import shutil
import socket
import ssl
import subprocess
from urllib import error as urlerror
from urllib.parse import urlparse
from urllib import request as urlrequest

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from studica_robot_platform.provisioning import validate_domain_id


def _private_copy(source: Path, destination: Path, mode: int) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)
    os.chmod(destination, mode)


def _redeem_pairing_code(
    robot_url: str,
    certificate: Path,
    code: str,
    companion_id: str,
    domain_id: int = 42,
) -> tuple[str, int]:
    payload = json.dumps(
        {"code": code, "companion_id": companion_id}
    ).encode("utf-8")
    request = urlrequest.Request(
        f"{robot_url.rstrip('/')}/api/v1/companion/pair",
        data=payload,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    context = ssl.create_default_context(cafile=str(certificate))
    try:
        with urlrequest.urlopen(request, context=context, timeout=10) as response:
            document = json.loads(response.read(1024 * 1024).decode("utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, urlerror.URLError) as error:
        raise SystemExit(f"companion pairing failed: {error}") from error
    token = str(document.get("token", "")) if isinstance(document, dict) else ""
    if len(token) < 24 or document.get("scope") != "companion":
        raise SystemExit("companion pairing returned an invalid credential")
    if document.get("ros_domain_id", 42) != domain_id:
        raise SystemExit("robot ROS domain differs from --domain-id; re-pair with its configured domain")
    try:
        peer_version = ipaddress.ip_address(document["peer_address"]).version
    except (KeyError, TypeError, ValueError) as error:
        raise SystemExit("companion pairing returned an invalid peer") from error
    return token, peer_version


def _robot_peer(robot_url: str, ip_version: int | None = None) -> str:
    hostname = urlparse(robot_url).hostname
    if not hostname:
        raise SystemExit("robot URL has no hostname")
    try:
        answers = socket.getaddrinfo(
            hostname, 443, type=socket.SOCK_STREAM
        )
    except OSError as error:
        raise SystemExit(f"cannot resolve robot for DDS setup: {error}") from error
    addresses = []
    for family, _, _, _, socket_address in answers:
        if family in {socket.AF_INET, socket.AF_INET6}:
            if ip_version == 4 and family != socket.AF_INET:
                continue
            if ip_version == 6 and family != socket.AF_INET6:
                continue
            addresses.append((family != socket.AF_INET, socket_address[0]))
    if not addresses:
        raise SystemExit("robot URL did not resolve to an IP address")
    return sorted(set(addresses))[0][1]


def _write_cyclonedds(path: Path, peer: str) -> None:
    path.write_text(
        f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces><NetworkInterface autodetermine="true" multicast="false" /></Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
      <Peers AddLocalhost="true"><Peer Address="{peer}" /></Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
''',
        encoding="utf-8",
    )
    os.chmod(path, 0o600)


def main() -> None:
    parser = argparse.ArgumentParser()
    credential = parser.add_mutually_exclusive_group(required=True)
    credential.add_argument(
        "--pairing-code",
        help="one-time code displayed by an authenticated robot operator",
    )
    credential.add_argument(
        "--token-file",
        type=Path,
        help="administrative recovery path; prefer a scoped pairing code",
    )
    parser.add_argument("--robot-certificate", type=Path, required=True)
    parser.add_argument("--robot-url", default="https://robot.local")
    parser.add_argument("--companion-id", default=os.uname().nodename)
    parser.add_argument("--no-enable", action="store_true")
    parser.add_argument("--robot-id", help="named independent robot session, e.g. robot01")
    parser.add_argument("--domain-id", type=int, default=42)
    options = parser.parse_args()
    validate_domain_id(options.domain_id)
    if options.robot_id and not re.fullmatch(r"[a-z][a-z0-9_-]{0,47}", options.robot_id):
        raise SystemExit("robot-id must be a lowercase identifier of at most 48 characters")
    if not options.robot_url.startswith("https://"):
        raise SystemExit("robot URL must use HTTPS")
    home = Path.home()
    config = home / ".config/studica"
    registrations = list((config / "robots").glob("*/session.json"))
    if (config / "session.json").exists():
        registrations.append(config / "session.json")
    for registration in registrations:
        existing = json.loads(registration.read_text())
        if existing["robot_id"] != (options.robot_id or "default") and existing["domain_id"] == options.domain_id:
            raise SystemExit("another robot session already uses this ROS domain")
    if options.robot_id:
        config = config / "robots" / options.robot_id
    cache = home / ".cache/studica" / (options.robot_id or "default") / "maps"
    cache.mkdir(parents=True, exist_ok=True)
    _private_copy(options.robot_certificate, config / "robot-ca.crt", 0o600)
    peer_version = None
    if options.pairing_code:
        token, peer_version = _redeem_pairing_code(
            options.robot_url,
            config / "robot-ca.crt",
            options.pairing_code,
            options.companion_id,
            options.domain_id,
        )
        (config / "token").write_text(token + "\n", encoding="utf-8")
        os.chmod(config / "token", 0o600)
    else:
        _private_copy(options.token_file, config / "token", 0o600)
    _write_cyclonedds(
        config / "cyclonedds.xml",
        _robot_peer(options.robot_url, peer_version),
    )

    prefix = Path(get_package_prefix("studica_vmxpi_ros2"))
    share = Path(get_package_share_directory("studica_vmxpi_ros2"))
    runtime = home / ".local/bin" / (
        f"studica-companion-{options.robot_id}" if options.robot_id else "studica-companion-runtime")
    runtime.parent.mkdir(parents=True, exist_ok=True)
    runtime.write_text(
        "#!/usr/bin/env bash\n"
        "set -eo pipefail\n"
        "source /opt/ros/humble/setup.bash\n"
        f"source {shlex.quote(str(prefix / 'setup.bash'))}\n"
        f"export ROS_DOMAIN_ID={options.domain_id}\n"
        "export ROS_LOCALHOST_ONLY=0\n"
        "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp\n"
        f"export CYCLONEDDS_URI={shlex.quote('file://' + str(config / 'cyclonedds.xml'))}\n"
        f"exec {shlex.quote(str(prefix / 'lib/studica_vmxpi_ros2/studica_companion.py'))} "
        f"--robot-url {shlex.quote(options.robot_url)} "
        f"--token-file {shlex.quote(str(config / 'token'))} "
        f"--ca-file {shlex.quote(str(config / 'robot-ca.crt'))} "
        f"--cache-root {shlex.quote(str(cache))} "
        f"--companion-id {shlex.quote(options.companion_id)}\n",
        encoding="utf-8",
    )
    os.chmod(runtime, 0o700)
    unit_root = home / ".config/systemd/user"
    unit_root.mkdir(parents=True, exist_ok=True)
    unit_source = share / "deployment/systemd/studica-companion.service"
    unit_name = (f"studica-companion-{options.robot_id}.service" if options.robot_id
                 else "studica-companion.service")
    unit = unit_source.read_text().replace(
        "%h/.local/bin/studica-companion-runtime", "%h/.local/bin/" + runtime.name)
    (unit_root / unit_name).write_text(unit)
    (config / "session.json").write_text(json.dumps({
        "robot_id": options.robot_id or "default", "domain_id": options.domain_id,
        "robot_url": options.robot_url,
    }, indent=2) + "\n")
    if not options.no_enable:
        subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
        subprocess.run(
            ["systemctl", "--user", "enable", "--now", unit_name],
            check=True,
        )
    print(
        "Companion installed. Its scoped token and robot certificate are "
        "private user files."
    )


if __name__ == "__main__":
    main()
