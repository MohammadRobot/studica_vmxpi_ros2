"""Redacted, bounded diagnostic support-bundle generation."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import subprocess
import tarfile
import tempfile


MAX_COMMAND_OUTPUT = 2 * 1024 * 1024
UNITS = (
    "studica-hardware.service",
    "studica-mode-manager.service",
    "studica-lidar.service",
    "studica-camera.service",
    "studica-joystick.service",
    "studica-web.service",
    "studica-update.service",
)
SECRET_PATTERN = re.compile(
    r"(?i)(token|password|passwd|psk|secret|authorization)(\s*[:=]\s*)(\S+)"
)
IP_PATTERN = re.compile(r"\b(?!127\.0\.0\.1\b)(?:\d{1,3}\.){3}\d{1,3}\b")
MAC_PATTERN = re.compile(r"(?i)\b(?:[0-9a-f]{2}[:-]){5}[0-9a-f]{2}\b")


def redact(text: str) -> str:
    text = SECRET_PATTERN.sub(r"\1\2[REDACTED]", text)
    text = IP_PATTERN.sub("[REDACTED-IP]", text)
    return MAC_PATTERN.sub("[REDACTED-MAC]", text)


def _command(arguments: list[str]) -> str:
    try:
        result = subprocess.run(
            arguments,
            check=False,
            capture_output=True,
            timeout=20,
        )
        payload = (result.stdout + result.stderr)[:MAX_COMMAND_OUTPUT]
        return redact(payload.decode("utf-8", errors="replace"))
    except (OSError, subprocess.TimeoutExpired) as error:
        return f"command unavailable: {error}\n"


def _copy_redacted(source: Path, destination: Path) -> None:
    try:
        value = source.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as error:
        value = f"unavailable: {error}\n"
    destination.write_text(redact(value)[:MAX_COMMAND_OUTPUT], encoding="utf-8")


def create_bundle(output_root: Path = Path("/var/lib/studica/support")) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    destination = output_root / f"studica-support-{stamp}.tar.gz"
    with tempfile.TemporaryDirectory(prefix="studica-support-") as temporary:
        root = Path(temporary) / "studica-support"
        root.mkdir()
        metadata = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "redacted": True,
        }
        (root / "bundle.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        (root / "system.txt").write_text(
            _command(["/usr/bin/uname", "-a"])
            + _command(["/usr/bin/systemd-analyze", "blame"]),
            encoding="utf-8",
        )
        (root / "units.txt").write_text(
            _command(
                [
                    "/usr/bin/systemctl",
                    "show",
                    *UNITS,
                    "--property=Id,ActiveState,SubState,Result,NRestarts",
                ]
            ),
            encoding="utf-8",
        )
        (root / "journal.txt").write_text(
            _command(
                [
                    "/usr/bin/journalctl",
                    "--no-pager",
                    "--since=-2 hours",
                    "-n",
                    "3000",
                    *sum((["-u", unit] for unit in UNITS), []),
                ]
            ),
            encoding="utf-8",
        )
        allowlisted = {
            "device.json": Path("/var/lib/studica/device.json"),
            "platform-status.json": Path("/run/studica/platform-status.json"),
            "robot.env": Path("/etc/studica/robot.env"),
            "cyclonedds.xml": Path("/etc/studica/cyclonedds.xml"),
            "release.json": Path(
                "/opt/studica/current/metadata/release.json"
            ),
        }
        for name, source in allowlisted.items():
            _copy_redacted(source, root / name)
        update_statuses = []
        for path in sorted(Path("/var/lib/studica/updates/staged").glob("*/status.json")):
            try:
                update_statuses.append(json.loads(path.read_text(encoding="utf-8")))
            except (OSError, json.JSONDecodeError):
                continue
        (root / "updates.json").write_text(
            json.dumps(update_statuses, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        with tarfile.open(destination, "w:gz") as archive:
            archive.add(root, arcname="studica-support", recursive=True)
    os.chmod(destination, 0o640)
    return destination
