"""First-boot identity, TLS, network, and safety-gate provisioning."""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import uuid
from typing import Any, Optional

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID


class ProvisioningError(RuntimeError):
    """Provisioning input or target filesystem is unsafe."""


def canonical_root(root: Path) -> Path:
    if not root.is_absolute():
        raise ProvisioningError("target root must be absolute")
    root = root.resolve()
    if root.is_symlink() or not root.is_dir():
        raise ProvisioningError("target root must be a real directory")
    return root


def root_path(root: Path, absolute: str) -> Path:
    if not absolute.startswith("/") or ".." in Path(absolute).parts:
        raise ProvisioningError(f"unsafe managed path: {absolute}")
    return root / absolute.removeprefix("/")


def private_write(path: Path, value: str, mode: int = 0o600) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    descriptor = os.open(temporary, flags, mode)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        os.chmod(path, mode)
    finally:
        temporary.unlink(missing_ok=True)


def _tls_pair(hostname: str, device_name: str) -> tuple[bytes, bytes]:
    key = rsa.generate_private_key(public_exponent=65537, key_size=3072)
    subject = issuer = x509.Name(
        [x509.NameAttribute(NameOID.COMMON_NAME, f"Studica robot {hostname}")]
    )
    now = datetime.now(timezone.utc)
    certificate = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - timedelta(minutes=5))
        .not_valid_after(now + timedelta(days=825))
        .add_extension(
            x509.SubjectAlternativeName(
                [
                    x509.DNSName("robot.local"),
                    x509.DNSName(f"{device_name}.local"),
                ]
            ),
            critical=False,
        )
        .add_extension(x509.BasicConstraints(ca=True, path_length=0), critical=True)
        .sign(key, hashes.SHA256())
    )
    return (
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.PKCS8,
            serialization.NoEncryption(),
        ),
        certificate.public_bytes(serialization.Encoding.PEM),
    )


def render_cyclonedds(companion_address: Optional[str]) -> str:
    peer = ""
    if companion_address:
        try:
            address = str(ipaddress.ip_address(companion_address))
        except ValueError as error:
            raise ProvisioningError("companion address must be an IP literal") from error
        peer = f'        <Peer Address="{address}" />\n'
    return f'''<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <AllowMulticast>false</AllowMulticast>
      <Interfaces><NetworkInterface autodetermine="true" multicast="false" /></Interfaces>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>32</MaxAutoParticipantIndex>
      <Peers AddLocalhost="true">
{peer}      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
'''


def provision(root: Path, companion_address: Optional[str] = None) -> dict[str, Any]:
    """Create idempotent per-device state below *root*."""
    root = canonical_root(root)
    state_root = root_path(root, "/var/lib/studica")
    state_root.mkdir(parents=True, exist_ok=True)
    identity_path = state_root / "device.json"
    created = not identity_path.exists()
    if created:
        device_id = str(uuid.uuid4())
        suffix = device_id.replace("-", "")[:8]
        identity = {
            "schema_version": 1,
            "device_id": device_id,
            # The appliance default is deliberately stable so a first-time user
            # can always open robot.local.  The immutable device_name remains
            # unique and is used for the hotspot and support identity.
            "hostname": "robot",
            "device_name": f"studica-{suffix}",
            "created_at": datetime.now(timezone.utc).isoformat(),
        }
        private_write(identity_path, json.dumps(identity, indent=2) + "\n")
    else:
        identity = json.loads(identity_path.read_text(encoding="utf-8"))
        try:
            device_id = str(uuid.UUID(str(identity["device_id"])))
        except (KeyError, ValueError) as error:
            raise ProvisioningError("stored device identity is invalid") from error
        device_name = f"studica-{device_id.replace('-', '')[:8]}"
        if identity.get("hostname") != "robot" or identity.get(
            "device_name"
        ) != device_name:
            identity["hostname"] = "robot"
            identity["device_name"] = device_name
            private_write(identity_path, json.dumps(identity, indent=2) + "\n")
    hostname = str(identity["hostname"])
    device_name = str(identity["device_name"])
    if hostname != "robot" or re.fullmatch(
        r"studica-[0-9a-f]{8}", device_name
    ) is None:
        raise ProvisioningError("stored device naming is invalid")

    secrets_root = state_root / "secrets"
    token_path = secrets_root / "api-token"
    hotspot_path = secrets_root / "hotspot-password"
    if not token_path.exists():
        private_write(token_path, secrets.token_urlsafe(32) + "\n")
    if not hotspot_path.exists():
        private_write(hotspot_path, secrets.token_urlsafe(18) + "\n")

    tls_root = state_root / "tls"
    key_path = tls_root / "robot.key"
    cert_path = tls_root / "robot.crt"
    if not key_path.exists() or not cert_path.exists():
        key, certificate = _tls_pair(hostname, device_name)
        private_write(key_path, key.decode("ascii"))
        private_write(cert_path, certificate.decode("ascii"), mode=0o644)

    for relative, mode in (
        ("maps", 0o750),
        ("pairing", 0o700),
        ("updates", 0o750),
        ("support", 0o750),
    ):
        directory = state_root / relative
        directory.mkdir(mode=mode, exist_ok=True)
        os.chmod(directory, mode)

    etc_studica = root_path(root, "/etc/studica")
    etc_studica.mkdir(parents=True, exist_ok=True)
    private_write(
        etc_studica / "companion-address",
        (companion_address or "") + "\n",
        mode=0o640,
    )
    private_write(
        etc_studica / "cyclonedds.xml",
        render_cyclonedds(companion_address),
        mode=0o644,
    )

    hostname_file = root_path(root, "/etc/hostname")
    private_write(hostname_file, hostname + "\n", mode=0o644)
    nm_root = root_path(root, "/etc/NetworkManager/system-connections")
    hotspot = f'''[connection]
id=Studica fallback hotspot
type=wifi
autoconnect=true
autoconnect-priority=-100

[wifi]
mode=ap
ssid=Studica-{device_name[-8:].upper()}
powersave=2

[wifi-security]
key-mgmt=wpa-psk
psk={hotspot_path.read_text(encoding="utf-8").strip()}

[ipv4]
method=shared

[ipv6]
method=disabled
'''
    private_write(nm_root / "studica-hotspot.nmconnection", hotspot)
    summary = {
        **identity,
        "created": created,
        "url": "https://robot.local",
        "hotspot_ssid": f"Studica-{device_name[-8:].upper()}",
        "token_file": "/var/lib/studica/secrets/api-token",
        "certificate_file": "/var/lib/studica/tls/robot.crt",
    }
    private_write(
        state_root / "provisioning.json", json.dumps(summary, indent=2) + "\n"
    )
    return summary


def validate_qualification(path: Path) -> dict[str, Any]:
    """Validate the explicit hardware evidence required before autostart."""
    document = json.loads(path.read_text(encoding="utf-8"))
    requirements = {
        "schema_version": 1,
        "independent_torque_removal": True,
        "failure_injection_passed": True,
        "zero_motion_all_boots": True,
    }
    for field, expected in requirements.items():
        if document.get(field) != expected:
            raise ProvisioningError(f"qualification field {field} must be {expected!r}")
    if int(document.get("cold_boots_passed", 0)) < 50:
        raise ProvisioningError("at least 50 cold boots must pass")
    if not document.get("tested_release_sha256"):
        raise ProvisioningError("qualification must identify the tested release")
    return document
