"""Independent companion installation and classroom identity contracts."""

import importlib.util
import json
from pathlib import Path
import sys

import pytest

from studica_robot_platform.provisioning import ProvisioningError, configure_domain, dds_port_range, provision

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("install_companion_test", ROOT / "scripts/install_companion.py")
INSTALL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(INSTALL)


def test_companions_keep_credentials_caches_and_domains_separate(tmp_path, monkeypatch):
    certificate = tmp_path / "input.crt"
    certificate.write_text("fixture certificate")
    token = tmp_path / "input.token"
    token.write_text("fixture credential")
    monkeypatch.setattr(INSTALL.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(INSTALL, "get_package_prefix", lambda _: str(tmp_path / "install"))
    monkeypatch.setattr(INSTALL, "get_package_share_directory", lambda _: str(ROOT))
    monkeypatch.setattr(INSTALL, "_robot_peer", lambda *args: "192.0.2.20")

    def install(name, domain):
        monkeypatch.setattr(sys, "argv", ["install_companion", "--no-enable", "--robot-id", name,
                                          "--domain-id", str(domain), "--robot-url", f"https://{name}.local",
                                          "--robot-certificate", str(certificate), "--token-file", str(token)])
        INSTALL.main()

    install("robot01", 11)
    install("robot02", 12)
    for name, domain in (("robot01", 11), ("robot02", 12)):
        config = tmp_path / ".config/studica/robots" / name
        assert json.loads((config / "session.json").read_text())["domain_id"] == domain
        assert (config / "token").stat().st_mode & 0o077 == 0
        runtime = (tmp_path / ".local/bin" / f"studica-companion-{name}").read_text()
        assert f"export ROS_DOMAIN_ID={domain}" in runtime
        assert str(config / "token") in runtime
        assert str(tmp_path / ".cache/studica" / name / "maps") in runtime
        assert (tmp_path / ".config/systemd/user" / f"studica-companion-{name}.service").exists()
    with pytest.raises(SystemExit, match="already uses"):
        install("robot03", 11)


def test_provisioning_preserves_peer_and_uses_unique_hostname(tmp_path):
    first = provision(tmp_path, "192.0.2.20")
    second = provision(tmp_path)
    assert first["device_id"] == second["device_id"]
    assert (tmp_path / "etc/hostname").read_text().strip() == first["device_name"]
    assert (tmp_path / "etc/studica/companion-address").read_text().strip() == "192.0.2.20"
    environment = tmp_path / "etc/studica/robot.env"
    environment.write_text("ROS_DOMAIN_ID=42\nROS_LOCALHOST_ONLY=0\n")
    configure_domain(tmp_path, 11)
    assert environment.read_text().count("ROS_DOMAIN_ID=") == 1
    assert "ROS_DOMAIN_ID=11" in environment.read_text()
    assert dds_port_range(11) == "10160:10225"


@pytest.mark.parametrize("domain", [-1, 101, True])
def test_invalid_domains_are_rejected(domain):
    with pytest.raises(ProvisioningError):
        dds_port_range(domain)
