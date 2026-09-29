import json
from pathlib import Path

from cryptography import x509
import pytest

from studica_robot_platform.orchestrator import UnitController, handle_request
from studica_robot_platform.provisioning import (
    ProvisioningError,
    provision,
    validate_qualification,
)
from studica_robot_platform.support import redact


ROOT = Path(__file__).resolve().parents[1]


class Result:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_orchestrator_has_fixed_unit_and_bluetooth_allowlists():
    calls = []

    def runner(arguments, **kwargs):
        calls.append(arguments)
        if arguments[-1] == "devices":
            return Result(stdout="Device AA:BB:CC:DD:EE:FF Wireless Controller\n")
        return Result(stdout="active\n")

    controller = UnitController(
        runner=runner,
        camera_resource_probe=lambda: (True, "test resources available"),
    )
    assert handle_request(
        {"action": "set_sensor", "sensor": "camera", "enabled": True}, controller
    )["ok"]
    assert not handle_request(
        {"action": "set_sensor", "sensor": "../../ssh", "enabled": True}, controller
    )["ok"]
    assert not handle_request(
        {"action": "bluetooth_pair", "address": "--help"}, controller
    )["ok"]
    devices = handle_request(
        {"action": "bluetooth_devices", "scan": False}, controller
    )
    assert devices["devices"][0]["name"] == "Wireless Controller"
    assert all(isinstance(call, list) for call in calls)


def test_camera_start_is_rejected_when_resource_preflight_fails():
    controller = UnitController(
        runner=lambda *args, **kwargs: Result(),
        camera_resource_probe=lambda: (False, "compute busy"),
    )
    result = controller.set_active("studica-camera.service", True)
    assert not result["ok"]
    assert result["error"] == "compute busy"


def test_orchestrator_configures_only_a_routable_companion_peer(tmp_path: Path):
    calls = []

    def runner(arguments, **kwargs):
        del kwargs
        calls.append(arguments)
        return Result()

    address = tmp_path / "companion-address"
    cyclone = tmp_path / "cyclonedds.xml"
    controller = UnitController(
        runner=runner,
        companion_address_path=address,
        cyclonedds_path=cyclone,
        transport_state_root=tmp_path / "transport",
        domain_id=11,
    )
    assert not controller.configure_companion_peer("127.0.0.1")["ok"]
    result = controller.configure_companion_peer("192.0.2.20")
    assert result["ok"] and result["restart_scheduled"]
    assert address.read_text() == "192.0.2.20\n"
    assert '<Peer Address="192.0.2.20"' in cyclone.read_text()
    assert "studica-peer-apply.timer" in calls[-1]
    assert controller.companion_transport(True)["ok"]
    assert "10160:10225" in calls[-1]
    assert (tmp_path / "transport/companion").is_file()
    assert controller.companion_transport(False)["ok"]
    assert not (tmp_path / "transport/companion").exists()


def test_dds_firewall_does_not_commit_owner_when_allow_fails(tmp_path: Path):
    calls = []

    def runner(arguments, **kwargs):
        del kwargs
        calls.append(arguments)
        if arguments[2:4] == ["allow", "in"]:
            return Result(returncode=1, stderr="injected firewall failure")
        return Result()

    address = tmp_path / "companion-address"
    address.write_text("192.0.2.20\n", encoding="utf-8")
    controller = UnitController(
        runner=runner,
        companion_address_path=address,
        transport_state_root=tmp_path / "transport",
    )
    result = controller.companion_transport(True)
    assert not result["ok"]
    assert result["error"] == "injected firewall failure"
    assert not (tmp_path / "transport/companion").exists()
    assert calls[0][2:4] == ["deny", "out"]


def test_support_redaction_removes_network_and_secret_values():
    value = redact(
        "token=abc password: value 192.168.1.10 AA:BB:CC:DD:EE:FF"
    )
    assert "abc" not in value
    assert "value" not in value
    assert "192.168.1.10" not in value
    assert "AA:BB:CC:DD:EE:FF" not in value


def test_first_boot_is_idempotent_and_private(tmp_path: Path):
    first = provision(tmp_path)
    second = provision(tmp_path)
    assert first["device_id"] == second["device_id"]
    token = tmp_path / "var/lib/studica/secrets/api-token"
    key = tmp_path / "var/lib/studica/tls/robot.key"
    assert token.stat().st_mode & 0o077 == 0
    assert key.stat().st_mode & 0o077 == 0
    pairing = tmp_path / "var/lib/studica/pairing"
    assert pairing.stat().st_mode & 0o077 == 0
    assert (tmp_path / "etc/hostname").read_text() == first["device_name"] + "\n"
    assert first["device_name"].startswith("studica-")
    assert first["hotspot_ssid"].endswith(first["device_name"][-8:].upper())
    certificate = x509.load_pem_x509_certificate(
        (tmp_path / "var/lib/studica/tls/robot.crt").read_bytes()
    )
    names = certificate.extensions.get_extension_for_class(
        x509.SubjectAlternativeName
    ).value.get_values_for_type(x509.DNSName)
    assert "robot.local" in names
    assert f"{first['device_name']}.local" in names
    assert "Studica-" in (
        tmp_path
        / "etc/NetworkManager/system-connections/studica-hotspot.nmconnection"
    ).read_text()


def test_autostart_qualification_requires_all_safety_evidence(tmp_path: Path):
    report = tmp_path / "qualification.json"
    report.write_text(json.dumps({"schema_version": 1, "cold_boots_passed": 49}))
    with pytest.raises(ProvisioningError):
        validate_qualification(report)
    report.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "cold_boots_passed": 50,
                "independent_torque_removal": True,
                "failure_injection_passed": True,
                "zero_motion_all_boots": True,
                "tested_release_sha256": "a" * 64,
            }
        )
    )
    assert validate_qualification(report)["cold_boots_passed"] == 50


def test_systemd_contract_keeps_camera_on_demand_and_final_topic_unique():
    target = (ROOT / "deployment/systemd/studica-robot.target").read_text()
    hardware = (ROOT / "deployment/systemd/studica-hardware.service").read_text()
    wrapper = (ROOT / "scripts/studica_hardware_runtime").read_text()
    installer = (ROOT / "scripts/install_robot_platform.py").read_text()
    peer_timer = (
        ROOT / "deployment/systemd/studica-peer-apply.timer"
    ).read_text()
    assert "studica-lidar.service" in target
    assert "studica-camera.service" not in target
    assert "Requires=studica-firewall.service" in target
    assert "Requires=studica-update-recovery.service" in target
    assert "PartOf=studica-robot.target" in hardware
    assert "safety_input_cmd_vel_topic:=/robot/platform/cmd_vel" in wrapper
    assert 'frozenset({"studica-companion.service"})' in installer
    assert "OnActiveSec=3s" in peer_timer
    guard = (ROOT / "scripts/studica_network_guard").read_text()
    assert "ufw default deny incoming" in guard
    assert "ufw --force reset" in guard
    assert 'ufw deny out to "${studica_companion_peer}"' in guard
    assert 'ufw allow 443/tcp comment "Studica HTTPS API"' in guard


def test_boot_services_and_update_environment():
    target = (ROOT / "deployment/systemd/studica-robot.target").read_text()
    assert "network-online.target" not in target
    for name in ("studica-update.service", "studica-update-activate@.service", "studica-update-recovery.service"):
        unit = (ROOT / "deployment/systemd" / name).read_text()
        assert "studica_hardware_runtime update " in unit
    wrapper = (ROOT / "scripts/studica_hardware_runtime").read_text()
    assert 'studica_update_agent.py "$@"' in wrapper
