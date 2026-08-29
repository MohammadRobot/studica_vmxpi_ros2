#!/usr/bin/env python3
"""Contracts for the read-only VMXPi production runtime audit."""

from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
AUDIT_PATH = ROOT / "scripts" / "audit_vmxpi_runtime.py"
PROFILE_PATH = ROOT / "deployment" / "vmxpi-production-v1.json"
SPEC = importlib.util.spec_from_file_location("audit_vmxpi_runtime", AUDIT_PATH)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = AUDIT
SPEC.loader.exec_module(AUDIT)


class VmxpiRuntimeAuditTest(unittest.TestCase):

    def setUp(self):
        self.profile = AUDIT.load_profile(PROFILE_PATH)
        units = {
            unit: True for unit in self.profile["required_active_units"]
        }
        units.update(
            {unit: False for unit in self.profile["prohibited_active_units"]}
        )
        self.clean_snapshot = AUDIT.Snapshot(
            timestamp_utc="2026-08-29T15:00:00+00:00",
            os_id="ubuntu",
            version_id="26.04",
            architecture="arm64",
            hostname="studica-a1b2c3",
            system_state="running",
            cpu_count=4,
            load_1=0.2,
            load_5=0.3,
            load_15=0.4,
            memory_total_bytes=4 * 1024**3,
            memory_available_bytes=3 * 1024**3,
            swap_total_bytes=4 * 1024**3,
            swap_free_bytes=4 * 1024**3,
            root_total_bytes=64 * 1024**3,
            root_free_bytes=48 * 1024**3,
            temperature_c=42.0,
            journal_bytes=128 * 1024**2,
            unit_active=units,
            failed_units=(),
            installed_packages=(),
            listeners=(AUDIT.Listener("tcp", "0.0.0.0", 22),),
            ufw_enabled=True,
            sshd_settings={
                "passwordauthentication": "no",
                "permitrootlogin": "no",
                "pubkeyauthentication": "yes",
                "x11forwarding": "no",
            },
            process_markers=(),
            command_errors=(),
        )

    def test_profile_is_versioned_and_keeps_bluetooth(self):
        raw = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        self.assertEqual(raw["schema_version"], 1)
        self.assertIn("bluetooth.service", raw["required_active_units"])
        self.assertIn("hciuart.service", raw["required_active_units"])
        self.assertNotIn("bluetooth.service", raw["prohibited_active_units"])
        self.assertIn("xrdp.service", raw["prohibited_active_units"])
        self.assertIn("ros-humble-desktop", raw["prohibited_installed_packages"])
        self.assertEqual(raw["platform"]["version_id"], "26.04")

    def test_clean_appliance_snapshot_passes(self):
        self.assertEqual(
            AUDIT.evaluate_snapshot(self.clean_snapshot, self.profile), []
        )

    def test_unsafe_snapshot_reports_service_port_ssh_and_image_findings(self):
        units = dict(self.clean_snapshot.unit_active)
        units["xrdp.service"] = True
        unsafe = replace(
            self.clean_snapshot,
            hostname="vmx",
            system_state="degraded",
            journal_bytes=800 * 1024**2,
            unit_active=units,
            failed_units=("apport-autoreport.service",),
            installed_packages=("ros-humble-desktop", "xrdp"),
            listeners=(
                AUDIT.Listener("tcp", "0.0.0.0", 22),
                AUDIT.Listener("tcp", "*", 3389),
                AUDIT.Listener("udp", "0.0.0.0", 7400),
            ),
            ufw_enabled=False,
            sshd_settings={
                "passwordauthentication": "yes",
                "permitrootlogin": "without-password",
                "pubkeyauthentication": "yes",
                "x11forwarding": "yes",
            },
            process_markers=("xrdp",),
        )
        finding_ids = {
            finding.finding_id
            for finding in AUDIT.evaluate_snapshot(unsafe, self.profile)
        }
        expected = {
            "generic-hostname",
            "system-state",
            "failed-unit:apport-autoreport.service",
            "unneeded-unit:xrdp.service",
            "nonruntime-package:ros-humble-desktop",
            "nonruntime-package:xrdp",
            "nonruntime-processes",
            "public-tcp:3389",
            "public-udp:7400",
            "firewall-disabled",
            "sshd:passwordauthentication",
            "sshd:permitrootlogin",
            "sshd:x11forwarding",
            "journal-size",
        }
        self.assertTrue(expected.issubset(finding_ids))

    def test_listener_parser_distinguishes_loopback_and_public(self):
        listeners = AUDIT.parse_ss_listeners(
            "\n".join(
                (
                    "LISTEN 0 128 0.0.0.0:22 0.0.0.0:*",
                    "LISTEN 0 4096 127.0.0.53:53 0.0.0.0:*",
                    "LISTEN 0 2 [::1]:3350 [::]:*",
                    "LISTEN 0 2 *:3389 *:*",
                )
            )
        )
        by_port = {listener.port: listener for listener in listeners}
        self.assertTrue(by_port[22].public)
        self.assertFalse(by_port[53].public)
        self.assertFalse(by_port[3350].public)
        self.assertTrue(by_port[3389].public)
        udp = AUDIT.parse_ss_listeners(
            "UNCONN 0 0 *:7400 *:*", "udp"
        )[0]
        self.assertEqual(udp.protocol, "udp")
        self.assertTrue(udp.public)

    def test_sshd_config_fallback_resolves_include_and_defaults(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            config_dir = Path(temporary_dir)
            fragment_dir = config_dir / "sshd_config.d"
            fragment_dir.mkdir()
            (fragment_dir / "10-product.conf").write_text(
                "PasswordAuthentication no\nPermitRootLogin no\n",
                encoding="utf-8",
            )
            main = config_dir / "sshd_config"
            main.write_text(
                f"Include {fragment_dir}/*.conf\nX11Forwarding no\n",
                encoding="utf-8",
            )
            settings = AUDIT.parse_sshd_config(
                main,
                (
                    "passwordauthentication",
                    "permitrootlogin",
                    "pubkeyauthentication",
                    "x11forwarding",
                ),
            )
            self.assertEqual(settings["passwordauthentication"], "no")
            self.assertEqual(settings["permitrootlogin"], "no")
            self.assertEqual(settings["pubkeyauthentication"], "yes")
            self.assertEqual(settings["x11forwarding"], "no")

    def test_help_states_that_the_audit_is_read_only(self):
        result = subprocess.run(
            [str(AUDIT_PATH), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Read-only audit", result.stdout)
        self.assertNotIn("--apply", result.stdout)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
