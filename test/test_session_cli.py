#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Independent PC session contracts without hardware or services."""

import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
loader = importlib.machinery.SourceFileLoader("studica_cli", str(ROOT / "scripts/studica"))
spec = importlib.util.spec_from_loader(loader.name, loader)
CLI = importlib.util.module_from_spec(spec)
loader.exec_module(CLI)


class SessionTests(unittest.TestCase):
    def test_duplicate_domains_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)
            dds = path / "dds.xml"
            dds.write_text("<CycloneDDS/>")
            config = path / "sessions.json"
            config.write_text(json.dumps({"schema_version": 1, "sessions": {
                "robot01": {"kind": "robot", "domain_id": 11, "dds_config": str(dds)},
                "sim": {"kind": "sim", "domain_id": 11, "dds_config": str(dds)}}}))
            with self.assertRaisesRegex(ValueError, "distinct"):
                CLI.load_sessions(config)

    def test_child_environments_isolate_two_robots_and_sim(self):
        with patch.dict(os.environ, {"ROS_DOMAIN_ID": "99", "CYCLONEDDS_URI": "bad"}):
            environments = [CLI.environment({"kind": kind, "domain_id": domain,
                                            "dds_config": "/tmp/fixture.xml"})
                            for kind, domain in (("robot", 11), ("robot", 12), ("sim", 80))]
            self.assertEqual([env["ROS_DOMAIN_ID"] for env in environments], ["11", "12", "80"])
            self.assertEqual(environments[2]["STUDICA_USE_SIM_TIME"], "true")
            self.assertEqual(environments[0]["STUDICA_USE_SIM_TIME"], "false")
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "99")

    def test_same_application_argv_in_both_targets(self):
        command = ["ros2", "run", "robot_course_examples", "sensor_reporter"]
        for kind in ("robot", "sim"):
            result = CLI.command_for({"kind": kind}, "run", command)
            self.assertEqual(result[:len(command)], command)
            self.assertIn("use_sim_time:=" + ("true" if kind == "sim" else "false"), result)

    def test_remote_hardware_launch_and_mode_override_rejected(self):
        with self.assertRaises(ValueError):
            CLI.command_for({"kind": "robot"}, "launch", [])
        with self.assertRaises(ValueError):
            CLI.command_for({"kind": "sim"}, "mapping", ["mode:=hardware"])

    def test_update_requires_https_and_a_version(self):
        with self.assertRaises(ValueError):
            CLI.api_plan({"api_url": "http://robot.local"}, "update", ["v1"])
        self.assertEqual(CLI.api_plan({"api_url": "https://robot.local"}, "update", ["1.0"]),
                         ("POST", "https://robot.local/api/v1/updates/activate", {"version": "1.0"}))


if __name__ == "__main__":
    unittest.main()
