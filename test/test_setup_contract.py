#!/usr/bin/env python3
"""Non-mutating contract tests for the Ubuntu classroom installer."""

from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
SETUP = ROOT / "scripts" / "setup_ubuntu.sh"
CYCLONEDDS_SETUP = ROOT / "scripts" / "configure_cyclonedds.py"
APT_BUNDLE_DIR = ROOT / "dependencies" / "apt"
PIN = "a2d290e37be67ba082744e323339d82031f051c0"
COMMIT = re.compile(r"[0-9a-f]{40}")


class SetupContractTest(unittest.TestCase):

    def test_shell_syntax_and_help(self):
        syntax = subprocess.run(["bash", "-n", str(SETUP)], check=False)
        self.assertEqual(syntax.returncode, 0)
        help_result = subprocess.run(
            [str(SETUP), "--help"], check=False, capture_output=True, text=True
        )
        self.assertEqual(help_result.returncode, 0)
        self.assertIn("--mode simulation|hardware", help_result.stdout)
        self.assertIn("--check-only", help_result.stdout)
        self.assertIn("--non-interactive", help_result.stdout)

    def test_invalid_mode_is_a_usage_error(self):
        result = subprocess.run(
            [str(SETUP), "--mode", "invalid"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 2)
        self.assertIn("simulation or hardware", result.stderr)

    def test_manifests_are_separate_and_overlay_is_pinned(self):
        simulation = yaml.safe_load(
            ROOT.joinpath("dependencies/simulation.repos").read_text(encoding="utf-8")
        )["repositories"]
        hardware = yaml.safe_load(
            ROOT.joinpath("dependencies/hardware.repos").read_text(encoding="utf-8")
        )["repositories"]
        self.assertEqual(simulation["gz_ros2_control"]["version"], PIN)
        self.assertIn("studica_robot_monitor", simulation)
        self.assertNotIn("OrbbecSDK_ROS2", simulation)
        self.assertIn("OrbbecSDK_ROS2", hardware)
        self.assertIn("ydlidar_ros2_driver", hardware)
        for manifest in (simulation, hardware):
            for repository, metadata in manifest.items():
                with self.subTest(repository=repository):
                    self.assertRegex(str(metadata["version"]), COMMIT)
        for repository in simulation.keys() & hardware.keys():
            self.assertEqual(
                simulation[repository]["version"], hardware[repository]["version"]
            )

    def test_classroom_apt_dependencies_are_explicit_mode_bundles(self):
        def packages(filename):
            return {
                line.strip()
                for line in APT_BUNDLE_DIR.joinpath(filename)
                .read_text(encoding="utf-8")
                .splitlines()
                if line.strip() and not line.lstrip().startswith("#")
            }

        core = packages("development-core.txt")
        desktop = packages("development-desktop.txt")
        simulation = packages("simulation-harmonic.txt")
        self.assertIn("ros-humble-ros2-control", core)
        self.assertNotIn("ros-humble-desktop", core)
        self.assertIn("ros-humble-desktop", desktop)
        self.assertIn("ros-humble-nav2-bringup", desktop)
        self.assertIn("gz-harmonic", simulation)
        self.assertTrue(core.isdisjoint(desktop))
        self.assertTrue(core.isdisjoint(simulation))
        self.assertTrue(desktop.isdisjoint(simulation))

        setup = SETUP.read_text(encoding="utf-8")
        for filename in (
            "development-core.txt",
            "development-desktop.txt",
            "simulation-harmonic.txt",
        ):
            self.assertIn(filename, setup)

    def test_installer_never_starts_ros_or_edits_shell_startup(self):
        source = SETUP.read_text(encoding="utf-8")
        forbidden = (
            "ros2 launch",
            "ros2 topic pub",
            "validate_motors",
            ">> ~/.bashrc",
            ">> \"$HOME/.bashrc\"",
        )
        for marker in forbidden:
            with self.subTest(marker=marker):
                self.assertNotIn(marker, source)
        self.assertIn("--return-code-on-test-failure", source)

    def test_cyclonedds_profiles_are_generated_without_editing_startup(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            simulation = subprocess.run(
                [
                    str(CYCLONEDDS_SETUP),
                    "sim",
                    "--domain-id",
                    "7",
                    "--output-dir",
                    temporary_dir,
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(simulation.returncode, 0, simulation.stderr)
            environment = Path(temporary_dir, "studica_sim.env").read_text(
                encoding="utf-8"
            )
            self.assertIn("ROS_DOMAIN_ID=7", environment)
            self.assertIn("GZ_VERSION=harmonic", environment)
            self.assertIn("cyclonedds_sim.xml", environment)

            peer = subprocess.run(
                [
                    str(CYCLONEDDS_SETUP),
                    "peer",
                    "--name",
                    "pc_wifi",
                    "--local-address",
                    "192.0.2.10",
                    "--peer-address",
                    "192.0.2.20",
                    "--interface",
                    "test0",
                    "--domain-id",
                    "1",
                    "--skip-interface-check",
                    "--output-dir",
                    temporary_dir,
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(peer.returncode, 0, peer.stderr)
            self.assertIn("7660:7761", peer.stdout)
            peer_xml = Path(temporary_dir, "cyclonedds_pc_wifi.xml").read_text(
                encoding="utf-8"
            )
            self.assertIn('address="192.0.2.10"', peer_xml)
            self.assertIn('Address="192.0.2.20"', peer_xml)
            peer_environment = Path(temporary_dir, "studica_pc_wifi.env").read_text(
                encoding="utf-8"
            )
            self.assertNotIn("GZ_VERSION", peer_environment)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
