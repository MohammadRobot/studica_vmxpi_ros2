#!/usr/bin/env python3
"""Contracts for the versioned VMXPi production package manifest."""

from copy import deepcopy
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
VALIDATOR_PATH = ROOT / "scripts" / "validate_production_manifest.py"
MANIFEST_PATH = ROOT / "deployment" / "vmxpi-runtime-packages-v1.json"
SPEC = importlib.util.spec_from_file_location(
    "validate_production_manifest", VALIDATOR_PATH
)
assert SPEC is not None and SPEC.loader is not None
VALIDATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATOR)


class ProductionManifestTest(unittest.TestCase):

    def setUp(self):
        self.manifest = VALIDATOR.load_json(MANIFEST_PATH)

    def test_checked_in_manifest_passes(self):
        self.assertEqual(VALIDATOR.validate_manifest(ROOT, self.manifest), [])

    def test_enabled_runtime_is_headless_and_keeps_bluetooth(self):
        packages = set(VALIDATOR.enabled_apt_packages(self.manifest))
        self.assertIn("ros-humble-ros-base", packages)
        self.assertIn("ros-humble-joy", packages)
        self.assertIn("ros-humble-teleop-twist-joy", packages)
        self.assertNotIn("ros-humble-desktop", packages)
        self.assertNotIn("ubuntu-desktop", packages)
        self.assertNotIn("ros-humble-foxglove-bridge", packages)
        self.assertNotIn("ros-humble-nav2-bringup", packages)

    def test_robot_core_manifest_excludes_developer_dependencies(self):
        failures = []
        VALIDATOR.validate_dependency_boundary(ROOT, failures)
        self.assertEqual(failures, [])

    def test_optional_foxglove_is_off_by_default(self):
        robot_launch = ROOT.joinpath(
            "bringup", "launch", "robot.launch.py"
        ).read_text(encoding="utf-8")
        self.assertRegex(
            robot_launch,
            r'"use_foxglove",\s+default_value="false"',
        )
        bringup_launch = ROOT.joinpath(
            "bringup", "launch", "bringup.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'if not use_foxglove:\n        use_foxglove = "false"',
            bringup_launch,
        )

    def test_platform_cannot_drift_from_selected_baseline(self):
        changed = deepcopy(self.manifest)
        changed["platform"]["version_id"] = "24.04"
        failures = VALIDATOR.validate_manifest(ROOT, changed)
        self.assertTrue(any("platform must equal" in failure for failure in failures))
        self.assertTrue(any("platform.version_id" in failure for failure in failures))

    def test_desktop_or_duplicate_package_is_rejected(self):
        changed = deepcopy(self.manifest)
        changed["required_apt_packages"]["ros_core"].append("ros-humble-desktop")
        changed["required_apt_packages"]["ros_core"].sort()
        changed["features"]["foxglove"]["apt_packages"].append(
            "ros-humble-ros-base"
        )
        changed["features"]["foxglove"]["apt_packages"].sort()
        failures = VALIDATOR.validate_manifest(ROOT, changed)
        self.assertIn(
            "enabled package is prohibited: ros-humble-desktop",
            failures,
        )
        self.assertTrue(any("occur in multiple groups" in failure for failure in failures))

    def test_cli_prints_only_enabled_package_names(self):
        result = subprocess.run(
            [
                sys.executable,
                str(VALIDATOR_PATH),
                "--root",
                str(ROOT),
                "--print-enabled-apt",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        packages = result.stdout.splitlines()
        self.assertEqual(packages, sorted(packages))
        self.assertIn("ros-humble-ros-base", packages)
        self.assertNotIn("ros-humble-foxglove-bridge", packages)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
