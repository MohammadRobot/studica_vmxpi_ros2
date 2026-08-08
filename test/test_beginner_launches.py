#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Source-tree contract checks for the beginner launch surface."""

import importlib.util
from pathlib import Path
import py_compile
import sys
import unittest

import yaml


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
LAUNCH_DIR = ROOT / "bringup" / "launch"
if str(LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(LAUNCH_DIR))


class BeginnerLaunchContractTest(unittest.TestCase):
    def test_only_intended_public_launches_are_exposed(self):
        public_launches = {
            path.name for path in LAUNCH_DIR.glob("*.launch.py") if not path.name.startswith("_")
        }
        self.assertEqual(
            public_launches,
            {
                "bringup.launch.py",
                "mapping.launch.py",
                "navigation.launch.py",
                "robot.launch.py",
                "sim.launch.py",
            },
        )

    def test_launch_python_is_syntax_valid(self):
        for path in LAUNCH_DIR.glob("*.py"):
            with self.subTest(path=path.name):
                py_compile.compile(str(path), doraise=True)

    def test_every_launch_description_constructs(self):
        for index, path in enumerate(sorted(LAUNCH_DIR.glob("*.launch.py"))):
            with self.subTest(path=path.name):
                spec = importlib.util.spec_from_file_location(
                    f"studica_launch_contract_{index}", path
                )
                self.assertIsNotNone(spec)
                self.assertIsNotNone(spec.loader)
                module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(module)
                description = module.generate_launch_description()
                self.assertGreater(len(description.entities), 0)

    def test_beginner_defaults_and_no_embedded_teleop(self):
        expected = {
            "sim.launch.py": (
                '"maze"',
                '"class_4wd"',
                '"sim_enable_camera": use_camera',
                '"use_lidar": "true"',
                '"use_monitoring": "true"',
            ),
            "robot.launch.py": ('"mode": "hardware"', '"use_monitoring": "true"', '"127.0.0.1"'),
            "mapping.launch.py": ('"office_map"', '"slam_toolbox"'),
            "navigation.launch.py": ('"office_map.yaml"', '"nav2_bringup"'),
        }
        for filename, markers in expected.items():
            source = (LAUNCH_DIR / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                for marker in markers:
                    self.assertIn(marker, source)
                self.assertNotIn("teleop_twist", source.lower())
                self.assertNotIn("joystick", source.lower())
                self.assertNotIn("patrol", source.lower())

    def test_advanced_launch_uses_private_standard_topic_runtime(self):
        bringup = (LAUNCH_DIR / "bringup.launch.py").read_text(encoding="utf-8")
        runtime = (LAUNCH_DIR / "_robot_runtime.launch.py").read_text(encoding="utf-8")
        self.assertIn('"_robot_runtime.launch.py"', bringup)
        self.assertIn('"class_4wd"', bringup)
        self.assertNotIn("joystick", bringup.lower())
        self.assertIn('"input_cmd_vel_topic": "/cmd_vel"', runtime)
        self.assertIn('"output_odom_topic": "/odom"', runtime)
        self.assertIn('"mock_scan_output_topic": "/scan"', runtime)

    def test_removed_generated_and_legacy_files_stay_removed(self):
        removed = (
            "bringup/launch/robot_gz_sim.launch.py",
            "bringup/launch/robot_bringup.launch.py",
            "bringup/launch/start_patrolling.launch.py",
            "bringup/config/profiles/training_2wd",
            "bringup/config/profiles/training_4wd",
            "maps/my_map.yaml",
            "maps/my_map.pgm",
            "scripts/motor_smoke_test.sh",
            "src/patrol.cpp",
        )
        for relative_path in removed:
            with self.subTest(path=relative_path):
                self.assertFalse((ROOT / relative_path).exists())

        self.assertTrue((ROOT / "maps" / "office_map.yaml").is_file())
        self.assertTrue((ROOT / "maps" / "office_map.pgm").is_file())

    def test_classroom_drive_controller_has_a_short_command_timeout(self):
        controller_file = (
            ROOT / "bringup" / "config" / "profiles" / "class_4wd" / "robot_controllers.yaml"
        )
        controllers = yaml.safe_load(controller_file.read_text(encoding="utf-8"))
        timeout = controllers["robot_base_controller"]["ros__parameters"]["cmd_vel_timeout"]
        self.assertGreater(float(timeout), 0.0)
        self.assertLessEqual(float(timeout), 0.5)


if __name__ == "__main__":
    # CTest passes ROOT as argv[1]; keep unittest from treating it as a test name.
    unittest.main(argv=[sys.argv[0]])
