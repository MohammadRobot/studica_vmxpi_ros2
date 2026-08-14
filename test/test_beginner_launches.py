#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Source-tree contract checks for the beginner launch surface."""

import importlib.util
from pathlib import Path
import py_compile
import sys
import unittest
from unittest.mock import patch

import yaml


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
LAUNCH_DIR = ROOT / "bringup" / "launch"
if str(LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(LAUNCH_DIR))

from _launch_helpers import _profile_assets  # noqa: E402


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

    def test_beginner_defaults_and_joystick_policy(self):
        expected = {
            "sim.launch.py": (
                '"maze"',
                '"class_4wd"',
                '"use_point_cloud"',
                'executable="depth_to_pointcloud.py"',
                'executable="point_cloud_filter.py"',
                '"output_topic": "/camera/depth/points_filtered"',
                '"sim_enable_camera": PythonExpression',
                '"use_lidar": "true"',
                '"use_monitoring": "true"',
                'default_value="true"',
                '"use_joystick": use_joystick',
            ),
            "robot.launch.py": (
                '"mode": "hardware"',
                'default_value="stack_4wd"',
                '"robot_profile": robot_profile',
                '"use_lidar": use_lidar',
                '"use_monitoring": "true"',
                '"127.0.0.1"',
                'default_value="false"',
                '"use_joystick": use_joystick',
                '"use_camera_color"',
                '"use_colored_depth_cloud"',
                '"orbbec_depth_registration": use_colored_depth_cloud',
                "'320' if '",
                '"use_point_cloud"',
                '"use_point_cloud_filter"',
                '"orbbec_depth_width": "320"',
                '"orbbec_depth_height": "240"',
                '"orbbec_depth_fps": PythonExpression',
                '"stride": 2',
                'use_point_cloud_filter,',
            ),
            "mapping.launch.py": (
                '"office_map"',
                '"slam_toolbox"',
                'choices=["gz_sim", "hardware"]',
                '"slam_toolbox_hardware_mapper_params.yaml"',
                'hardware_client = GroupAction(',
                'package="rviz2"',
                'arguments=["-d", rviz_config_file, "-f", "map"]',
                'package="joy"',
                'package="teleop_twist_joy"',
                '"scale_linear.x": 0.08',
                '"scale_angular.yaw": 0.25',
                'default_value="true"',
                '"use_joystick": use_joystick',
            ),
            "navigation.launch.py": (
                '"office_map.yaml"',
                "'stack_4wd' if '",
                '"robot_profile": robot_profile',
                '"nav2_bringup"',
                'choices=["gz_sim", "hardware"]',
                '"real_robot_map.yaml"',
                'hardware_client = GroupAction(',
                'package="rviz2"',
                '"use_point_cloud"',
                'executable="depth_to_pointcloud.py"',
                'executable="point_cloud_filter.py"',
                '"output_topic": "/camera/depth/points_filtered"',
                'Nav2PointCloudParams(',
                'hardware_mode=hardware_mode',
                '"hardware_max_linear_speed"',
                '"hardware_max_angular_speed"',
                'default_value="false"',
                '"use_joystick": use_joystick',
            ),
        }
        for filename, markers in expected.items():
            source = (LAUNCH_DIR / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                for marker in markers:
                    self.assertIn(marker, source)
                self.assertNotIn("patrol", source.lower())

    def test_hardware_camera_defaults_are_low_load_and_point_cloud_is_opt_in(self):
        expected_markers = {
            "bringup.launch.py": (
                '"orbbec_enable_point_cloud",\n                "false"',
                '"orbbec_enable_color",\n                "false"',
                '"orbbec_enable_depth",\n                "true"',
                '"orbbec_depth_width",\n                "320"',
                '"orbbec_depth_height",\n                "240"',
                '"orbbec_depth_fps",\n                "5"',
            ),
            "_robot_runtime.launch.py": (
                '"orbbec_enable_point_cloud",\n            "false"',
                '"orbbec_enable_color",\n            "false"',
                '"orbbec_enable_depth",\n            "true"',
                '"orbbec_depth_width",\n            "320"',
                '"orbbec_depth_height",\n            "240"',
                '"orbbec_depth_fps",\n            "5"',
            ),
            "_camera_hw.launch.py": (
                '"orbbec_enable_point_cloud",\n            default_value="false"',
                '"orbbec_enable_color",\n            default_value="false"',
                '"orbbec_enable_depth",\n            default_value="true"',
                '"orbbec_depth_width",\n            default_value="320"',
                '"orbbec_depth_height",\n            default_value="240"',
                '"orbbec_depth_fps",\n            default_value="5"',
            ),
        }
        for filename, markers in expected_markers.items():
            source = (LAUNCH_DIR / filename).read_text(encoding="utf-8")
            with self.subTest(filename=filename):
                for marker in markers:
                    self.assertIn(marker, source)

    def test_advanced_launch_uses_private_standard_topic_runtime(self):
        bringup = (LAUNCH_DIR / "bringup.launch.py").read_text(encoding="utf-8")
        runtime = (LAUNCH_DIR / "_robot_runtime.launch.py").read_text(encoding="utf-8")
        self.assertIn('"_robot_runtime.launch.py"', bringup)
        self.assertIn('"class_4wd"', bringup)
        self.assertIn('package="joy"', bringup)
        self.assertIn('package="teleop_twist_joy"', bringup)
        self.assertIn('"use_joystick",', bringup)
        self.assertIn('"input_cmd_vel_topic": "/cmd_vel"', runtime)
        self.assertIn('"output_odom_topic": "/odom"', runtime)
        self.assertIn('"mock_scan_output_topic": "/scan"', runtime)
        self.assertIn(
            '"monitor_color_camera": LaunchConfiguration("orbbec_enable_color")',
            runtime,
        )
        self.assertIn(
            '"monitor_depth_camera": LaunchConfiguration("orbbec_enable_depth")',
            runtime,
        )

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

    def test_hardware_mapping_profile_prioritizes_scan_overlap(self):
        params_file = (
            ROOT
            / "bringup"
            / "config"
            / "slam_toolbox_hardware_mapper_params.yaml"
        )
        document = yaml.safe_load(params_file.read_text(encoding="utf-8"))
        params = document["slam_toolbox"]["ros__parameters"]

        self.assertEqual(params["odom_frame"], "odom")
        self.assertEqual(params["base_frame"], "base_link")
        self.assertEqual(params["scan_topic"], "/scan")
        self.assertEqual(params["throttle_scans"], 1)
        self.assertLessEqual(float(params["resolution"]), 0.05)
        self.assertLessEqual(float(params["minimum_travel_distance"]), 0.02)
        self.assertLessEqual(float(params["minimum_travel_heading"]), 0.02)
        self.assertGreaterEqual(int(params["scan_buffer_size"]), 10)
        self.assertFalse(params["use_scan_matching"])
        self.assertTrue(params["do_loop_closing"])
        self.assertLessEqual(float(params["loop_search_maximum_distance"]), 2.0)
        self.assertGreaterEqual(int(params["loop_match_minimum_chain_size"]), 15)
        self.assertLessEqual(float(params["loop_match_maximum_variance_coarse"]), 1.0)
        self.assertGreaterEqual(float(params["loop_match_minimum_response_coarse"]), 0.5)
        self.assertGreaterEqual(float(params["loop_match_minimum_response_fine"]), 0.6)

    def test_stack_4wd_uses_confirmed_physical_geometry_and_calibration(self):
        profile_dir = ROOT / "bringup" / "config" / "profiles" / "stack_4wd"
        profile = yaml.safe_load(
            profile_dir.joinpath("robot_profile.yaml").read_text(encoding="utf-8")
        )
        geometry = profile["xacro"]
        drive = profile["drive"]
        controllers = yaml.safe_load(
            profile_dir.joinpath("robot_controllers.yaml").read_text(encoding="utf-8")
        )
        controller = controllers["robot_base_controller"]["ros__parameters"]

        self.assertEqual(
            {
                "base_length": geometry["base_length"],
                "base_width": geometry["base_width"],
                "base_height": geometry["base_height"],
                "ground_clearance": geometry["ground_clearance"],
                "wheelbase": geometry["wheelbase"],
                "wheel_track": geometry["wheel_track"],
                "overall_length": geometry["overall_length"],
                "overall_width": geometry["overall_width"],
            },
            {
                "base_length": 0.34,
                "base_width": 0.29,
                "base_height": 0.10,
                "ground_clearance": 0.035,
                "wheelbase": 0.19,
                "wheel_track": 0.34,
                "overall_length": 0.35,
                "overall_width": 0.385,
            },
        )
        self.assertAlmostEqual(drive["wheel_radius_m"], 0.060)
        self.assertAlmostEqual(controller["wheel_separation"], 0.5)
        self.assertAlmostEqual(controller["left_wheel_radius_multiplier"], 1.0173)
        self.assertAlmostEqual(controller["right_wheel_radius_multiplier"], 1.0173)
        self.assertAlmostEqual(geometry["imu_yaw"], 1.5707963267948966)
        self.assertTrue(geometry["use_chassis_mesh"])
        self.assertAlmostEqual(geometry["chassis_mesh_scale"], 1.0)

        modeled_mass = geometry["base_mass"] + 4.0 * geometry["wheel_mass"] + 0.01
        self.assertAlmostEqual(modeled_mass, geometry["robot_mass"])

        description = ROOT.joinpath(
            "description", "robot", "urdf", "robot_description.urdf.xacro"
        ).read_text(encoding="utf-8")
        self.assertIn('${wheelbase/2} ${wheel_track/2}', description)
        self.assertIn("${imu_pos_x} ${imu_pos_y} ${imu_pos_z}", description)
        self.assertIn("${cam_pos_x} ${cam_pos_y} ${cam_pos_z}", description)
        self.assertIn("${chassis_mesh_scale} ${chassis_mesh_scale}", description)

    def test_hardware_imu_acceleration_stays_in_the_sensor_frame(self):
        hardware_source = ROOT.joinpath("hardware", "vmx_system.cpp").read_text(
            encoding="utf-8"
        )
        for axis in "XYZ":
            self.assertIn(f"GetRawAccel{axis}()", hardware_source)
            self.assertNotIn(f"GetWorldLinearAccel{axis}()", hardware_source)

    def test_hardware_controller_rate_override_preserves_simulation_profile(self):
        with patch(
            "_launch_helpers.get_package_share_directory",
            return_value=str(ROOT / "bringup"),
        ):
            _profile, hardware_controllers_file = _profile_assets(
                "class_4wd", control_rate_hz="50"
            )
            _profile, simulation_controllers_file = _profile_assets("class_4wd")

        hardware = yaml.safe_load(
            Path(hardware_controllers_file).read_text(encoding="utf-8")
        )
        simulation = yaml.safe_load(
            Path(simulation_controllers_file).read_text(encoding="utf-8")
        )

        self.assertNotEqual(
            Path(hardware_controllers_file).parent,
            Path(simulation_controllers_file).parent,
        )
        self.assertEqual(
            hardware["controller_manager"]["ros__parameters"]["update_rate"],
            50,
        )
        self.assertEqual(
            hardware["robot_base_controller"]["ros__parameters"]["publish_rate"],
            50.0,
        )
        self.assertEqual(
            simulation["controller_manager"]["ros__parameters"]["update_rate"],
            100,
        )
        self.assertEqual(
            simulation["robot_base_controller"]["ros__parameters"]["publish_rate"],
            100.0,
        )

    def test_hardware_imu_odometry_has_one_tf_owner(self):
        with patch(
            "_launch_helpers.get_package_share_directory",
            return_value=str(ROOT / "bringup"),
        ):
            _profile, fused_controllers_file = _profile_assets(
                "class_4wd",
                control_rate_hz="25",
                enable_odom_tf=False,
            )

        fused_controllers = yaml.safe_load(
            Path(fused_controllers_file).read_text(encoding="utf-8")
        )
        self.assertFalse(
            fused_controllers["robot_base_controller"]["ros__parameters"][
                "enable_odom_tf"
            ]
        )

        ekf = yaml.safe_load(
            ROOT.joinpath("bringup", "config", "hardware_imu_odometry.yaml").read_text(
                encoding="utf-8"
            )
        )["hardware_odometry_filter"]["ros__parameters"]
        self.assertTrue(ekf["publish_tf"])
        self.assertTrue(ekf["two_d_mode"])
        self.assertEqual(ekf["world_frame"], "odom")
        self.assertEqual(ekf["base_link_frame"], "base_footprint")
        self.assertEqual(ekf["odom0"], "/wheel/odom")
        self.assertEqual(ekf["imu0"], "/imu")
        self.assertTrue(ekf["odom0_config"][6])
        self.assertEqual(sum(ekf["odom0_config"]), 1)
        self.assertTrue(ekf["imu0_config"][5])
        self.assertTrue(ekf["imu0_config"][11])
        self.assertEqual(sum(ekf["imu0_config"]), 2)

        runtime = (LAUNCH_DIR / "_robot_runtime.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('package="robot_localization"', runtime)
        self.assertIn('"output_odom_topic": "/wheel/odom"', runtime)
        self.assertIn('remappings=[("odometry/filtered", "/odom")]', runtime)
        self.assertIn('enable_odom_tf=False if use_imu_odometry else None', runtime)

        robot_launch = (LAUNCH_DIR / "robot.launch.py").read_text(encoding="utf-8")
        self.assertIn('"use_imu_odometry"', robot_launch)
        self.assertIn('default_value="true"', robot_launch)

        package_xml = ROOT.joinpath("package.xml").read_text(encoding="utf-8")
        setup = ROOT.joinpath("scripts", "setup_ubuntu.sh").read_text(encoding="utf-8")
        self.assertIn("<exec_depend>robot_localization</exec_depend>", package_xml)
        self.assertIn("ros-humble-robot-localization", setup)

    def test_hardware_controller_rate_defaults_to_validated_25_hz(self):
        expected_fragments = {
            "robot.launch.py": (
                'DeclareLaunchArgument( "hardware_control_rate_hz", '
                'default_value="25",'
            ),
            "bringup.launch.py": '_declare_arg( "hardware_control_rate_hz", "25",',
            "_robot_runtime.launch.py": '_declare_arg( "hardware_control_rate_hz", "25",',
        }

        for filename, fragment in expected_fragments.items():
            with self.subTest(filename=filename):
                source = (LAUNCH_DIR / filename).read_text(encoding="utf-8")
                compact_source = " ".join(source.split())
                self.assertIn(fragment, compact_source)

    def test_hardware_controller_rate_must_be_a_positive_integer(self):
        with patch(
            "_launch_helpers.get_package_share_directory",
            return_value=str(ROOT / "bringup"),
        ):
            for value in ("0", "-1", "50.5", "invalid"):
                with self.subTest(value=value):
                    with self.assertRaises(ValueError):
                        _profile_assets("class_4wd", control_rate_hz=value)


if __name__ == "__main__":
    # CTest passes ROOT as argv[1]; keep unittest from treating it as a test name.
    unittest.main(argv=[sys.argv[0]])
