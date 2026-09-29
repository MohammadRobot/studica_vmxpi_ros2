"""Sensor command contracts with ROS setup and executables replaced by stubs."""

import ast
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def run_sensor(tmp_path, *arguments, setup_fails=False):
    # Only this stub is on PATH: tests cannot launch a real ROS executable.
    ros2 = tmp_path / "ros2"
    ros2.write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    ros2.chmod(0o700)
    return subprocess.run(
        [
            "/bin/bash", "-c",
            ('source() { return 31; };' if setup_fails else
             'source() { : "$SENSOR_TEST_UNSET_SETUP_VAR"; };')
            + ' export -f source; exec /bin/bash "$@"',
            "sensor-test", str(ROOT / "scripts/studica_sensor_runtime"),
            *arguments,
        ],
        env={"PATH": str(tmp_path), "STUDICA_RELEASE_ROOT": "/unused-test-release"},
        capture_output=True, text=True, check=False, timeout=5,
    )


def test_lidar_selects_x2_sensor_only(tmp_path):
    result = run_sensor(tmp_path, "lidar")
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "launch", "studica_vmxpi_ros2", "_lidar_hw.launch.py", "lidar_type:=x2",
    ]


def test_failed_ros_setup_still_aborts_without_launch(tmp_path):
    result = run_sensor(tmp_path, "lidar", setup_fails=True)
    assert result.returncode == 31
    assert result.stdout == ""


def test_both_production_wrappers_suspend_nounset_only_for_setup():
    for name in ("studica_sensor_runtime", "studica_hardware_runtime"):
        script = (ROOT / "scripts" / name).read_text()
        assert script.index("set +u") < script.index("source /opt/ros/humble/setup.bash")
        assert script.index('source "${release_root}/install/setup.bash"') < script.index("\nset -u")


def test_x2_matches_robot_profile_and_resolves_x2_parameters():
    profile = yaml.safe_load(
        (ROOT / "bringup/config/profiles/stack_4wd/robot_profile.yaml").read_text()
    )
    assert profile["hardware"]["lidar_type"] == "x2"
    # Inspect the private launch's preset table without importing ROS or drivers.
    tree = ast.parse((ROOT / "bringup/launch/_lidar_hw.launch.py").read_text())
    tables = [
        ast.literal_eval(node.value)
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "lidar_to_yaml"
                for target in node.targets)
    ]
    assert len(tables) == 1
    assert tables[0]["x2"] == "X2.yaml"


def test_camera_remains_low_resource_depth_only(tmp_path):
    result = run_sensor(tmp_path, "camera")
    assert result.returncode == 0, result.stderr
    assert result.stdout.splitlines() == [
        "launch", "studica_vmxpi_ros2", "_camera_hw.launch.py",
        "orbbec_enable_color:=false", "orbbec_enable_depth:=true",
        "orbbec_enable_ir:=false", "orbbec_enable_point_cloud:=false",
        "orbbec_depth_width:=320", "orbbec_depth_height:=240", "orbbec_depth_fps:=5",
    ]


@pytest.mark.parametrize("arguments", [(), ("motor",), ("unknown",)])
def test_invalid_sensor_does_not_launch_ros(tmp_path, arguments):
    result = run_sensor(tmp_path, *arguments)
    assert result.returncode == 64
    assert result.stdout == ""
    assert "usage:" in result.stderr


def test_training_lidar_unit_is_separate_and_serial_device_only():
    unit = (ROOT / "deployment/training/studica-training-lidar.service").read_text()
    assert "User=vmx" in unit
    assert "DevicePolicy=closed" in unit
    assert "After=network.target dev-ttyUSB0.device" in unit
    assert "BindsTo=dev-ttyUSB0.device" in unit
    assert "JobTimeoutSec=30" in unit
    assert "DeviceAllow=/dev/ttyUSB0 rw" in unit
    assert unit.count("DeviceAllow=") == 1
    assert "ROS_DOMAIN_ID=78" in unit
    assert "studica_sensor_runtime lidar" in unit
    assert "WantedBy=multi-user.target" in unit
    assert "Requires=studica-hardware" not in unit
    assert "PartOf=studica-robot.target" not in unit
    assert "studica_hardware_runtime" not in unit


def test_prepared_install_never_enables_or_starts_production():
    script = (ROOT / "deployment/training/install_prepared_setup.sh").read_text()
    assert "systemctl enable" not in script
    assert "systemctl start" not in script
    assert "[[ ! -e /opt/studica/current && ! -L /opt/studica/current ]]" in script
    assert "/etc/hostname" not in script
    assert "/etc/ssh/" not in script
    assert "/etc/NetworkManager/" not in script
