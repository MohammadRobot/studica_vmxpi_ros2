"""Classroom launcher contracts; dry runs never start ROS or touch hardware."""

import os
from pathlib import Path
import shlex
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
LAUNCHER = ROOT / "scripts" / "training.sh"


def run(*arguments, workspace=None):
    env = dict(os.environ)
    env.update(ROS_DOMAIN_ID="0", CYCLONEDDS_URI="file:///hardware-peers.xml")
    if workspace is not None:
        env["STUDICA_WS"] = str(workspace)
    return subprocess.run(
        ["bash", str(LAUNCHER), *arguments], capture_output=True, text=True,
        env=env, check=False,
    )


@pytest.mark.parametrize("action,launch", [
    ("sim", "sim.launch.py"), ("slam", "mapping.launch.py"),
    ("nav", "navigation.launch.py"),
])
def test_launch_is_simulation_only_and_does_not_arm(action, launch, tmp_path):
    result = run(action, "--headless", "--dry-run", workspace=tmp_path / "class workspace")
    assert result.returncode == 0, result.stderr
    command = shlex.split(result.stdout.splitlines()[-1])
    assert command[:4] == ["ros2", "launch", "studica_vmxpi_ros2", launch]
    assert "gui:=false" in command and "gz_headless:=true" in command
    assert "use_joystick:=false" in command
    assert not any("hardware" in item or "/robot/arm" in item for item in command)
    if action != "sim":
        assert "mode:=gz_sim" in command
    assert "ROS_DOMAIN_ID=77" in result.stdout
    assert "rmw_cyclonedds_cpp" in result.stdout
    assert "config/network/cyclonedds_sim.xml" in result.stdout
    assert "hardware-peers" not in result.stdout
    assert not (tmp_path / "class workspace").exists()


@pytest.mark.parametrize("arguments", [
    ("hardware",), ("robot",), ("sim", "mode:=hardware"),
    ("slam", "use_hardware:=true"), ("nav", "../office_nav"),
    ("nav", "/tmp/map.yaml"), ("nav", "one", "two"),
    ("save-map",), ("save-map", ".."), ("save-map", "bad name"),
    ("arm", "--headless"), ("nav", "--unknown"),
])
def test_rejects_hardware_overrides_and_unsafe_map_names(arguments):
    result = run(*arguments, "--dry-run")
    assert result.returncode == 2


def test_map_save_and_load_match_and_do_not_overwrite_in_dry_run(tmp_path):
    workspace = tmp_path / "student workspace"
    saved = run("save-map", "lesson_7", "--dry-run", workspace=workspace)
    loaded = run("nav", "lesson_7", "--dry-run", workspace=workspace)
    assert saved.returncode == loaded.returncode == 0
    save_command = shlex.split(saved.stdout.splitlines()[-1])
    nav_command = shlex.split(loaded.stdout.splitlines()[-1])
    destination = workspace / "project_maps/training/lesson_7/map"
    assert str(destination) in save_command
    assert f"map:={destination}.yaml" in nav_command
    assert not workspace.exists()


def test_explicit_arm_and_disarm_have_bounded_service_waits():
    for action in ("arm", "disarm"):
        result = run(action, "--dry-run")
        assert result.returncode == 0
        command = shlex.split(result.stdout.splitlines()[-1])
        assert command == [
            "timeout", "15", "ros2", "service", "call", f"/robot/{action}",
            "std_srvs/srv/Trigger", "{}",
        ]


def test_default_navigation_is_camera_off_and_keyboard_has_low_defaults():
    nav = shlex.split(run("nav", "--dry-run").stdout.splitlines()[-1])
    assert "use_point_cloud:=false" in nav
    assert not any(item.startswith("map:=") for item in nav)
    teleop = shlex.split(run("teleop", "--dry-run").stdout.splitlines()[-1])
    assert "speed:=0.10" in teleop and "turn:=0.25" in teleop


def test_loopback_profile_has_no_external_peers():
    import xml.etree.ElementTree as ET

    tree = ET.parse(ROOT / "bringup/config/network/cyclonedds_sim.xml")
    namespace = {"c": "https://cdds.io/config"}
    interfaces = tree.findall(".//c:NetworkInterface", namespace)
    peers = tree.findall(".//c:Peer", namespace)
    assert [item.attrib["name"] for item in interfaces] == ["lo"]
    assert [item.attrib["Address"] for item in peers] == ["127.0.0.1"]
    assert tree.find(".//c:AllowMulticast", namespace).text == "false"
