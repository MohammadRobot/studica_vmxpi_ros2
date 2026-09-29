"""Observer logic without ROS initialization, devices or network access."""

import ast
from collections import deque
from pathlib import Path
import threading
from types import SimpleNamespace

from studica_robot_platform.sensor_observer import SensorObserver
import studica_robot_platform.sensor_observer as module


ROOT = Path(__file__).resolve().parents[1]


def test_sensor_freshness_does_not_invent_safety_or_disabled_state(monkeypatch):
    observer = object.__new__(SensorObserver)
    observer._lock = threading.Lock()
    observer._frames = {name: deque(maxlen=60) for name in ("lidar", "camera")}
    observer._details = {"lidar": {}, "camera": {}}
    monkeypatch.setattr(module.time, "monotonic", lambda: 100.0)
    initial = observer.status()
    assert initial["read_only"] and initial["armed"] is None and not initial["ready"]
    assert initial["safety_state"] == "NOT_MONITORED"
    assert initial["sensors"]["camera"]["enabled"] is None
    observer._scan(SimpleNamespace(header=SimpleNamespace(frame_id="laser_scan_frame"), ranges=[1.0] * 270))
    observer._image(SimpleNamespace(header=SimpleNamespace(frame_id="camera_depth_optical_frame"),
                                    width=320, height=240, encoding="16UC1"))
    fresh = observer.status()
    assert fresh["sensors"]["lidar"]["healthy"]
    assert fresh["sensors"]["camera"]["width"] == 320
    monkeypatch.setattr(module.time, "monotonic", lambda: 103.0)
    stale = observer.status()
    assert not stale["sensors"]["camera"]["healthy"]
    assert stale["sensors"]["camera"]["enabled"] is None


def test_observer_has_no_command_publisher_or_service_client():
    tree = ast.parse((ROOT / "studica_robot_platform/sensor_observer.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            assert node.func.attr not in {"create_publisher", "create_client", "create_service"}
    wrapper = (ROOT / "deployment/training/studica_observer_web_runtime").read_text()
    assert "--observation-only" in wrapper
    assert "--pairing-root /run/studica-training-web/pairing" in wrapper
    assert "/home/vmx/studica_ws" not in wrapper
    unit = (ROOT / "deployment/training/studica-training-web.service").read_text()
    assert "User=studica" in unit and "PrivateDevices=yes" in unit
    assert "RuntimeDirectory=studica-training-web" in unit
    assert "RestrictAddressFamilies=AF_INET AF_INET6 AF_UNIX AF_NETLINK" in unit
    assert "Requires=studica-hardware" not in unit
    camera = (ROOT / "deployment/training/studica-training-camera.service").read_text()
    assert "[Install]" not in camera
    assert "DevicePolicy=closed" in camera and "MemoryMax=700M" in camera
