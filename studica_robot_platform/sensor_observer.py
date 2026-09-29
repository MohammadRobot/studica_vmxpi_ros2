"""Read-only standard ROS sensor subscriptions; no robot command interfaces."""

from collections import deque
import os
from pathlib import Path
import shutil
import threading
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan

from . import API_VERSION


class SensorObserver(Node):
    """Observes live sensors without claiming hardware safety or motor status."""

    def __init__(self):
        super().__init__("studica_sensor_observer", enable_rosout=False,
                         start_parameter_services=False)
        self._lock = threading.Lock()
        self._frames = {name: deque(maxlen=60) for name in ("lidar", "camera")}
        self._details = {"lidar": {}, "camera": {}}
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self._image,
                                 qos_profile_sensor_data)

    def _scan(self, message):
        with self._lock:
            self._frames["lidar"].append(time.monotonic())
            self._details["lidar"] = {
                "frame_id": message.header.frame_id, "points": len(message.ranges),
            }

    def _image(self, message):
        with self._lock:
            self._frames["camera"].append(time.monotonic())
            self._details["camera"] = {
                "frame_id": message.header.frame_id, "width": message.width,
                "height": message.height, "encoding": message.encoding,
            }

    def status(self):
        now = time.monotonic()
        with self._lock:
            sensors = {}
            for name, arrivals in self._frames.items():
                age = now - arrivals[-1] if arrivals else None
                fresh = age is not None and age < 2.0
                recent = [value for value in arrivals if now - value < 5.0]
                rate = ((len(recent) - 1) / (recent[-1] - recent[0])
                        if len(recent) > 1 and recent[-1] > recent[0] else 0.0)
                sensors[name] = {
                    # Missing messages do not distinguish disabled from failed.
                    "enabled": True if fresh else None, "healthy": fresh,
                    "last_message_age_sec": round(age, 3) if age is not None else None,
                    "rate_hz": round(rate, 2), **self._details[name],
                }
        memory = {}
        for line in Path("/proc/meminfo").read_text().splitlines():
            key, value = line.split(":", 1)
            memory[key] = int(value.split()[0])
        disk = shutil.disk_usage("/")
        return {
            "api_version": API_VERSION, "read_only": True,
            "mode": "OBSERVATION_ONLY", "requested_mode": "OBSERVATION_ONLY",
            "transition": "CONTROL_UNAVAILABLE", "ready": False, "armed": None,
            "safety_state": "NOT_MONITORED",
            "safety_reason": "Motor runtime is not enabled; physical safety inputs are not monitored here.",
            "active_control_source": "", "last_error": "Driving, SLAM and navigation are disabled.",
            "companion_connected": False, "developer_mode": False,
            "sensors": sensors,
            "diagnostics": {
                "summary": "Sensor observation only; not a safety monitor",
                "safety_inputs": {}, "motors": [], "battery_voltage": None,
                "compute": {
                    "cpu_load_1m_percent": 100 * os.getloadavg()[0] / (os.cpu_count() or 1),
                    "memory_used_percent": 100 * (1 - memory["MemAvailable"] / memory["MemTotal"]),
                    "disk_used_percent": 100 * disk.used / disk.total,
                },
            },
        }
