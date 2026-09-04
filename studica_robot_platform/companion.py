"""Ubuntu companion daemon for SLAM Toolbox, Nav2, and map synchronization."""

from __future__ import annotations

import argparse
from io import BytesIO
import json
import os
from pathlib import Path
import shutil
import signal
import ssl
import subprocess
import tempfile
import threading
import time
from typing import Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import zipfile

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from studica_vmxpi_ros2.srv import SaveMap

from . import API_VERSION
from .map_registry import MapRegistry, MapRegistryError
from .model import AUTONOMY_MODES, Mode, valid_map_id


class CompanionError(RuntimeError):
    """The companion cannot safely satisfy an autonomy request."""


def _private_token(path: Path) -> str:
    mode = path.stat().st_mode
    if mode & 0o077:
        raise CompanionError("companion token file permissions must be 0600")
    value = path.read_text(encoding="utf-8").strip()
    if len(value) < 24:
        raise CompanionError("companion token is invalid")
    return value


class RobotApi:
    """Authenticated companion transport with explicit TLS policy."""

    def __init__(
        self,
        base_url: str,
        token: str,
        ca_file: Optional[Path] = None,
        insecure: bool = False,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        if not self.base_url.startswith("https://") and not insecure:
            raise CompanionError("robot API must use HTTPS")
        self.token = token
        if insecure:
            self.context = ssl._create_unverified_context()  # test-only option
        else:
            self.context = ssl.create_default_context(
                cafile=str(ca_file) if ca_file else None
            )

    def _request(
        self,
        path: str,
        method: str = "GET",
        body: Optional[bytes] = None,
        content_type: Optional[str] = None,
        timeout: float = 15.0,
    ) -> bytes:
        headers = {"Authorization": f"Bearer {self.token}"}
        if content_type:
            headers["Content-Type"] = content_type
        request = Request(
            f"{self.base_url}{path}",
            data=body,
            method=method,
            headers=headers,
        )
        try:
            with urlopen(request, timeout=timeout, context=self.context) as response:
                return response.read(33 * 1024 * 1024)
        except HTTPError as error:
            detail = error.read(4096).decode("utf-8", errors="replace")
            raise CompanionError(
                f"robot API returned HTTP {error.code}: {detail}"
            ) from error
        except (OSError, URLError) as error:
            raise CompanionError(f"robot API is unavailable: {error}") from error

    def download_map(self, map_id: str) -> bytes:
        if not valid_map_id(map_id):
            raise CompanionError("invalid map_id")
        return self._request(f"/api/v1/maps/{map_id}/bundle")

    def status(self) -> dict:
        try:
            value = json.loads(self._request("/api/v1/status", timeout=2.0))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise CompanionError("robot status response is invalid") from error
        if not isinstance(value, dict):
            raise CompanionError("robot status response must be an object")
        return value

    def heartbeat(self, payload: dict) -> None:
        self._request(
            "/api/v1/companion/heartbeat",
            method="POST",
            body=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            content_type="application/json",
            timeout=2.0,
        )

    def upload_map(self, map_id: str, payload: bytes) -> None:
        if not valid_map_id(map_id):
            raise CompanionError("invalid map_id")
        from urllib.parse import quote

        self._request(
            f"/api/v1/maps?map_id={quote(map_id, safe='')}",
            method="POST",
            body=payload,
            content_type="application/zip",
            timeout=30.0,
        )


class AutonomyProcess:
    """Own exactly one child launch process and stop its complete process group."""

    def __init__(self, ros2: str, startup_grace: float) -> None:
        self.ros2 = ros2
        self.startup_grace = startup_grace
        self.process: Optional[subprocess.Popen] = None
        self.mode = Mode.IDLE
        self.started_at = 0.0

    def start(self, mode: Mode, map_yaml: Optional[Path] = None) -> None:
        self.stop()
        if mode == Mode.SLAM:
            arguments = [
                self.ros2,
                "launch",
                "studica_vmxpi_ros2",
                "mapping.launch.py",
                "mode:=hardware",
                "gui:=false",
                "use_joystick:=false",
            ]
        elif mode == Mode.NAVIGATION and map_yaml is not None:
            arguments = [
                self.ros2,
                "launch",
                "studica_vmxpi_ros2",
                "navigation.launch.py",
                "mode:=hardware",
                "gui:=false",
                "use_joystick:=false",
                f"map:={map_yaml}",
            ]
        else:
            raise CompanionError("unsupported autonomy process request")
        self.process = subprocess.Popen(arguments, start_new_session=True)
        self.mode = mode
        self.started_at = time.monotonic()

    def stop(self) -> None:
        process = self.process
        self.process = None
        self.mode = Mode.IDLE
        self.started_at = 0.0
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)

    def state(self) -> tuple[bool, str]:
        process = self.process
        if process is None:
            return False, ""
        return_code = process.poll()
        if return_code is not None:
            self.process = None
            return False, f"AUTONOMY_PROCESS_EXITED_{return_code}"
        return time.monotonic() - self.started_at >= self.startup_grace, ""


class CompanionNode(Node):
    """Reconcile requested platform state with PC-side autonomy processes."""

    def __init__(
        self,
        api: RobotApi,
        cache_root: Path,
        ros2: str,
        companion_id: str,
        startup_grace: float,
        startup_timeout: float,
    ) -> None:
        super().__init__("studica_companion")
        self._api = api
        self._registry = MapRegistry(cache_root)
        self._process = AutonomyProcess(ros2, startup_grace)
        self._startup_timeout = max(startup_timeout, startup_grace)
        self._companion_id = companion_id
        self._lock = threading.RLock()
        self._requested_mode = Mode.IDLE
        self._requested_map = ""
        self._active_mode = Mode.IDLE
        self._ready = True
        self._detail = ""
        self._last_sequence = -1
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError as error:
            raise CompanionError("nav2_msgs is required on the companion") from error
        self._navigate_action_type = NavigateToPose
        self._navigate_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self._navigation_goal_handle = None
        self.create_subscription(
            PoseStamped, "/robot/navigation/goal", self._on_navigation_goal, 10
        )
        self.create_service(SaveMap, "/robot/companion/save_map", self._save_map)
        self.create_timer(0.2, self._reconcile)
        self._transport_stop = threading.Event()
        self._transport_thread = threading.Thread(
            target=self._transport_loop,
            name="studica-companion-https",
            daemon=True,
        )
        self._transport_thread.start()

    def _accept_status(self, status: dict) -> None:
        try:
            sequence = int(status["sequence"])
            requested = Mode[str(status["requested_mode"])]
            transition = str(status["transition"])
            requested_map = str(status.get("requested_map_id", ""))
        except (KeyError, TypeError, ValueError) as error:
            raise CompanionError("robot status fields are invalid") from error
        if sequence == self._last_sequence:
            return
        self._last_sequence = sequence
        if transition == "WAITING_FOR_DISARM":
            requested = Mode.IDLE
        with self._lock:
            self._requested_mode = requested
            self._requested_map = requested_map

    def _heartbeat_payload(self, connected: bool = True) -> dict:
        with self._lock:
            return {
                "api_version": API_VERSION,
                "companion_id": self._companion_id,
                "connected": connected,
                "active_mode": int(self._active_mode),
                "ready": self._ready if connected else False,
                "detail": self._detail,
            }

    def _transport_loop(self) -> None:
        while not self._transport_stop.is_set():
            try:
                self._accept_status(self._api.status())
                self._api.heartbeat(self._heartbeat_payload())
            except CompanionError as error:
                self.get_logger().warn(
                    f"companion HTTPS transport unavailable: {error}",
                    throttle_duration_sec=5.0,
                )
            self._transport_stop.wait(0.5)

    def _prepare_map(self, map_id: str) -> Path:
        try:
            return self._registry.yaml_path(map_id)
        except MapRegistryError:
            payload = self._api.download_map(map_id)
            self._registry.import_bundle(map_id, payload)
            return self._registry.yaml_path(map_id)

    def _autonomy_dependency_ready(self, mode: Mode) -> tuple[bool, str]:
        """Confirm that the launched autonomy stack exposes its real API."""
        if mode == Mode.NAVIGATION:
            ready = self._navigate_client.server_is_ready()
            return ready, "NAVIGATION_ACTION_UNAVAILABLE"
        if mode == Mode.SLAM:
            services = {
                name for name, _ in self.get_service_names_and_types()
            }
            ready = "/slam_toolbox/save_map" in services
            return ready, "SLAM_SAVE_MAP_SERVICE_UNAVAILABLE"
        return False, "AUTONOMY_MODE_UNSUPPORTED"

    def _reconcile(self) -> None:
        with self._lock:
            requested = self._requested_mode
            map_id = self._requested_map
            if requested not in AUTONOMY_MODES:
                if self._process.mode != Mode.IDLE:
                    self._cancel_navigation_goal()
                    self._process.stop()
                self._active_mode = Mode.IDLE
                self._ready = True
                self._detail = ""
                return
            if self._process.mode != requested:
                self._cancel_navigation_goal()
                self._ready = False
                self._detail = ""
                try:
                    map_yaml = (
                        self._prepare_map(map_id)
                        if requested == Mode.NAVIGATION
                        else None
                    )
                    self._process.start(requested, map_yaml)
                    self._active_mode = requested
                except (CompanionError, MapRegistryError, OSError) as error:
                    self._process.stop()
                    self._active_mode = requested
                    self._detail = f"COMPANION_START_FAILED: {error}"
                    return
            ready, detail = self._process.state()
            if detail:
                self._ready = False
                self._detail = detail
                return
            if not ready:
                self._ready = False
                return
            dependency_ready, dependency_error = self._autonomy_dependency_ready(
                requested
            )
            self._ready = dependency_ready
            if dependency_ready:
                self._detail = ""
            elif (
                time.monotonic() - self._process.started_at
                >= self._startup_timeout
            ):
                self._detail = dependency_error

    def _on_navigation_goal(self, pose: PoseStamped) -> None:
        with self._lock:
            if self._active_mode != Mode.NAVIGATION or not self._ready:
                return
        if not self._navigate_client.wait_for_server(timeout_sec=0.5):
            with self._lock:
                self._detail = "NAVIGATION_ACTION_UNAVAILABLE"
            return
        self._cancel_navigation_goal()
        goal = self._navigate_action_type.Goal()
        goal.pose = pose
        future = self._navigate_client.send_goal_async(goal)
        future.add_done_callback(self._navigation_goal_response)

    def _navigation_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as error:
            with self._lock:
                self._detail = f"NAVIGATION_GOAL_FAILED: {error}"
            return
        if not handle.accepted:
            with self._lock:
                self._detail = "NAVIGATION_GOAL_REJECTED"
            return
        with self._lock:
            self._navigation_goal_handle = handle
            self._detail = ""

    def _cancel_navigation_goal(self) -> None:
        handle = self._navigation_goal_handle
        self._navigation_goal_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    @staticmethod
    def _map_bundle(yaml_path: Path, image_path: Path) -> bytes:
        output = BytesIO()
        with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.write(yaml_path, "map.yaml")
            archive.write(image_path, image_path.name)
        return output.getvalue()

    def _save_map(self, request: SaveMap.Request, response: SaveMap.Response):
        map_id = request.map_id.strip()
        if not valid_map_id(map_id):
            response.saved = False
            response.message = "invalid map_id"
            return response
        with self._lock:
            if self._active_mode != Mode.SLAM or not self._ready:
                response.saved = False
                response.message = "SLAM is not ready"
                return response
        try:
            with tempfile.TemporaryDirectory(prefix="studica-map-") as temporary:
                base = Path(temporary) / "map"
                completed = subprocess.run(
                    [
                        self._process.ros2,
                        "run",
                        "nav2_map_server",
                        "map_saver_cli",
                        "-f",
                        str(base),
                    ],
                    check=False,
                    timeout=30.0,
                    capture_output=True,
                    text=True,
                )
                if completed.returncode != 0:
                    detail = (completed.stderr or completed.stdout).strip()[-512:]
                    raise CompanionError(f"map saver failed: {detail}")
                yaml_path = base.with_suffix(".yaml")
                images = [
                    path
                    for suffix in (".pgm", ".png")
                    if (path := base.with_suffix(suffix)).is_file()
                ]
                if not yaml_path.is_file() or len(images) != 1:
                    raise CompanionError("map saver did not produce a valid map pair")
                payload = self._map_bundle(yaml_path, images[0])
                # Validate the generated output before sending it to the robot.
                validation_root = Path(temporary) / "validated"
                MapRegistry(validation_root).import_bundle(map_id, payload)
                self._api.upload_map(map_id, payload)
            response.saved = True
            response.message = f"map saved and uploaded: {map_id}"
        except (CompanionError, MapRegistryError, OSError, subprocess.TimeoutExpired) as error:
            response.saved = False
            response.message = str(error)
        return response

    def close(self) -> None:
        self._transport_stop.set()
        try:
            self._api.heartbeat(self._heartbeat_payload(connected=False))
        except CompanionError:
            pass
        self._transport_thread.join(timeout=3.0)
        with self._lock:
            self._cancel_navigation_goal()
            self._process.stop()


def main(args=None) -> None:
    """Run the companion process supervisor."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-url", default="https://robot.local")
    parser.add_argument(
        "--token-file", type=Path, default=Path.home() / ".config/studica/token"
    )
    parser.add_argument(
        "--ca-file", type=Path, default=Path.home() / ".config/studica/robot-ca.crt"
    )
    parser.add_argument(
        "--cache-root", type=Path, default=Path.home() / ".cache/studica/maps"
    )
    parser.add_argument("--companion-id", default=os.uname().nodename)
    parser.add_argument("--startup-grace-sec", type=float, default=3.0)
    parser.add_argument("--startup-timeout-sec", type=float, default=45.0)
    parser.add_argument("--insecure", action="store_true", help="local testing only")
    options, ros_args = parser.parse_known_args(args)
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise CompanionError("ros2 executable is not available")
    api = RobotApi(
        options.robot_url,
        _private_token(options.token_file),
        None if options.insecure else options.ca_file,
        options.insecure,
    )
    rclpy.init(args=ros_args)
    node = CompanionNode(
        api,
        options.cache_root.resolve(),
        ros2,
        options.companion_id,
        options.startup_grace_sec,
        options.startup_timeout_sec,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
