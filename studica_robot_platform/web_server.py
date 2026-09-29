"""Authenticated HTTPS/REST/WebSocket gateway for the local robot UI."""

from __future__ import annotations

import argparse
import asyncio
from collections import defaultdict, deque
import copy
from dataclasses import dataclass
import hmac
import json
import math
from pathlib import Path
import re
import secrets
import ssl
import threading
import time
from typing import Any, Deque, Dict, Optional, TYPE_CHECKING

from aiohttp import web
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

if TYPE_CHECKING:
    from studica_vmxpi_ros2.msg import PlatformStatus
    from studica_robot_monitor.msg import MotorTelemetryArray

from . import API_VERSION
from .map_registry import MapRegistry, MapRegistryError
from .model import Mode
from .orchestrator_client import OrchestratorError, request as orchestrator_request
from .pairing import COMPANION_ID, CompanionPairingStore, PairingError


MAX_JSON_BYTES = 16 * 1024
MAX_TELEOP_LINEAR = 0.30
MAX_TELEOP_ANGULAR = 0.90
SESSION_LIFETIME_SEC = 8 * 60 * 60
MAX_BROWSER_SESSIONS = 64

# ``web.AppKey`` was added after the aiohttp release shipped by Ubuntu 22.04.
# Keep typed keys on newer developer systems while using collision-resistant
# string keys on the Humble production base image.
if hasattr(web, "AppKey"):
    ALLOW_INSECURE_HTTP = web.AppKey("allow_insecure_http", bool)
    OPERATOR_STATE = web.AppKey("operator_state", dict)
else:
    ALLOW_INSECURE_HTTP = "studica.allow_insecure_http"
    OPERATOR_STATE = "studica.operator_state"


def status_to_dict(message: PlatformStatus) -> Dict[str, Any]:
    """Convert a platform status message into the stable REST representation."""
    return {
        "api_version": message.api_version,
        "sequence": message.sequence,
        "mode": message.mode_name,
        "requested_mode": message.requested_mode_name,
        "transition": message.transition,
        "safety_state": message.safety_state,
        "safety_reason": message.safety_reason,
        "ready": message.ready,
        "armed": message.armed,
        "sensors": {
            "lidar": {
                "enabled": message.lidar_enabled,
                "healthy": message.lidar_healthy,
            },
            "camera": {
                "enabled": message.camera_enabled,
                "healthy": message.camera_healthy,
            },
        },
        "companion_connected": message.companion_connected,
        "developer_mode": message.developer_mode,
        "active_control_source": message.active_control_source,
        "requested_map_id": message.requested_map_id,
        "last_error": message.last_error,
    }


def clamp(value: Any, lower: float, upper: float) -> float:
    """Parse and clamp a finite numeric API field."""
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise web.HTTPBadRequest(text="teleop values must be numeric") from error
    if number != number or number in {float("inf"), float("-inf")}:
        raise web.HTTPBadRequest(text="teleop values must be finite")
    return max(lower, min(upper, number))


class RosBridge(Node):
    """Bridge API requests to versioned ROS services and private command input."""

    def __init__(self) -> None:
        # The observation-only service needs standard sensor messages, not the
        # production control interfaces or any of their publishers/clients.
        from studica_vmxpi_ros2.msg import CompanionHeartbeat, PlatformStatus
        from studica_vmxpi_ros2.srv import SaveMap, SetDeveloperMode, SetMode, SetSensor
        from studica_robot_monitor.msg import MotorTelemetryArray

        super().__init__("studica_web_bridge")
        self._lock = threading.Lock()
        self._status: Optional[Dict[str, Any]] = None
        self._status_received_at = 0.0
        self._diagnostics: Dict[str, Any] = {
            "summary": "waiting for diagnostics",
            "level": 3,
            "items": [],
            "compute": {},
            "motors": [],
            "battery_voltage": None,
            "safety_inputs": {},
        }
        self.create_subscription(
            PlatformStatus, "/robot/platform/status", self._on_status, 10
        )
        self.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        self.create_subscription(
            MotorTelemetryArray, "/robot_status/motors", self._on_motors, 10
        )
        self.teleop_publisher = self.create_publisher(
            Twist, "/robot/control/web", 1
        )
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped, "/robot/navigation/goal", 10
        )
        self.companion_heartbeat_publisher = self.create_publisher(
            CompanionHeartbeat, "/robot/companion/heartbeat", 10
        )
        self.mode_client = self.create_client(SetMode, "/robot/platform/set_mode")
        self.sensor_client = self.create_client(
            SetSensor, "/robot/platform/set_sensor"
        )
        self.developer_client = self.create_client(
            SetDeveloperMode, "/robot/platform/set_developer_mode"
        )
        self.save_map_client = self.create_client(
            SaveMap, "/robot/companion/save_map"
        )

    def _on_status(self, message: PlatformStatus) -> None:
        with self._lock:
            self._status = status_to_dict(message)
            self._status_received_at = time.monotonic()

    def status(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if (
                self._status is None
                or time.monotonic() - self._status_received_at > 2.0
            ):
                return None
            result = copy.deepcopy(self._status)
            result["diagnostics"] = copy.deepcopy(self._diagnostics)
            return result

    @staticmethod
    def _diagnostic_level(value: Any) -> int:
        if isinstance(value, bytes):
            return value[0] if value else 3
        return int(value)

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        items = []
        compute: Dict[str, Any] = {}
        safety_inputs: Optional[Dict[str, str]] = None
        highest = 0
        for status in message.status:
            level = self._diagnostic_level(status.level)
            highest = max(highest, level)
            values = {item.key: item.value for item in status.values}
            items.append(
                {"name": status.name, "level": level, "message": status.message}
            )
            if status.name == "Robot/Compute/Pi":
                for key in (
                    "cpu_load_1m_percent",
                    "memory_used_percent",
                    "disk_used_percent",
                    "cpu_temperature_c",
                ):
                    try:
                        compute[key] = float(values[key])
                    except (KeyError, TypeError, ValueError):
                        compute[key] = None
            if status.name == "Robot/Control/HardwareSafety":
                safety_inputs = values
        labels = {0: "healthy", 1: "warning", 2: "error", 3: "stale"}
        with self._lock:
            self._diagnostics["level"] = highest
            self._diagnostics["summary"] = labels.get(highest, "unknown")
            self._diagnostics["items"] = items
            self._diagnostics["compute"] = compute
            if safety_inputs is not None:
                self._diagnostics["safety_inputs"] = safety_inputs

    def _on_motors(self, message: MotorTelemetryArray) -> None:
        motors = []
        for motor in message.motors:
            motors.append(
                {
                    "joint": motor.joint_name,
                    "channel": motor.motor_channel,
                    "target_rad_s": motor.target_velocity_rad_s,
                    "measured_rad_s": motor.measured_velocity_rad_s,
                    "encoder_fresh": motor.encoder_fresh,
                    "level": motor.health_level,
                    "message": motor.health_message,
                }
            )
        with self._lock:
            self._diagnostics["motors"] = motors

    async def call(self, client, request, timeout: float = 3.0):
        """Await a ROS future while its executor spins in another thread."""
        if not client.service_is_ready():
            raise web.HTTPServiceUnavailable(text="robot service is unavailable")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            await asyncio.sleep(0.01)
        if not future.done():
            raise web.HTTPGatewayTimeout(text="robot service timed out")
        error = future.exception()
        if error is not None:
            raise web.HTTPBadGateway(text=f"robot service failed: {error}")
        return future.result()

    def publish_teleop(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.teleop_publisher.publish(message)

    def publish_navigation_goal(self, x: float, y: float, yaw: float) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self.navigation_goal_publisher.publish(message)

    def publish_companion_heartbeat(
        self,
        companion_id: str,
        connected: bool,
        active_mode: Mode,
        ready: bool,
        detail: str,
    ) -> None:
        """Relay an authenticated HTTPS heartbeat onto the local ROS graph."""
        from studica_vmxpi_ros2.msg import CompanionHeartbeat

        message = CompanionHeartbeat()
        message.stamp = self.get_clock().now().to_msg()
        message.api_version = API_VERSION
        message.companion_id = companion_id
        message.connected = connected
        message.active_mode = int(active_mode)
        message.ready = ready
        message.detail = detail
        self.companion_heartbeat_publisher.publish(message)


@dataclass
class Session:
    """Short-lived browser session created from the device pairing token."""

    csrf: str
    expires_at: float


class AuthStore:
    """Constant-time bearer validation and bounded browser sessions."""

    def __init__(
        self, token: str, companion_pairing: CompanionPairingStore
    ) -> None:
        if len(token) < 24:
            raise ValueError("API token must contain at least 24 characters")
        self._token = token
        self._companion_pairing = companion_pairing
        self._sessions: Dict[str, Session] = {}
        self._failures: Dict[str, Deque[float]] = defaultdict(deque)

    def check_token(self, token: str) -> bool:
        return hmac.compare_digest(token, self._token)

    def bearer_scope(self, token: str) -> str:
        """Return the narrow authorization scope for a bearer credential."""
        if self.check_token(token):
            return "admin"
        if self._companion_pairing.validate_token(token):
            return "companion"
        return ""

    def rate_limited(self, peer: str) -> bool:
        now = time.monotonic()
        failures = self._failures[peer]
        while failures and now - failures[0] > 60.0:
            failures.popleft()
        return len(failures) >= 5

    def failure(self, peer: str) -> None:
        self._failures[peer].append(time.monotonic())

    def create_session(self) -> tuple[str, Session]:
        now = time.monotonic()
        expired = [
            session_id
            for session_id, session in self._sessions.items()
            if now > session.expires_at
        ]
        for session_id in expired:
            self._sessions.pop(session_id, None)
        while len(self._sessions) >= MAX_BROWSER_SESSIONS:
            self._sessions.pop(next(iter(self._sessions)))
        session_id = secrets.token_urlsafe(32)
        session = Session(
            csrf=secrets.token_urlsafe(24),
            expires_at=now + SESSION_LIFETIME_SEC,
        )
        self._sessions[session_id] = session
        return session_id, session

    def session(self, session_id: str) -> Optional[Session]:
        session = self._sessions.get(session_id)
        if session is None:
            return None
        if time.monotonic() > session.expires_at:
            self._sessions.pop(session_id, None)
            return None
        return session


def openapi_document() -> Dict[str, Any]:
    """Return the checked-in API v1 contract served to developers."""
    def operation(summary: str, status: str = "200", public: bool = False):
        value = {
            "summary": summary,
            "responses": {status: {"description": "Request completed"}},
        }
        if public:
            value["security"] = []
        return value

    return {
        "openapi": "3.0.3",
        "info": {"title": "Studica Robot API", "version": API_VERSION},
        "servers": [{"url": "https://robot.local"}],
        "security": [{"bearerAuth": []}, {"browserSession": []}],
        "paths": {
            "/api/v1/health": {
                "get": operation("Process and ROS readiness", public=True)
            },
            "/api/v1/session": {
                "post": operation("Create a browser session", public=True)
            },
            "/api/v1/companion/pair": {
                "post": operation(
                    "Redeem a one-time code for a scoped companion token",
                    public=True,
                )
            },
            "/api/v1/companion/pairing-code": {
                "post": operation(
                    "Issue a one-time companion code while safely disarmed",
                    "201",
                )
            },
            "/api/v1/companion/heartbeat": {
                "post": operation(
                    "Report paired companion presence and autonomy readiness"
                )
            },
            "/api/v1/openapi.json": {
                "get": operation("This OpenAPI contract", public=True)
            },
            "/api/v1/status": {"get": operation("Current robot state")},
            "/api/v1/mode": {"put": operation("Request a safe mode change")},
            "/api/v1/sensors/{sensor}": {
                "put": operation("Enable or disable lidar/camera"),
                "parameters": [
                    {
                        "name": "sensor",
                        "in": "path",
                        "required": True,
                        "schema": {"type": "string", "enum": ["lidar", "camera"]},
                    }
                ],
            },
            "/api/v1/developer-mode": {
                "put": operation("Enable restricted native DDS developer ingress")
            },
            "/api/v1/maps": {
                "get": operation("List maps"),
                "post": operation("Import a validated map ZIP", "201"),
            },
            "/api/v1/maps/{map_id}/bundle": {
                "get": operation("Download a validated portable map bundle"),
                "parameters": [
                    {
                        "name": "map_id",
                        "in": "path",
                        "required": True,
                        "schema": {"type": "string", "maxLength": 64},
                    }
                ],
            },
            "/api/v1/maps/save": {
                "post": operation("Save the active SLAM map")
            },
            "/api/v1/navigation/goal": {
                "post": operation("Send a map-frame Nav2 pose goal")
            },
            "/api/v1/teleop": {
                "get": operation("Authenticated hold-to-drive WebSocket", "101")
            },
            "/api/v1/telemetry": {
                "get": operation("Authenticated status WebSocket", "101")
            },
            "/api/v1/bluetooth/devices": {
                "get": operation("List or scan for Bluetooth controllers")
            },
            "/api/v1/bluetooth/pair": {
                "post": operation("Pair, trust, and connect a controller")
            },
            "/api/v1/network/wifi": {
                "get": operation("Scan Wi-Fi networks"),
                "post": operation("Provision infrastructure Wi-Fi"),
            },
            "/api/v1/updates": {
                "get": operation("List signed staged updates")
            },
            "/api/v1/updates/activate": {
                "post": operation("Approve a disarmed update activation", "202")
            },
            "/api/v1/support-bundle": {
                "post": operation("Generate a redacted support archive")
            },
        },
        "components": {
            "securitySchemes": {
                "bearerAuth": {"type": "http", "scheme": "bearer"},
                "browserSession": {
                    "type": "apiKey",
                    "in": "cookie",
                    "name": "studica_session",
                },
            }
        },
    }


async def json_body(request: web.Request) -> Dict[str, Any]:
    if request.content_length is not None and request.content_length > MAX_JSON_BYTES:
        raise web.HTTPRequestEntityTooLarge(
            max_size=MAX_JSON_BYTES, actual_size=request.content_length
        )
    try:
        payload = await request.json()
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise web.HTTPBadRequest(text="invalid JSON") from error
    if not isinstance(payload, dict):
        raise web.HTTPBadRequest(text="JSON body must be an object")
    return payload


def create_app(
    bridge: RosBridge,
    auth: AuthStore,
    registry: MapRegistry,
    companion_pairing: CompanionPairingStore,
    web_root: Path,
    *,
    observation_only: bool = False,
) -> web.Application:
    """Create the complete local API/UI application."""

    @web.middleware
    async def security_headers(request: web.Request, handler):
        response = await handler(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "DENY"
        response.headers["Referrer-Policy"] = "no-referrer"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self' wss: ws:; img-src 'self' data:"
        )
        return response

    @web.middleware
    async def authentication(request: web.Request, handler):
        if not request.path.startswith("/api/v1") or request.path in {
            "/api/v1/health",
            "/api/v1/session",
            "/api/v1/openapi.json",
            "/api/v1/companion/pair",
        }:
            return await handler(request)
        session_id = request.cookies.get("studica_session", "")
        session = auth.session(session_id)
        bearer = request.headers.get("Authorization", "")
        bearer_scope = (
            auth.bearer_scope(bearer[7:])
            if bearer.startswith("Bearer ")
            else ""
        )
        bearer_ok = bool(bearer_scope)
        if session is None and not bearer_ok:
            raise web.HTTPUnauthorized(text="authentication required")
        if bearer_scope == "companion":
            companion_allowed = (
                request.method == "GET"
                and (
                    request.path == "/api/v1/status"
                    or (
                        request.path.startswith("/api/v1/maps/")
                        and request.path.endswith("/bundle")
                    )
                )
            ) or (
                request.method == "POST"
                and request.path
                in {"/api/v1/maps", "/api/v1/companion/heartbeat"}
            )
            if not companion_allowed:
                raise web.HTTPForbidden(text="companion token scope rejected")
        if request.method not in {"GET", "HEAD"} and not bearer_ok:
            if not hmac.compare_digest(
                request.headers.get("X-CSRF-Token", ""), session.csrf
            ):
                raise web.HTTPForbidden(text="CSRF token required")
        request["session"] = session
        request["bearer_ok"] = bearer_ok
        return await handler(request)

    @web.middleware
    async def observation_guard(request: web.Request, handler):
        if observation_only and request.path.startswith("/api/v1"):
            allowed = (
                request.method == "GET" and request.path in {
                    "/api/v1/health", "/api/v1/openapi.json",
                    "/api/v1/status", "/api/v1/telemetry", "/api/v1/maps",
                }
            ) or (
                request.method == "POST" and request.path == "/api/v1/session"
            ) or (
                request.method == "GET" and re.fullmatch(
                    r"/api/v1/maps/[A-Za-z0-9][A-Za-z0-9._-]{0,63}/bundle",
                    request.path,
                ) is not None
            )
            if not allowed:
                raise web.HTTPForbidden(text="observation-only deployment: control is disabled")
        return await handler(request)

    app = web.Application(
        middlewares=[security_headers, authentication, observation_guard],
        client_max_size=33 * 1024**2,
    )
    app[OPERATOR_STATE] = {"teleop_active": False}

    def websocket_protocol(request: web.Request) -> Optional[str]:
        """Require browser WebSockets to prove the same CSRF session secret."""
        if request["bearer_ok"]:
            return None
        session = request["session"]
        expected = f"studica-v1.{session.csrf}"
        offered = {
            value.strip()
            for value in request.headers.get("Sec-WebSocket-Protocol", "").split(",")
        }
        if not any(hmac.compare_digest(value, expected) for value in offered):
            raise web.HTTPForbidden(text="WebSocket CSRF protocol required")
        return expected

    async def health(request):
        del request
        return web.json_response(
            {"ok": True, "api_version": API_VERSION,
             "ros_ready": not observation_only and bridge.status() is not None,
             "read_only": observation_only}
        )

    async def session_login(request):
        peer = request.remote or "unknown"
        if auth.rate_limited(peer):
            raise web.HTTPTooManyRequests(text="too many authentication attempts")
        payload = await json_body(request)
        if not auth.check_token(str(payload.get("token", ""))):
            auth.failure(peer)
            raise web.HTTPUnauthorized(text="invalid pairing token")
        session_id, session = auth.create_session()
        response = web.json_response({"ok": True, "csrf": session.csrf})
        response.set_cookie(
            "studica_session",
            session_id,
            secure=not request.app[ALLOW_INSECURE_HTTP],
            httponly=True,
            samesite="Strict",
            max_age=SESSION_LIFETIME_SEC,
        )
        return response

    async def openapi(request):
        del request
        document = openapi_document()
        if observation_only:
            document["info"]["description"] = (
                "Observation-only deployment. No motion, sensor control or maintenance writes."
            )
            allowed_paths = {
                "/api/v1/health", "/api/v1/session", "/api/v1/openapi.json",
                "/api/v1/status", "/api/v1/telemetry", "/api/v1/maps",
                "/api/v1/maps/{map_id}/bundle",
            }
            document["paths"] = {
                path: {method: value for method, value in methods.items()
                       if method == "get" or path == "/api/v1/session"}
                for path, methods in document["paths"].items() if path in allowed_paths
            }
        return web.json_response(document)

    async def companion_pair(request):
        peer = request.remote or "unknown"
        if auth.rate_limited(peer):
            raise web.HTTPTooManyRequests(text="too many authentication attempts")
        current = bridge.status() or {}
        if (
            current.get("mode") != "IDLE"
            or current.get("requested_mode") != "IDLE"
            or current.get("transition") != "READY"
            or current.get("safety_state") != "READY_DISARMED"
            or current.get("armed")
        ):
            raise web.HTTPConflict(
                text="companion pairing requires ready, disarmed IDLE mode"
            )
        payload = await json_body(request)
        token = None
        configured = False
        try:
            code = str(payload.get("code", ""))
            companion_id = str(payload.get("companion_id", ""))
            # Consume before awaiting privileged peer configuration.  This
            # prevents concurrent redeemers from racing the allowlisted peer.
            token = companion_pairing.redeem(code, companion_id, peer)
            transport = await orchestrate(
                {
                    "action": "configure_companion_peer",
                    "address": peer,
                }
            )
            configured = True
        except PairingError as error:
            auth.failure(peer)
            raise web.HTTPUnauthorized(
                text="invalid or expired companion pairing code"
            ) from error
        finally:
            if token is not None and not configured:
                companion_pairing.revoke_token(token)
        return web.json_response(
            {
                "token": token,
                "scope": "companion",
                "api_version": API_VERSION,
                "peer_address": peer,
                "platform_restart_scheduled": bool(
                    transport.get("restart_scheduled")
                ),
            }
        )

    async def issue_companion_pairing(request):
        del request
        current = bridge.status() or {}
        if (
            current.get("mode") != "IDLE"
            or current.get("requested_mode") != "IDLE"
            or current.get("transition") != "READY"
            or current.get("safety_state") != "READY_DISARMED"
            or current.get("armed")
        ):
            raise web.HTTPConflict(
                text="companion pairing requires IDLE and READY_DISARMED"
            )
        return web.json_response(companion_pairing.issue_code(), status=201)

    async def companion_heartbeat(request):
        payload = await json_body(request)
        if payload.get("api_version") != API_VERSION:
            raise web.HTTPConflict(text="companion API version is incompatible")
        try:
            active_mode = Mode(int(payload.get("active_mode")))
        except (TypeError, ValueError) as error:
            raise web.HTTPBadRequest(text="active_mode is invalid") from error
        companion_id = str(payload.get("companion_id", ""))
        detail = str(payload.get("detail", ""))
        if COMPANION_ID.fullmatch(companion_id) is None:
            raise web.HTTPBadRequest(text="companion_id is invalid")
        if len(detail) > 256:
            raise web.HTTPBadRequest(text="heartbeat detail is too long")
        if not isinstance(payload.get("connected"), bool) or not isinstance(
            payload.get("ready"), bool
        ):
            raise web.HTTPBadRequest(text="heartbeat booleans are invalid")
        bridge.publish_companion_heartbeat(
            companion_id,
            payload["connected"],
            active_mode,
            payload["ready"],
            detail,
        )
        return web.json_response({"accepted": True})

    async def status(request):
        del request
        value = bridge.status()
        if value is None:
            raise web.HTTPServiceUnavailable(text="waiting for platform status")
        return web.json_response(value)

    async def set_mode(request):
        from studica_vmxpi_ros2.srv import SetMode

        payload = await json_body(request)
        try:
            mode = Mode[str(payload.get("mode", "")).upper()]
        except KeyError as error:
            raise web.HTTPBadRequest(text="unsupported mode") from error
        ros_request = SetMode.Request()
        ros_request.mode = int(mode)
        ros_request.map_id = str(payload.get("map_id", ""))
        result = await bridge.call(bridge.mode_client, ros_request)
        code = 200 if result.accepted else 409
        return web.json_response(
            {"accepted": result.accepted, "message": result.message}, status=code
        )

    async def set_sensor(request):
        from studica_vmxpi_ros2.srv import SetSensor

        sensor = request.match_info["sensor"].lower()
        payload = await json_body(request)
        if not isinstance(payload.get("enabled"), bool):
            raise web.HTTPBadRequest(text="enabled must be boolean")
        ros_request = SetSensor.Request()
        ros_request.sensor = sensor
        ros_request.enabled = payload["enabled"]
        result = await bridge.call(bridge.sensor_client, ros_request)
        code = 200 if result.accepted else 409
        return web.json_response(
            {"accepted": result.accepted, "message": result.message}, status=code
        )

    async def set_developer(request):
        from studica_vmxpi_ros2.srv import SetDeveloperMode

        payload = await json_body(request)
        if not isinstance(payload.get("enabled"), bool):
            raise web.HTTPBadRequest(text="enabled must be boolean")
        ros_request = SetDeveloperMode.Request()
        ros_request.enabled = payload["enabled"]
        result = await bridge.call(bridge.developer_client, ros_request)
        code = 200 if result.accepted else 409
        return web.json_response(
            {"accepted": result.accepted, "message": result.message}, status=code
        )

    async def list_maps(request):
        del request
        return web.json_response({"maps": registry.list_maps()})

    async def import_map(request):
        map_id = request.query.get("map_id", "")
        payload = await request.read()
        try:
            metadata = registry.import_bundle(map_id, payload)
        except MapRegistryError as error:
            raise web.HTTPBadRequest(text=str(error)) from error
        return web.json_response(metadata, status=201)

    async def export_map(request):
        try:
            payload = registry.export_bundle(request.match_info["map_id"])
        except MapRegistryError as error:
            raise web.HTTPNotFound(text=str(error)) from error
        return web.Response(body=payload, content_type="application/zip")

    async def save_map(request):
        from studica_vmxpi_ros2.srv import SaveMap

        payload = await json_body(request)
        ros_request = SaveMap.Request()
        ros_request.map_id = str(payload.get("map_id", ""))
        result = await bridge.call(bridge.save_map_client, ros_request, timeout=45.0)
        code = 200 if result.saved else 409
        return web.json_response(
            {"saved": result.saved, "message": result.message}, status=code
        )

    async def navigation_goal(request):
        payload = await json_body(request)
        current = bridge.status() or {}
        if current.get("mode") != "NAVIGATION" or not current.get("ready"):
            raise web.HTTPConflict(text="navigation mode is not ready")
        values = []
        for field in ("x", "y", "yaw"):
            try:
                value = float(payload[field])
            except (KeyError, TypeError, ValueError) as error:
                raise web.HTTPBadRequest(text=f"{field} must be numeric") from error
            if not math.isfinite(value):
                raise web.HTTPBadRequest(text=f"{field} must be finite")
            values.append(value)
        if (
            abs(values[0]) > 1000.0
            or abs(values[1]) > 1000.0
            or abs(values[2]) > 100.0
        ):
            raise web.HTTPBadRequest(text="navigation goal is outside accepted bounds")
        bridge.publish_navigation_goal(*values)
        return web.json_response(
            {"accepted": True, "message": "navigation goal sent"}
        )

    async def orchestrate(payload: Dict[str, Any]) -> Dict[str, Any]:
        try:
            result = await asyncio.to_thread(orchestrator_request, payload)
        except OrchestratorError as error:
            raise web.HTTPServiceUnavailable(text=str(error)) from error
        if not result.get("ok"):
            raise web.HTTPConflict(text=str(result.get("error", "request rejected")))
        return result

    async def bluetooth_devices(request):
        scan = request.query.get("scan", "false").lower() == "true"
        return web.json_response(
            await orchestrate({"action": "bluetooth_devices", "scan": scan})
        )

    async def bluetooth_pair(request):
        payload = await json_body(request)
        return web.json_response(
            await orchestrate(
                {"action": "bluetooth_pair", "address": payload.get("address", "")}
            )
        )

    async def wifi_networks(request):
        del request
        return web.json_response(await orchestrate({"action": "wifi_networks"}))

    async def wifi_connect(request):
        payload = await json_body(request)
        return web.json_response(
            await orchestrate(
                {
                    "action": "wifi_connect",
                    "ssid": payload.get("ssid", ""),
                    "password": payload.get("password", ""),
                }
            )
        )

    async def update_status(request):
        del request
        return web.json_response(await orchestrate({"action": "update_status"}))

    async def activate_update(request):
        payload = await json_body(request)
        current = bridge.status() or {}
        if (
            current.get("mode") != "IDLE"
            or current.get("requested_mode") != "IDLE"
            or current.get("transition") != "READY"
            or current.get("safety_state") != "READY_DISARMED"
            or current.get("armed")
        ):
            raise web.HTTPConflict(text="update activation requires IDLE and READY_DISARMED")
        return web.json_response(
            await orchestrate(
                {"action": "activate_update", "version": payload.get("version", "")}
            ),
            status=202,
        )

    async def support_bundle(request):
        del request
        result = await orchestrate({"action": "support_bundle"})
        return web.FileResponse(
            Path(result["path"]),
            headers={
                "Content-Disposition": f'attachment; filename="{result["name"]}"'
            },
        )

    async def telemetry(request):
        protocol = websocket_protocol(request)
        websocket = web.WebSocketResponse(
            heartbeat=15.0, protocols=(protocol,) if protocol else ()
        )
        await websocket.prepare(request)
        try:
            while not websocket.closed:
                value = bridge.status()
                if value is not None:
                    await websocket.send_json(value)
                try:
                    message = await websocket.receive(timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                if message.type in {
                    web.WSMsgType.CLOSE,
                    web.WSMsgType.CLOSED,
                    web.WSMsgType.ERROR,
                }:
                    break
        finally:
            await websocket.close()
        return websocket

    async def teleop(request):
        if request.app[OPERATOR_STATE]["teleop_active"]:
            raise web.HTTPConflict(text="another browser operator holds control")
        protocol = websocket_protocol(request)
        request.app[OPERATOR_STATE]["teleop_active"] = True
        websocket = web.WebSocketResponse(
            heartbeat=5.0,
            max_msg_size=4096,
            protocols=(protocol,) if protocol else (),
        )
        task = None

        async def watchdog():
            nonlocal last_received
            while not websocket.closed:
                await asyncio.sleep(0.05)
                if time.monotonic() - last_received > 0.25:
                    bridge.publish_teleop(0.0, 0.0)

        try:
            await websocket.prepare(request)
            last_sequence = -1
            last_received = time.monotonic()
            task = asyncio.create_task(watchdog())
            async for message in websocket:
                if message.type != web.WSMsgType.TEXT:
                    continue
                try:
                    payload = json.loads(message.data)
                    sequence = int(payload["sequence"])
                    deadman = payload.get("deadman") is True
                except (KeyError, TypeError, ValueError, json.JSONDecodeError):
                    await websocket.close(code=1008, message=b"invalid teleop frame")
                    break
                if sequence <= last_sequence:
                    continue
                last_sequence = sequence
                last_received = time.monotonic()
                if not deadman:
                    bridge.publish_teleop(0.0, 0.0)
                    continue
                current = bridge.status() or {}
                if current.get("mode") != "MANUAL_WEB" or not current.get("armed"):
                    bridge.publish_teleop(0.0, 0.0)
                    continue
                linear = clamp(
                    payload.get("linear_x", 0.0),
                    -MAX_TELEOP_LINEAR,
                    MAX_TELEOP_LINEAR,
                )
                angular = clamp(
                    payload.get("angular_z", 0.0),
                    -MAX_TELEOP_ANGULAR,
                    MAX_TELEOP_ANGULAR,
                )
                bridge.publish_teleop(linear, angular)
        finally:
            if task is not None:
                task.cancel()
            bridge.publish_teleop(0.0, 0.0)
            request.app[OPERATOR_STATE]["teleop_active"] = False
            await websocket.close()
        return websocket

    async def index(request):
        del request
        return web.FileResponse(web_root / "index.html")

    app.router.add_get("/", index)
    app.router.add_static("/assets", web_root / "assets", show_index=False)
    app.router.add_get("/api/v1/health", health)
    app.router.add_post("/api/v1/session", session_login)
    app.router.add_post("/api/v1/companion/pair", companion_pair)
    app.router.add_post(
        "/api/v1/companion/pairing-code", issue_companion_pairing
    )
    app.router.add_post(
        "/api/v1/companion/heartbeat", companion_heartbeat
    )
    app.router.add_get("/api/v1/openapi.json", openapi)
    app.router.add_get("/api/v1/status", status)
    app.router.add_put("/api/v1/mode", set_mode)
    app.router.add_put("/api/v1/sensors/{sensor}", set_sensor)
    app.router.add_put("/api/v1/developer-mode", set_developer)
    app.router.add_get("/api/v1/maps", list_maps)
    app.router.add_post("/api/v1/maps", import_map)
    app.router.add_get("/api/v1/maps/{map_id}/bundle", export_map)
    app.router.add_post("/api/v1/maps/save", save_map)
    app.router.add_post("/api/v1/navigation/goal", navigation_goal)
    app.router.add_get("/api/v1/bluetooth/devices", bluetooth_devices)
    app.router.add_post("/api/v1/bluetooth/pair", bluetooth_pair)
    app.router.add_get("/api/v1/network/wifi", wifi_networks)
    app.router.add_post("/api/v1/network/wifi", wifi_connect)
    app.router.add_get("/api/v1/updates", update_status)
    app.router.add_post("/api/v1/updates/activate", activate_update)
    app.router.add_post("/api/v1/support-bundle", support_bundle)
    app.router.add_get("/api/v1/telemetry", telemetry)
    app.router.add_get("/api/v1/teleop", teleop)
    return app


def load_token(path: Path) -> str:
    """Read a root-provisioned token without accepting loose permissions."""
    stat_result = path.stat()
    if stat_result.st_mode & 0o077:
        raise PermissionError("API token file must not be accessible by other users")
    token = path.read_text(encoding="utf-8").strip()
    if len(token) < 24:
        raise ValueError("API token is invalid")
    return token


def main(args=None) -> None:
    """Run the ROS bridge and HTTPS server until interrupted."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=443)
    parser.add_argument(
        "--token-file",
        type=Path,
        default=Path("/var/lib/studica/secrets/api-token"),
    )
    parser.add_argument(
        "--cert-file", type=Path, default=Path("/var/lib/studica/tls/robot.crt")
    )
    parser.add_argument(
        "--key-file", type=Path, default=Path("/var/lib/studica/tls/robot.key")
    )
    parser.add_argument("--map-root", type=Path, default=Path("/var/lib/studica/maps"))
    parser.add_argument(
        "--pairing-root",
        type=Path,
        default=Path("/var/lib/studica/pairing"),
    )
    parser.add_argument("--web-root", type=Path)
    parser.add_argument("--allow-insecure-http", action="store_true")
    parser.add_argument("--observation-only", action="store_true")
    options, ros_args = parser.parse_known_args(args)

    web_root = options.web_root or (
        Path(get_package_share_directory("studica_vmxpi_ros2")) / "deployment" / "web"
    )
    rclpy.init(args=ros_args)
    if options.observation_only:
        from .sensor_observer import SensorObserver

        bridge = SensorObserver()
    else:
        bridge = RosBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    spin_thread = threading.Thread(target=executor.spin, name="ros-api-bridge", daemon=True)
    spin_thread.start()
    companion_pairing = CompanionPairingStore(options.pairing_root)
    app = create_app(
        bridge,
        AuthStore(load_token(options.token_file), companion_pairing),
        MapRegistry(options.map_root),
        companion_pairing,
        web_root,
        observation_only=options.observation_only,
    )
    app[ALLOW_INSECURE_HTTP] = options.allow_insecure_http
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    ssl_context = None
    if not options.allow_insecure_http:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(options.cert_file, options.key_file)
    try:
        web.run_app(
            app,
            host=options.host,
            port=options.port,
            ssl_context=ssl_context,
            handle_signals=True,
            loop=loop,
        )
    finally:
        if not options.observation_only:
            bridge.publish_teleop(0.0, 0.0)
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
