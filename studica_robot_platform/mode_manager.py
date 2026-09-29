"""ROS 2 mode manager and sole command arbiter for the robot platform."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from typing import Dict

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Joy, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from studica_vmxpi_ros2.msg import CompanionHeartbeat, PlatformStatus
from studica_vmxpi_ros2.srv import SetDeveloperMode, SetMode, SetSensor

from . import API_VERSION
from .model import (
    AUTONOMY_MODES,
    MODE_NAMES,
    MODE_SOURCES,
    CommandArbiter,
    Mode,
    PlanarCommand,
    PlatformModel,
    parse_mode,
)
from .orchestrator_client import OrchestratorError, request as orchestrator_request


STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ModeManager(Node):
    """Own operating mode, sensor intent, and the sole safe command ingress."""

    def __init__(self) -> None:
        super().__init__("studica_mode_manager")
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("companion_timeout_sec", 2.0)
        self.declare_parameter("joystick_deadman_button", 4)
        self.declare_parameter("orchestrator_socket", "/run/studica/orchestrator.sock")
        self.declare_parameter("orchestrator_required", True)
        self.declare_parameter("lidar_freshness_sec", 1.0)
        self.declare_parameter("camera_freshness_sec", 2.0)
        self.declare_parameter("output_cmd_vel_topic", "/robot/platform/cmd_vel")
        self.declare_parameter("navigation_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("maintenance_lock_file", "/var/lib/studica/activation-in-progress")
        self.declare_parameter(
            "status_snapshot_path", "/run/studica/platform-status.json"
        )

        command_timeout = float(self.get_parameter("command_timeout_sec").value)
        self._companion_timeout = float(
            self.get_parameter("companion_timeout_sec").value
        )
        self._deadman_button = int(
            self.get_parameter("joystick_deadman_button").value
        )
        self._orchestrator_socket = str(
            self.get_parameter("orchestrator_socket").value
        )
        self._orchestrator_required = bool(
            self.get_parameter("orchestrator_required").value
        )
        self._lidar_freshness = float(
            self.get_parameter("lidar_freshness_sec").value
        )
        self._camera_freshness = float(
            self.get_parameter("camera_freshness_sec").value
        )
        self._output_cmd_vel_topic = str(
            self.get_parameter("output_cmd_vel_topic").value
        )
        navigation_topic = str(
            self.get_parameter("navigation_cmd_vel_topic").value
        )
        self._status_snapshot_path = Path(
            str(self.get_parameter("status_snapshot_path").value)
        )
        if not 0 <= self._deadman_button <= 31:
            raise ValueError("joystick_deadman_button must be in [0, 31]")
        if self._companion_timeout <= 0.0:
            raise ValueError("companion_timeout_sec must be positive")

        self._lock = threading.RLock()
        self._model = PlatformModel()
        self._arbiter = CommandArbiter(command_timeout)
        self._safety_state = "BOOTING"
        self._safety_reason = "WAITING_FOR_SAFETY_SUPERVISOR"
        self._command_reason = "PLATFORM_DISARMED"
        self._command_was_live = False
        self._command_loss_inhibit = False
        self._last_command_disarm = 0.0
        self._maintenance_lock = Path(str(self.get_parameter("maintenance_lock_file").value))
        self._last_companion = 0.0
        self._last_joy = 0.0
        self._joystick_deadman = False
        self._last_lidar = 0.0
        self._last_camera = 0.0
        self._sequence = 0
        self._pending_mode = Mode.IDLE
        self._pending_map_id = ""
        self._pending_generation = 0
        self._transport_safe = True

        self._command_topics: Dict[str, str] = {
            "joystick": "/robot/control/joystick",
            "web": "/robot/control/web",
            "navigation": navigation_topic,
            "developer": "/robot/control/developer",
        }
        self._command_publisher = self.create_publisher(
            Twist, self._output_cmd_vel_topic, 1
        )
        self._status_publisher = self.create_publisher(
            PlatformStatus, "/robot/platform/status", STATUS_QOS
        )
        self._event_publisher = self.create_publisher(
            String, "/robot/platform/events", 10
        )
        self._command_subscriptions = []
        for source, topic in self._command_topics.items():
            self._command_subscriptions.append(self.create_subscription(
                Twist,
                topic,
                lambda message, selected=source: self._on_command(selected, message),
                1,
            ))

        self.create_subscription(String, "/robot/state", self._on_safety_state, 10)
        self.create_subscription(
            String, "/robot/safety_reason", self._on_safety_reason, 10
        )
        self.create_subscription(Joy, "/joy", self._on_joy, qos_profile_sensor_data)
        self.create_subscription(
            CompanionHeartbeat,
            "/robot/companion/heartbeat",
            self._on_companion,
            10,
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_lidar, qos_profile_sensor_data
        )
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self._on_camera,
            qos_profile_sensor_data,
        )

        self._disarm_client = self.create_client(Trigger, "/robot/disarm")
        self.create_service(SetMode, "/robot/platform/set_mode", self._set_mode)
        self.create_service(SetSensor, "/robot/platform/set_sensor", self._set_sensor)
        self.create_service(
            SetDeveloperMode,
            "/robot/platform/set_developer_mode",
            self._set_developer_mode,
        )
        self.create_timer(0.02, self._publish_selected_command)
        self.create_timer(0.5, self._publish_status)
        self._publish_zero()
        if self._orchestrator_required:
            companion_closed = self._set_dds_transport("companion", False)
            developer_closed = self._set_dds_transport("developer", False)
            self._transport_safe = companion_closed and developer_closed
            if not self._transport_safe:
                self._model.last_error = "DDS_FIREWALL_CLEANUP_FAILED"
        self._publish_status()
        self.get_logger().info(
            f"Platform manager started in IDLE; sole robot output is "
            f"{self._output_cmd_vel_topic}"
        )

    def _emit_event(self, event: str, detail: str = "") -> None:
        message = String()
        message.data = json.dumps(
            {
                "api_version": API_VERSION,
                "event": event,
                "detail": detail,
                "monotonic_ns": time.monotonic_ns(),
            },
            separators=(",", ":"),
            sort_keys=True,
        )
        self._event_publisher.publish(message)

    def _publish_zero(self) -> None:
        self._command_publisher.publish(Twist())

    def _request_disarm(self):
        self._publish_zero()
        self._arbiter.clear()
        if not self._disarm_client.service_is_ready():
            return None
        return self._disarm_client.call_async(Trigger.Request())

    def _set_dds_transport(self, transport: str, enabled: bool) -> bool:
        if not self._orchestrator_required:
            return True
        action = (
            "set_companion_transport"
            if transport == "companion"
            else "set_developer_firewall"
        )
        try:
            result = orchestrator_request(
                {"action": action, "enabled": enabled},
                socket_path=self._orchestrator_socket,
            )
        except OrchestratorError as error:
            self._model.last_error = str(error)
            self._transport_safe = False
            return False
        if not result.get("ok"):
            self._model.last_error = str(result.get("error", "request rejected"))
            self._transport_safe = False
            return False
        return True

    def _reject_pending_mode(self, reason: str) -> None:
        self._model.mode = Mode.IDLE
        self._model.requested_mode = Mode.IDLE
        self._model.transition = "ERROR"
        self._model.last_error = reason
        self._pending_mode = Mode.IDLE
        self._pending_map_id = ""
        self._emit_event("MODE_REJECTED", reason)

    def _finish_pending_mode(self, future, generation: int) -> None:
        with self._lock:
            if generation != self._pending_generation:
                return
            try:
                result = future.result()
            except Exception as error:
                result = None
                self._model.last_error = f"DISARM_FAILED: {error}"
            if result is None or not result.success:
                self._set_dds_transport("companion", False)
                self._model.mode = Mode.IDLE
                self._model.requested_mode = Mode.IDLE
                self._model.transition = "ERROR"
                if not self._model.last_error:
                    self._model.last_error = "DISARM_REJECTED"
                self._pending_mode = Mode.IDLE
                self._pending_map_id = ""
                self._emit_event("MODE_REJECTED", self._model.last_error)
                return
            mode = self._pending_mode
            map_id = self._pending_map_id
            self._pending_mode = Mode.IDLE
            self._pending_map_id = ""
            if mode in AUTONOMY_MODES:
                if not self._set_dds_transport("companion", True):
                    self._reject_pending_mode("COMPANION_DDS_ENABLE_FAILED")
                    self._publish_status()
                    return
            else:
                if not self._set_dds_transport("companion", False):
                    self._reject_pending_mode("COMPANION_DDS_DISABLE_FAILED")
                    self._publish_status()
                    return
            if mode != Mode.DEVELOPER and self._model.developer_mode:
                if not self._set_dds_transport("developer", False):
                    self._reject_pending_mode("DEVELOPER_DDS_DISABLE_FAILED")
                    self._publish_status()
                    return
                self._model.developer_mode = False
            decision = self._model.request_mode(mode, map_id)
            if decision.accepted:
                self._emit_event("MODE_READY_FOR_ARM", MODE_NAMES[mode])
            else:
                self._model.mode = Mode.IDLE
                self._model.requested_mode = Mode.IDLE
                self._model.transition = "ERROR"
                self._model.last_error = decision.message
                if mode in AUTONOMY_MODES:
                    self._set_dds_transport("companion", False)
                self._emit_event("MODE_REJECTED", decision.message)
            self._publish_status()

    def _finish_companion_mode(self, future, generation: int, mode: Mode) -> None:
        """Enter autonomy only after a final disarm at dependency readiness."""
        with self._lock:
            if generation != self._pending_generation:
                return
            if (
                self._model.requested_mode != mode
                or self._model.transition != "WAITING_FOR_FINAL_DISARM"
            ):
                return
            try:
                result = future.result()
            except Exception as error:
                result = None
                self._model.last_error = f"FINAL_DISARM_FAILED: {error}"
            if result is None or not result.success:
                self._model.mode = Mode.IDLE
                self._model.requested_mode = Mode.IDLE
                self._model.transition = "ERROR"
                if not self._model.last_error:
                    self._model.last_error = "FINAL_DISARM_REJECTED"
                self._set_dds_transport("companion", False)
                self._emit_event("MODE_REJECTED", self._model.last_error)
            else:
                self._model.companion_ready(mode, True, "")
                self._emit_event("MODE_READY_FOR_ARM", MODE_NAMES[mode])
            self._publish_status()

    def _set_mode(self, request: SetMode.Request, response: SetMode.Response):
        if self._maintenance_active():
            response.accepted = False
            response.message = "release activation is in progress"
            return response
        with self._lock:
            try:
                mode = parse_mode(request.mode)
            except ValueError as error:
                response.accepted = False
                response.message = str(error)
                return response
            self._model.lidar_healthy = self._lidar_is_fresh(
                time.monotonic()
            )
            decision = self._model.validate_mode(mode, request.map_id.strip())
            if not decision.accepted:
                response.accepted = False
                response.message = decision.message
                return response
            if mode != Mode.IDLE and not self._transport_safe:
                response.accepted = False
                response.message = "DDS firewall cleanup has not been verified"
                return response
            future = self._request_disarm()
            if future is None:
                response.accepted = False
                response.message = "safety supervisor disarm service is unavailable"
                return response
            self._pending_generation += 1
            generation = self._pending_generation
            self._pending_mode = mode
            self._pending_map_id = request.map_id.strip()
            self._model.mode = Mode.IDLE
            self._model.requested_mode = mode
            self._model.requested_map_id = self._pending_map_id
            self._model.transition = "WAITING_FOR_DISARM"
            self._model.last_error = ""
            future.add_done_callback(
                lambda completed, selected=generation: self._finish_pending_mode(
                    completed, selected
                )
            )
            self._emit_event("MODE_REQUESTED", MODE_NAMES[mode])
            self._publish_status()
            response.accepted = True
            response.message = decision.message
            return response

    def _set_sensor(
        self, request: SetSensor.Request, response: SetSensor.Response
    ) -> SetSensor.Response:
        sensor = request.sensor.strip().lower()
        with self._lock:
            if sensor not in {"lidar", "camera"}:
                response.accepted = False
                response.message = "sensor must be lidar or camera"
                return response
            if self._safety_state != "READY_DISARMED":
                response.accepted = False
                response.message = "sensor changes require READY_DISARMED"
                return response
            if (
                sensor == "lidar"
                and not request.enabled
                and self._model.requested_mode in AUTONOMY_MODES
            ):
                response.accepted = False
                response.message = "LiDAR is required by SLAM and navigation"
                return response
            try:
                result = self._orchestrate_sensor(sensor, request.enabled)
            except OrchestratorError as error:
                response.accepted = False
                response.message = str(error)
                return response
            if sensor == "lidar":
                self._model.lidar_enabled = request.enabled
                if not request.enabled:
                    self._model.lidar_healthy = False
            else:
                self._model.camera_enabled = request.enabled
                if not request.enabled:
                    self._model.camera_healthy = False
            response.accepted = True
            response.message = result
            self._emit_event("SENSOR_CHANGED", f"{sensor}={request.enabled}")
            self._publish_status()
            return response

    def _orchestrate_sensor(self, sensor: str, enabled: bool) -> str:
        if not self._orchestrator_required:
            return f"{sensor} intent updated (test backend)"
        response = orchestrator_request(
            {"action": "set_sensor", "sensor": sensor, "enabled": enabled},
            socket_path=self._orchestrator_socket,
        )
        if not response.get("ok"):
            raise OrchestratorError(str(response.get("error", "request rejected")))
        return str(response.get("message", "request accepted"))

    def _set_developer_mode(
        self,
        request: SetDeveloperMode.Request,
        response: SetDeveloperMode.Response,
    ) -> SetDeveloperMode.Response:
        with self._lock:
            if self._safety_state != "READY_DISARMED":
                response.accepted = False
                response.message = "developer mode requires READY_DISARMED"
                return response
            if (
                self._model.mode != Mode.IDLE
                or self._model.requested_mode != Mode.IDLE
                or self._model.transition != "READY"
            ):
                response.accepted = False
                response.message = "developer mode changes require ready IDLE mode"
                return response
            self._request_disarm()
            if request.enabled and not self._set_dds_transport(
                "companion", False
            ):
                response.accepted = False
                response.message = self._model.last_error
                return response
            if not self._set_dds_transport("developer", request.enabled):
                response.accepted = False
                response.message = self._model.last_error
                return response
            self._model.developer_mode = request.enabled
            if not request.enabled and self._model.mode == Mode.DEVELOPER:
                self._model.request_mode(Mode.IDLE)
            response.accepted = True
            response.message = (
                "developer DDS ingress enabled"
                if request.enabled
                else "developer DDS ingress disabled"
            )
            self._emit_event("DEVELOPER_MODE_CHANGED", str(request.enabled))
            self._publish_status()
            return response

    def _on_command(self, source: str, message: Twist) -> None:
        command = PlanarCommand(
            linear_x=message.linear.x,
            linear_y=message.linear.y,
            angular_z=message.angular.z,
        )
        with self._lock:
            self._arbiter.update(source, time.monotonic(), command)

    def _on_safety_state(self, message: String) -> None:
        with self._lock:
            self._safety_state = message.data
            if message.data != "ARMED":
                self._arbiter.clear()

    def _on_safety_reason(self, message: String) -> None:
        with self._lock:
            self._safety_reason = message.data

    def _on_joy(self, message: Joy) -> None:
        now = time.monotonic()
        valid = self._deadman_button < len(message.buttons)
        active = valid and message.buttons[self._deadman_button] == 1
        with self._lock:
            self._last_joy = now
            self._joystick_deadman = active

    def _on_companion(self, message: CompanionHeartbeat) -> None:
        now = time.monotonic()
        with self._lock:
            if message.api_version.split(".", 1)[0] != API_VERSION.split(".", 1)[0]:
                self._model.last_error = "COMPANION_API_INCOMPATIBLE"
                return
            try:
                active_mode = parse_mode(message.active_mode)
            except ValueError:
                self._model.last_error = "COMPANION_MODE_INVALID"
                return
            self._last_companion = now
            if not message.connected:
                if self._model.lose_companion():
                    self._pending_generation += 1
                    self._pending_mode = Mode.IDLE
                    self._pending_map_id = ""
                    self._request_disarm()
                    self._set_dds_transport("companion", False)
                    self._emit_event("COMPANION_LOST")
                return
            self._model.companion_connected = True
            if (
                message.ready
                and active_mode == self._model.requested_mode
                and self._model.transition == "WAITING_FOR_COMPANION"
            ):
                future = self._request_disarm()
                if future is None:
                    self._model.mode = Mode.IDLE
                    self._model.requested_mode = Mode.IDLE
                    self._model.transition = "ERROR"
                    self._model.last_error = "FINAL_DISARM_SERVICE_UNAVAILABLE"
                    self._emit_event("MODE_REJECTED", self._model.last_error)
                    return
                generation = self._pending_generation
                self._model.transition = "WAITING_FOR_FINAL_DISARM"
                future.add_done_callback(
                    lambda completed, selected=generation, mode=active_mode: (
                        self._finish_companion_mode(completed, selected, mode)
                    )
                )
                return
            was_waiting = self._model.requested_mode in AUTONOMY_MODES
            self._model.companion_ready(active_mode, False, message.detail)
            if was_waiting and self._model.transition == "ERROR":
                self._request_disarm()
                self._set_dds_transport("companion", False)
                self._emit_event("MODE_REJECTED", self._model.last_error)

    def _on_lidar(self, message: LaserScan) -> None:
        metadata = (
            message.angle_min,
            message.angle_max,
            message.angle_increment,
            message.range_min,
            message.range_max,
        )
        if (
            len(message.ranges) < 10
            or not all(math.isfinite(value) for value in metadata)
            or message.angle_increment <= 0.0
            or message.angle_max <= message.angle_min
            or message.range_min < 0.0
            or message.range_max <= message.range_min
        ):
            return
        with self._lock:
            self._last_lidar = time.monotonic()

    def _on_camera(self, message: Image) -> None:
        if (
            message.width <= 0
            or message.height <= 0
            or not message.encoding
            or not message.data
        ):
            return
        with self._lock:
            self._last_camera = time.monotonic()

    def _lidar_is_fresh(self, now: float) -> bool:
        return self._model.lidar_enabled and (
            self._last_lidar > 0.0
            and now - self._last_lidar <= self._lidar_freshness
        )

    def _maintenance_active(self) -> bool:
        try:
            return self._maintenance_lock.exists()
        except OSError:
            return True

    def _publish_selected_command(self) -> None:
        now = time.monotonic()
        with self._lock:
            source = MODE_SOURCES[self._model.mode]
            publisher_count = (
                self.count_publishers(self._command_topics[source]) if source else 0
            )
            deadman_fresh = now - self._last_joy <= 0.25
            if self._model.mode in AUTONOMY_MODES and not self._lidar_is_fresh(now):
                command = PlanarCommand()
                reason = "LIDAR_UNHEALTHY"
            else:
                command, reason = self._arbiter.select(
                    self._model.mode,
                    now,
                    self._safety_state,
                    publisher_count,
                    self._joystick_deadman and deadman_fresh,
                )
            output = Twist()
            if self._maintenance_active():
                command = PlanarCommand()
                reason = "MAINTENANCE_LOCKED"
                if now - self._last_command_disarm >= 1.0:
                    self._request_disarm()
                    self._last_command_disarm = now
            if self._safety_state != "ARMED":
                self._command_was_live = False
                self._command_loss_inhibit = False
            elif self._command_was_live and reason in {
                "COMMAND_STALE", "COMMAND_SOURCE_LOST", "COMMAND_SOURCE_CONFLICT",
                "COMMAND_NONFINITE",
            }:
                self._command_loss_inhibit = True
                self._arbiter.clear()
            if self._command_loss_inhibit:
                command = PlanarCommand()
                reason = "COMMAND_LOST_WAITING_FOR_DISARM"
                if now - self._last_command_disarm >= 1.0:
                    self._request_disarm()
                    self._last_command_disarm = now
            elif reason == "COMMAND_ACCEPTED":
                self._command_was_live = True
            output.linear.x = command.linear_x
            output.linear.y = command.linear_y
            output.angular.z = command.angular_z
            self._command_reason = reason
            self._command_publisher.publish(output)

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self._lock:
            if self._last_companion and now - self._last_companion > self._companion_timeout:
                if self._model.lose_companion():
                    self._pending_generation += 1
                    self._pending_mode = Mode.IDLE
                    self._pending_map_id = ""
                    self._request_disarm()
                    self._set_dds_transport("companion", False)
                    self._emit_event("COMPANION_LOST")
            self._model.lidar_healthy = self._lidar_is_fresh(now)
            self._model.camera_healthy = self._model.camera_enabled and (
                self._last_camera > 0.0 and now - self._last_camera <= self._camera_freshness
            )
            if (
                not self._model.lidar_healthy
                and (
                    self._model.mode in AUTONOMY_MODES
                    or self._model.requested_mode in AUTONOMY_MODES
                )
            ):
                self._request_disarm()
                self._set_dds_transport("companion", False)
                self._model.mode = Mode.IDLE
                self._model.requested_mode = Mode.IDLE
                self._model.transition = "ERROR"
                self._model.last_error = "LIDAR_LOST"
                self._pending_generation += 1
                self._pending_mode = Mode.IDLE
                self._pending_map_id = ""
                self._emit_event("MODE_REJECTED", "LIDAR_LOST")
            self._sequence += 1
            status = PlatformStatus()
            status.stamp = self.get_clock().now().to_msg()
            status.api_version = API_VERSION
            status.sequence = self._sequence
            status.mode = int(self._model.mode)
            status.requested_mode = int(self._model.requested_mode)
            status.mode_name = MODE_NAMES[self._model.mode]
            status.requested_mode_name = MODE_NAMES[self._model.requested_mode]
            status.transition = self._model.transition
            status.safety_state = self._safety_state
            status.safety_reason = self._safety_reason
            status.ready = (
                self._model.transition == "READY"
                and self._safety_state in {"READY_DISARMED", "ARMED"}
                and (not self._model.lidar_enabled or self._model.lidar_healthy)
            )
            status.armed = self._safety_state == "ARMED"
            status.lidar_enabled = self._model.lidar_enabled
            status.lidar_healthy = self._model.lidar_healthy
            status.camera_enabled = self._model.camera_enabled
            status.camera_healthy = self._model.camera_healthy
            status.companion_connected = self._model.companion_connected
            status.developer_mode = self._model.developer_mode
            status.active_control_source = MODE_SOURCES[self._model.mode]
            status.requested_map_id = self._model.requested_map_id
            status.last_error = self._model.last_error or self._command_reason
            self._status_publisher.publish(status)
            self._write_status_snapshot(status)

    def _write_status_snapshot(self, status: PlatformStatus) -> None:
        document = {
            "api_version": status.api_version,
            "sequence": status.sequence,
            "mode": status.mode_name,
            "requested_mode": status.requested_mode_name,
            "transition": status.transition,
            "safety_state": status.safety_state,
            "safety_reason": status.safety_reason,
            "ready": status.ready,
            "armed": status.armed,
            "lidar_healthy": status.lidar_healthy,
            "camera_healthy": status.camera_healthy,
            "companion_connected": status.companion_connected,
            "last_error": status.last_error,
        }
        try:
            self._status_snapshot_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._status_snapshot_path.with_name(
                f".{self._status_snapshot_path.name}.{os.getpid()}"
            )
            temporary.write_text(
                json.dumps(document, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            os.chmod(temporary, 0o640)
            os.replace(temporary, self._status_snapshot_path)
        except OSError as error:
            self.get_logger().warn(
                f"cannot write platform status snapshot: {error}",
                throttle_duration_sec=30.0,
            )

    def close(self) -> None:
        """Fail closed before executor and local orchestration disappear."""
        with self._lock:
            self._request_disarm()
            self._set_dds_transport("companion", False)
            self._set_dds_transport("developer", False)


def main(args=None) -> None:
    """Run the managed robot mode and command arbiter node."""
    # Keep the context valid while Python handles SIGINT so shutdown can send
    # a final explicit zero before the safety supervisor's timeout also trips.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def request_interrupt(signum, frame) -> None:
        del signum, frame
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, request_interrupt)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
