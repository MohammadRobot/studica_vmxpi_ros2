"""Pure state and command arbitration for the managed robot platform."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import math
import re
from typing import Dict, Tuple


MAP_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}")


class Mode(IntEnum):
    """Mutually exclusive robot operating modes exposed by API v1."""

    IDLE = 0
    MANUAL_JOYSTICK = 1
    MANUAL_WEB = 2
    SLAM = 3
    NAVIGATION = 4
    DEVELOPER = 5


MODE_NAMES = {mode: mode.name for mode in Mode}
MODE_SOURCES = {
    Mode.IDLE: "",
    Mode.MANUAL_JOYSTICK: "joystick",
    Mode.MANUAL_WEB: "web",
    Mode.SLAM: "joystick",
    Mode.NAVIGATION: "navigation",
    Mode.DEVELOPER: "developer",
}
AUTONOMY_MODES = {Mode.SLAM, Mode.NAVIGATION}


def parse_mode(value: int) -> Mode:
    """Return a supported mode or raise a stable validation error."""
    try:
        return Mode(value)
    except ValueError as error:
        raise ValueError(f"unsupported mode: {value}") from error


def valid_map_id(map_id: str) -> bool:
    """Return whether *map_id* is safe for use as a registry key."""
    return bool(MAP_ID_PATTERN.fullmatch(map_id))


@dataclass(frozen=True)
class ModeDecision:
    """Result of a requested mode transition."""

    accepted: bool
    message: str


@dataclass
class PlatformModel:
    """Fail-closed state machine independent of ROS and systemd."""

    mode: Mode = Mode.IDLE
    requested_mode: Mode = Mode.IDLE
    transition: str = "READY"
    requested_map_id: str = ""
    last_error: str = ""
    developer_mode: bool = False
    lidar_enabled: bool = True
    lidar_healthy: bool = False
    camera_enabled: bool = False
    camera_healthy: bool = False
    companion_connected: bool = False

    def validate_mode(self, mode: Mode, map_id: str = "") -> ModeDecision:
        """Check a request without changing platform state."""
        if mode == Mode.NAVIGATION and not valid_map_id(map_id):
            return ModeDecision(False, "navigation requires a valid map_id")
        if mode in AUTONOMY_MODES and not self.companion_connected:
            return ModeDecision(False, "Ubuntu companion is not connected")
        if mode in AUTONOMY_MODES and not self.lidar_enabled:
            return ModeDecision(False, "LiDAR must be enabled")
        if mode in AUTONOMY_MODES and not self.lidar_healthy:
            return ModeDecision(False, "LiDAR must be healthy")
        if mode == Mode.DEVELOPER and not self.developer_mode:
            return ModeDecision(False, "developer mode is disabled")
        return ModeDecision(True, f"mode request accepted: {MODE_NAMES[mode]}")

    def request_mode(self, mode: Mode, map_id: str = "") -> ModeDecision:
        """Begin a transition only after the caller has confirmed physical disarm."""
        decision = self.validate_mode(mode, map_id)
        if not decision.accepted:
            return decision

        self.mode = Mode.IDLE
        self.requested_mode = mode
        self.requested_map_id = map_id if mode == Mode.NAVIGATION else ""
        self.last_error = ""
        if mode in AUTONOMY_MODES:
            self.transition = "WAITING_FOR_COMPANION"
        else:
            self.mode = mode
            self.transition = "READY"
        return decision

    def companion_ready(self, mode: Mode, ready: bool, detail: str) -> None:
        """Complete or fail a pending autonomy transition."""
        self.companion_connected = True
        if self.requested_mode not in AUTONOMY_MODES:
            return
        if mode != self.requested_mode:
            return
        if ready:
            self.mode = mode
            self.transition = "READY"
            self.last_error = ""
        elif detail:
            self.mode = Mode.IDLE
            self.transition = "ERROR"
            self.last_error = detail

    def lose_companion(self) -> bool:
        """Stop autonomy after companion loss; return whether mode changed."""
        self.companion_connected = False
        if self.mode in AUTONOMY_MODES or self.requested_mode in AUTONOMY_MODES:
            self.mode = Mode.IDLE
            self.requested_mode = Mode.IDLE
            self.transition = "ERROR"
            self.last_error = "COMPANION_LOST"
            return True
        return False


@dataclass(frozen=True)
class PlanarCommand:
    """Planar velocity command held by the arbiter."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    def finite(self) -> bool:
        return all(
            math.isfinite(value)
            for value in (self.linear_x, self.linear_y, self.angular_z)
        )


ZERO_COMMAND = PlanarCommand()


@dataclass
class CommandArbiter:
    """Select one fresh, valid source for the safety supervisor."""

    timeout_sec: float = 0.25
    samples: Dict[str, Tuple[float, PlanarCommand]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not 0.05 <= self.timeout_sec <= 0.5:
            raise ValueError("timeout_sec must be in [0.05, 0.5]")

    def update(self, source: str, received_at: float, command: PlanarCommand) -> None:
        """Record the newest command for a known source."""
        if source not in {"joystick", "web", "navigation", "developer"}:
            raise ValueError(f"unknown command source: {source}")
        self.samples[source] = (received_at, command)

    def clear(self) -> None:
        """Forget all commands during every mode transition."""
        self.samples.clear()

    def select(
        self,
        mode: Mode,
        now: float,
        safety_state: str,
        publisher_count: int,
        joystick_deadman: bool,
    ) -> Tuple[PlanarCommand, str]:
        """Return the sole accepted command and a diagnostic reason."""
        source = MODE_SOURCES[mode]
        if safety_state != "ARMED":
            return ZERO_COMMAND, "PLATFORM_DISARMED"
        if not source:
            return ZERO_COMMAND, "NO_CONTROL_SOURCE"
        if publisher_count != 1:
            return ZERO_COMMAND, (
                "COMMAND_SOURCE_LOST"
                if publisher_count == 0
                else "COMMAND_SOURCE_CONFLICT"
            )
        if source == "joystick" and not joystick_deadman:
            return ZERO_COMMAND, "JOYSTICK_DEADMAN_RELEASED"
        sample = self.samples.get(source)
        if sample is None:
            return ZERO_COMMAND, "WAITING_FOR_COMMAND"
        received_at, command = sample
        age = now - received_at
        if age < 0.0 or age > self.timeout_sec:
            return ZERO_COMMAND, "COMMAND_STALE"
        if not command.finite():
            return ZERO_COMMAND, "COMMAND_NONFINITE"
        return command, "COMMAND_ACCEPTED"
