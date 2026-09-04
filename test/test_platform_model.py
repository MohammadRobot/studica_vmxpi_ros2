from studica_robot_platform.model import (
    CommandArbiter,
    Mode,
    PlanarCommand,
    PlatformModel,
    ZERO_COMMAND,
)


def test_boot_and_every_mode_transition_are_disarmed_first():
    model = PlatformModel()
    assert model.mode == Mode.IDLE
    assert model.request_mode(Mode.MANUAL_WEB).accepted
    assert model.mode == Mode.MANUAL_WEB
    assert not model.request_mode(Mode.NAVIGATION, "office_nav").accepted
    model.companion_connected = True
    model.lidar_healthy = True
    assert model.request_mode(Mode.NAVIGATION, "office_nav").accepted
    assert model.mode == Mode.IDLE
    assert model.transition == "WAITING_FOR_COMPANION"
    model.companion_ready(Mode.NAVIGATION, True, "")
    assert model.mode == Mode.NAVIGATION
    assert model.transition == "READY"
    assert model.lose_companion()
    assert model.mode == Mode.IDLE


def test_arbiter_is_fail_closed_for_every_invalid_condition():
    arbiter = CommandArbiter(0.25)
    command = PlanarCommand(0.2, 0.0, 0.3)
    arbiter.update("web", 10.0, command)
    assert arbiter.select(Mode.MANUAL_WEB, 10.1, "READY_DISARMED", 1, False)[0] == ZERO_COMMAND
    assert arbiter.select(Mode.MANUAL_WEB, 10.1, "ARMED", 0, False)[0] == ZERO_COMMAND
    assert arbiter.select(Mode.MANUAL_WEB, 10.1, "ARMED", 2, False)[0] == ZERO_COMMAND
    assert arbiter.select(Mode.MANUAL_WEB, 10.3, "ARMED", 1, False)[0] == ZERO_COMMAND
    assert arbiter.select(Mode.MANUAL_WEB, 10.1, "ARMED", 1, False)[0] == command


def test_joystick_always_requires_fresh_deadman():
    arbiter = CommandArbiter(0.25)
    command = PlanarCommand(0.1, 0.0, 0.0)
    arbiter.update("joystick", 2.0, command)
    assert arbiter.select(Mode.MANUAL_JOYSTICK, 2.1, "ARMED", 1, False)[0] == ZERO_COMMAND
    assert arbiter.select(Mode.MANUAL_JOYSTICK, 2.1, "ARMED", 1, True)[0] == command
