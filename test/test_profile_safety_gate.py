#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Tests for fail-closed physical safety-gate profile validation."""

from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml


ROOT = Path(__file__).parents[1]
LAUNCH_DIR = ROOT / "bringup" / "launch"
if str(LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(LAUNCH_DIR))

from profile_validation import validate_profile_files  # noqa: E402


PROFILE_PATH = ROOT / "bringup" / "config" / "profiles" / "stack_4wd" / "robot_profile.yaml"
CONTROLLERS_PATH = (
    ROOT / "bringup" / "config" / "profiles" / "stack_4wd" / "robot_controllers.yaml"
)


def load_profile():
    return yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))


def validate(profile, tmp_path):
    profile_path = tmp_path / "robot_profile.yaml"
    profile_path.write_text(yaml.safe_dump(profile), encoding="utf-8")
    errors, *_ = validate_profile_files(
        "stack_4wd", profile_path, CONTROLLERS_PATH
    )
    return errors


def test_stack_4wd_uses_confirmed_safety_channels(tmp_path):
    profile = load_profile()
    safety = profile["hardware"]["safety_gate"]
    assert safety["estop_ok_dio_channel"] == 8
    assert safety["start_button_dio_channel"] == 9
    assert safety["reset_button_dio_channel"] == 10
    assert safety["stop_ok_dio_channel"] == 11
    assert safety["start_led_dio_channel"] == 12
    assert safety["stop_led_dio_channel"] == 13
    assert profile["hardware"]["titan_encoder_cpr"] == 1464
    assert profile["hardware"]["controller_temperature_safety_enabled"] is False
    assert validate(profile, tmp_path) == []


def test_unconfigured_panel_is_valid_but_remains_a_runtime_block(tmp_path):
    profile = deepcopy(load_profile())
    safety = profile["hardware"]["safety_gate"]
    for key in (
        "estop_ok_dio_channel",
        "start_button_dio_channel",
        "reset_button_dio_channel",
        "stop_ok_dio_channel",
        "start_led_dio_channel",
        "stop_led_dio_channel",
    ):
        safety[key] = -1
    assert validate(profile, tmp_path) == []


def test_configured_distinct_panel_channels_are_valid(tmp_path):
    profile = deepcopy(load_profile())
    safety = profile["hardware"]["safety_gate"]
    safety["estop_ok_dio_channel"] = 0
    safety["start_button_dio_channel"] = 1
    safety["reset_button_dio_channel"] = 2
    safety["stop_ok_dio_channel"] = 3
    safety["start_led_dio_channel"] = 4
    safety["stop_led_dio_channel"] = 5
    assert validate(profile, tmp_path) == []


@pytest.mark.parametrize(
    ("estop_channel", "start_channel", "message"),
    [
        (-1, 10, "must all be configured"),
        (10, 10, "must be different"),
        (30, 10, "must be -1 or in [0, 29]"),
    ],
)
def test_invalid_channel_sets_are_rejected(
    tmp_path, estop_channel, start_channel, message
):
    profile = deepcopy(load_profile())
    safety = profile["hardware"]["safety_gate"]
    safety["estop_ok_dio_channel"] = estop_channel
    safety["start_button_dio_channel"] = start_channel
    assert any(message in error for error in validate(profile, tmp_path))


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("button_debounce_ms", -1, "button_debounce_ms must be >= 0"),
        ("safe_release_ms", 0, "safe_release_ms must be > 0"),
    ],
)
def test_invalid_gate_timings_are_rejected(tmp_path, key, value, message):
    profile = deepcopy(load_profile())
    profile["hardware"]["safety_gate"][key] = value
    assert any(message in error for error in validate(profile, tmp_path))


def test_temperature_safety_mode_must_be_boolean(tmp_path):
    profile = deepcopy(load_profile())
    profile["hardware"]["controller_temperature_safety_enabled"] = "false"
    assert any(
        "controller_temperature_safety_enabled must be bool" in error
        for error in validate(profile, tmp_path)
    )


@pytest.mark.parametrize("value", [0, -1, 65536, 732.0, True])
def test_velocity_pid_requires_valid_titan_encoder_cpr(tmp_path, value):
    profile = deepcopy(load_profile())
    profile["hardware"]["titan_encoder_cpr"] = value
    assert any("titan_encoder_cpr" in error for error in validate(profile, tmp_path))
