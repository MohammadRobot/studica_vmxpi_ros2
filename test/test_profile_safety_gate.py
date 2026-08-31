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
    assert safety["local_enable_dio_channel"] == 9
    assert profile["hardware"]["titan_encoder_cpr"] == 732
    assert profile["hardware"]["controller_temperature_safety_enabled"] is False
    assert validate(profile, tmp_path) == []


def test_unconfigured_pair_is_valid_but_remains_a_runtime_block(tmp_path):
    profile = deepcopy(load_profile())
    safety = profile["hardware"]["safety_gate"]
    safety["estop_ok_dio_channel"] = -1
    safety["local_enable_dio_channel"] = -1
    assert validate(profile, tmp_path) == []


def test_configured_distinct_flexdio_channels_are_valid(tmp_path):
    profile = deepcopy(load_profile())
    safety = profile["hardware"]["safety_gate"]
    safety["estop_ok_dio_channel"] = 10
    safety["local_enable_dio_channel"] = 11
    assert validate(profile, tmp_path) == []


@pytest.mark.parametrize(
    ("estop_channel", "enable_channel", "message"),
    [
        (-1, 10, "must both be configured"),
        (10, 10, "must be different"),
        (30, 10, "must be -1 or in [0, 29]"),
    ],
)
def test_invalid_channel_pairs_are_rejected(
    tmp_path, estop_channel, enable_channel, message
):
    profile = deepcopy(load_profile())
    safety = profile["hardware"]["safety_gate"]
    safety["estop_ok_dio_channel"] = estop_channel
    safety["local_enable_dio_channel"] = enable_channel
    assert any(message in error for error in validate(profile, tmp_path))


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("enable_debounce_ms", -1, "enable_debounce_ms must be >= 0"),
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
