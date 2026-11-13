"""Offline composition: no ROS graph or hardware access."""
from copy import deepcopy
import json
from pathlib import Path

import pytest
from studica_robot_platform.configuration import compose

ROOT = Path(__file__).resolve().parents[1]


def inputs(profile='stack_4wd'):
    device = json.loads((ROOT / 'contracts/robot-configuration-v1/device.example.json').read_text())
    calibration = json.loads((ROOT / 'contracts/robot-configuration-v1/calibration.example.json').read_text())
    device['robot_profile'] = calibration['robot_profile'] = profile
    return device, calibration


def export(device, calibration):
    return compose(ROOT / 'bringup/config/profiles', device, calibration,
                   ROOT / 'bringup/launch/profile_validation.py')


@pytest.mark.parametrize('profile,count', [
    ('class_2wd', 2), ('class_4wd', 4), ('stack_4wd', 4), ('class_mecanum', 4), ('class_omni', 4)])
def test_profiles(profile, count):
    device, calibration = inputs(profile)
    _, result = export(device, calibration)
    assert len(result['drive']['motors']) == len(result['drive']['joints']) == count
    assert result['motion_authorized'] is False
    assert result['qualification'] == 'unqualified'
    assert result['software']['runtime_revision'] is None


def test_digest_stability_and_sensitivity():
    device, calibration = inputs()
    _, original = export(device, calibration)
    assert export(dict(reversed(list(device.items()))), calibration)[1] == original
    calibration['controller_corrections']['left_wheel_radius_multiplier'] = 1.01
    effective, changed = export(device, calibration)
    assert changed['configuration_sha256'] != original['configuration_sha256']
    assert changed['profile_sha256'] == original['profile_sha256']
    assert effective['controllers']['robot_base_controller']['ros__parameters'][
        'left_wheel_radius_multiplier'] == 1.01
    assert export(*inputs())[1] == original  # Source profile was not rewritten.


@pytest.mark.parametrize('key,value', [
    ('domain_id', True), ('domain_id', 101), ('robot_profile', '../stack_4wd'),
    ('use_lidar', 'yes'), ('environment', 'unknown')])
def test_invalid_device(key, value):
    device, calibration = inputs()
    device[key] = value
    with pytest.raises(ValueError):
        export(device, calibration)


@pytest.mark.parametrize('correction', [
    {'cmd_vel_timeout': 5}, {'left_wheel_radius_multiplier': float('nan')},
    {'left_wheel_radius_multiplier': True}, {'left_wheel_radius_multiplier': 0.1}])
def test_calibration_cannot_change_safety_or_use_invalid_values(correction):
    device, calibration = inputs()
    calibration['controller_corrections'] = correction
    with pytest.raises(ValueError):
        export(device, calibration)


def test_wrong_device_and_holonomic_corrections_rejected():
    device, calibration = inputs()
    other = deepcopy(calibration)
    other['device_id'] = 'another-robot'
    with pytest.raises(ValueError):
        export(device, other)
    device, calibration = inputs('class_mecanum')
    calibration['controller_corrections'] = {'left_wheel_radius_multiplier': 1.0}
    with pytest.raises(ValueError):
        export(device, calibration)


@pytest.mark.parametrize('name', ['class_2wd', 'stack_4wd'])
def test_handoff_fixtures_match_generator(name):
    device, calibration = inputs(name)
    if name == 'class_2wd':
        device['environment'] = 'simulation'
    expected = json.loads((ROOT / 'contracts/robot-configuration-v1' / (name + '.fixture.json')).read_text())
    assert export(device, calibration)[1] == expected
