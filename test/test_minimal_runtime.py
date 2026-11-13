"""Offline configuration and freshness failures; no ROS or service execution."""

import json
from types import SimpleNamespace as S

import pytest

from studica_robot_platform.minimal_health import MinimalHealth
from studica_robot_platform.minimal_runtime import launch_command, load_config


@pytest.fixture
def config():
    return {'schema_version': 1, 'device_id': 'classroom-01', 'domain_id': 78,
            'robot_profile': 'stack_4wd', 'use_lidar': True, 'use_camera': False}


def test_runtime_contract(tmp_path, config):
    path = tmp_path / 'minimal.json'
    path.write_text(json.dumps(config))
    command = launch_command(load_config(path))
    assert 'safety_input_cmd_vel_topic:=/cmd_vel' in command
    assert 'use_camera:=false' in command
    assert 'hardware_control_rate_hz:=25' in command
    assert 'use_joystick:=false' in command


@pytest.mark.parametrize('field,value', [
    ('domain_id', True), ('domain_id', 101), ('domain_id', 232), ('use_camera', 'false'),
    ('device_id', '../robot'), ('robot_profile', 'class_4wd'), ('extra', 'argument')])
def test_bad_config(tmp_path, config, field, value):
    config[field] = value
    path = tmp_path / 'minimal.json'
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError):
        load_config(path)


def healthy():
    health = MinimalHealth()
    motors = S(fault_latched=False, pid_supported=True, pid_type=2, motors=[
        S(motor_channel=i, encoder_fresh=True, health_level=0, feedback_age_sec=0.02,
          target_velocity_rad_s=0.0, measured_velocity_rad_s=0.0) for i in range(4)])
    odom = S(header=S(frame_id='odom'), child_frame_id='base_link',
             twist=S(twist=S(linear=S(x=0.0, y=0.0), angular=S(z=0.0))))
    health.receive('state', 'READY_DISARMED', 9.0)
    health.receive('state', 'READY_DISARMED', 10.0)
    health.receive('motors', motors, 10.0)
    health.receive('odom', odom, 10.0)
    return health, {'state': 1, 'motors': 1, 'odom': 1}


def test_readiness_positive_and_stale():
    health, publishers = healthy()
    assert health.evaluate(10.1, publishers)[0]
    assert not health.evaluate(10.6, publishers)[0]


def test_new_state_after_gap_does_not_count_old_sample():
    health, publishers = healthy()
    health.receive('state', 'READY_DISARMED', 20.0)
    assert health.state_count == 1
    assert not health.evaluate(20.0, publishers)[0]


@pytest.mark.parametrize('fault', ['armed', 'duplicate', 'nan', 'moving', 'encoder', 'latch'])
def test_readiness_failure(fault):
    health, publishers = healthy()
    motors = health.samples['motors'][0]
    if fault == 'armed':
        health.receive('state', 'ARMED', 10.0)
    elif fault == 'duplicate':
        publishers['state'] = 2
    elif fault == 'nan':
        motors.motors[0].measured_velocity_rad_s = float('nan')
    elif fault == 'moving':
        motors.motors[0].target_velocity_rad_s = 0.1
    elif fault == 'encoder':
        motors.motors[0].encoder_fresh = False
    else:
        motors.fault_latched = True
    assert not health.evaluate(10.1, publishers)[0]
