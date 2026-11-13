"""Offline device/profile composition. Never starts ROS or authorizes hardware."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import tempfile

import yaml


CONTRACT = 'studica.robot-configuration/1'
SLOTS = ('left_front', 'left_rear', 'right_front', 'right_rear')


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def exact(value, keys, label):
    if not isinstance(value, dict) or set(value) != set(keys):
        raise ValueError(f'{label}: unexpected or missing fields')


def compose(profile_root, device, calibration, validator_path):
    """Return effective files and proposal metadata from explicit, bounded inputs."""
    exact(device, ['schema_version', 'device_id', 'domain_id', 'robot_profile',
                   'environment', 'use_lidar', 'use_camera'], 'device')
    if type(device['schema_version']) is not int or device['schema_version'] != 1:
        raise ValueError('unsupported device schema')
    for key in ('device_id', 'robot_profile'):
        if not isinstance(device[key], str) or not re.fullmatch(r'[a-z][a-z0-9_-]{0,63}', device[key]):
            raise ValueError(f'invalid {key}')
    if type(device['domain_id']) is not int or not 0 <= device['domain_id'] <= 100:
        raise ValueError('invalid domain_id')
    if device['environment'] not in ('simulation', 'hardware'):
        raise ValueError('invalid environment')
    if any(type(device[k]) is not bool for k in ('use_lidar', 'use_camera')):
        raise ValueError('sensor selections must be boolean')
    exact(calibration, ['schema_version', 'device_id', 'robot_profile', 'controller_corrections'],
          'calibration')
    if type(calibration['schema_version']) is not int or calibration['schema_version'] != 1:
        raise ValueError('unsupported calibration schema')
    if any(calibration[k] != device[k] for k in ('device_id', 'robot_profile')):
        raise ValueError('calibration belongs to another device/profile')
    corrections = calibration['controller_corrections']
    allowed = {'left_wheel_radius_multiplier', 'right_wheel_radius_multiplier',
               'wheel_separation_multiplier'}
    if not isinstance(corrections, dict) or not set(corrections) <= allowed:
        raise ValueError('unsupported calibration correction')
    for value in corrections.values():
        if type(value) not in (int, float) or not math.isfinite(value) or not 0.5 <= value <= 2.0:
            raise ValueError('correction must be finite and within review bounds [0.5, 2]')

    name = device['robot_profile']
    directory = Path(profile_root).resolve() / name
    directory.resolve().relative_to(Path(profile_root).resolve())
    profile = yaml.safe_load((directory / 'robot_profile.yaml').read_text())
    controllers = yaml.safe_load((directory / 'robot_controllers.yaml').read_text())
    # Reuse launch's authoritative validator; do not implement a second profile schema.
    spec = importlib.util.spec_from_file_location('studica_profile_validation', validator_path)
    validator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(validator)
    with tempfile.TemporaryDirectory() as temporary:
        effective = Path(temporary) / name
        effective.mkdir()
        (effective / 'robot_profile.yaml').write_text(yaml.safe_dump(profile))
        (effective / 'robot_controllers.yaml').write_text(yaml.safe_dump(controllers))
        errors, controller, controller_type, layout = validator.validate_profile_directory(effective)
        if errors:
            raise ValueError('\n'.join(errors))
        if corrections and controller_type != 'diff_drive_controller/DiffDriveController':
            raise ValueError('wheel multiplier calibration is differential-drive only')
        parameters = controllers[controller]['ros__parameters']
        parameters.update(corrections)
        (effective / 'robot_controllers.yaml').write_text(yaml.safe_dump(controllers))
        errors, _, _, _ = validator.validate_profile_directory(effective)
        if errors:
            raise ValueError('\n'.join(errors))
    if layout in ('diff', 'diff_4wd'):
        joints = parameters['left_wheel_names'] + parameters['right_wheel_names']
    else:
        joints = [parameters[k + '_wheel_command_joint_name']
                  for k in ('front_left', 'rear_left', 'front_right', 'rear_right')]
    motors = [{'slot': slot, 'channel': profile['hardware'][slot + '_motor']}
              for slot in SLOTS if profile['hardware'][slot + '_motor'] >= 0]
    if len(motors) != len(joints):
        raise ValueError('motor mapping and controller joint count differ')
    if any(not isinstance(joint, str) or not joint for joint in joints) or len(set(joints)) != len(joints):
        raise ValueError('controller joints must be unique nonempty names')
    inputs = dict(device=device, calibration=calibration, profile=profile, controllers=controllers)
    metadata = dict(
        schema=CONTRACT, evidence_scope='offline_configuration',
        device_id=device['device_id'], domain_id=device['domain_id'],
        environment=device['environment'], profile_id=name,
        configuration_sha256=digest(inputs), profile_sha256=digest(profile),
        calibration_sha256=digest(calibration),
        software=dict(ros_distro='humble', runtime_revision=None, installed_artifact_sha256=None),
        drive=dict(layout=layout, controller_type=controller_type, joints=joints, motors=motors),
        sensors=dict(imu='configured', lidar='configured' if device['use_lidar'] else 'disabled',
                     camera='configured' if device['use_camera'] else 'disabled'),
        interfaces={'/cmd_vel': 'geometry_msgs/msg/Twist', '/odom': 'nav_msgs/msg/Odometry',
                    '/imu': 'sensor_msgs/msg/Imu', '/scan': 'sensor_msgs/msg/LaserScan'},
        clock='simulation' if device['environment'] == 'simulation' else 'system',
        qualification='unqualified', motion_authorized=False,
    )
    return inputs, metadata
