"""Explicit minimal-runtime configuration; validation never starts ROS."""

import argparse
import json
import os
from pathlib import Path
import platform
import re


CONFIG_PATH = '/etc/studica/minimal.json'


def load_config(path):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    required = {'schema_version', 'device_id', 'domain_id', 'robot_profile',
                'use_lidar', 'use_camera'}
    if not isinstance(config, dict) or set(config) != required:
        raise ValueError('minimal config must contain exactly the documented fields')
    if type(config['schema_version']) is not int or config['schema_version'] != 1:
        raise ValueError('unsupported minimal config schema')
    if not isinstance(config['device_id'], str) or not re.fullmatch(
            r'[a-z][a-z0-9_-]{0,63}', config['device_id']):
        raise ValueError('device_id must be a lowercase device identifier')
    if type(config['domain_id']) is not int or not 0 <= config['domain_id'] <= 100:
        raise ValueError('domain_id must be an integer from 0 to 100')
    if config['robot_profile'] != 'stack_4wd':
        raise ValueError('minimal runtime currently supports only stack_4wd')
    if any(type(config[name]) is not bool for name in ('use_lidar', 'use_camera')):
        raise ValueError('sensor selections must be JSON booleans')
    return config


def launch_command(config):
    return ['ros2', 'launch', 'studica_vmxpi_ros2', 'robot.launch.py',
            'robot_profile:=stack_4wd', 'hardware_control_rate_hz:=25',
            'use_lidar:=' + str(config['use_lidar']).lower(),
            'use_camera:=' + str(config['use_camera']).lower(),
            'use_joystick:=false', 'control_source:=application',
            'safety_input_cmd_vel_topic:=/cmd_vel']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=CONFIG_PATH)
    parser.add_argument('--check', action='store_true', help='validate without starting hardware')
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        command = launch_command(config)
        if args.check:
            print(json.dumps({'device_id': config['device_id'],
                              'domain_id': config['domain_id'], 'command': command}))
            return 0
        if platform.machine() not in ('aarch64', 'arm64'):
            raise ValueError('hardware runtime requires native ARM64')
        if os.environ.get('ROS_DISTRO') != 'humble':
            raise ValueError('hardware runtime requires the explicit Humble underlay')
        if os.environ.get('ROS_DOMAIN_ID') != str(config['domain_id']):
            raise ValueError('ROS_DOMAIN_ID must match minimal.json domain_id')
        os.environ['ROS_DOMAIN_ID'] = str(config['domain_id'])
        os.environ['ROS_LOCALHOST_ONLY'] = '0'
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        # No publisher, service call, software arm, or extra launch arguments here.
        os.execvp(command[0], command)
    except (OSError, ValueError) as error:
        parser.exit(2, f'minimal runtime: {error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
