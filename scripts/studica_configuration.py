#!/usr/bin/env python3
"""Export a validated offline configuration into a new directory; no runtime mutation."""
import argparse
import json
from pathlib import Path
import shutil
import tempfile

import yaml
from studica_robot_platform.configuration import compose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile-root', type=Path, required=True)
    parser.add_argument('--device', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    validator = Path(get_package_share_directory('studica_vmxpi_ros2')) / 'launch/profile_validation.py'
    try:
        if args.output.exists():
            raise ValueError('output must be a new directory; existing configuration is preserved')
        inputs, metadata = compose(args.profile_root, json.loads(args.device.read_text()),
                                   json.loads(args.calibration.read_text()), validator)
        # Compose first; copytree creates the destination exclusively, refusing
        # even an empty existing directory. A failed copy is not an activation.
        with tempfile.TemporaryDirectory(dir=args.output.parent) as temporary:
            staging = Path(temporary) / 'export'
            profile = staging / 'profiles' / metadata['profile_id']
            profile.mkdir(parents=True)
            for name, data in [('robot_profile', inputs['profile']),
                               ('robot_controllers', inputs['controllers'])]:
                (profile / (name + '.yaml')).write_text(yaml.safe_dump(data, sort_keys=True))
            for name, data in [('device', inputs['device']), ('calibration', inputs['calibration']),
                               ('robot-configuration', metadata)]:
                (staging / (name + '.json')).write_text(json.dumps(data, indent=2) + '\n')
            shutil.copytree(staging, args.output)
        print(f'Offline configuration exported: {args.output}; hardware qualification pending')
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
        parser.exit(2, f'configuration export rejected: {error}\n')


if __name__ == '__main__':
    main()
