#!/usr/bin/env python3
"""Read fresh minimal-runtime evidence without issuing any ROS command."""

import argparse
import json

from studica_robot_platform.minimal_health import observe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=5.0)
    args = parser.parse_args()
    ready, reason = observe(args.timeout)
    print(json.dumps({'ready_disarmed': ready, 'reason': reason,
                      'physical_qualification': 'unverified'}))
    return 0 if ready else 1


if __name__ == '__main__':
    raise SystemExit(main())
