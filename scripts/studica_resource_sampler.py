#!/usr/bin/env python3
"""Record read-only host metrics; does not start a workload or contact a robot."""
from studica_robot_platform.resource_sampler import main

if __name__ == '__main__':
    raise SystemExit(main())
