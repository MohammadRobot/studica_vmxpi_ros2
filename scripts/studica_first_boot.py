#!/usr/bin/env python3
"""Provision a unique Studica robot identity without enabling motor services."""

import argparse
import json
from pathlib import Path

from studica_robot_platform.provisioning import provision


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/"))
    parser.add_argument("--companion-address")
    options = parser.parse_args()
    print(
        json.dumps(
            provision(options.root, options.companion_address),
            indent=2,
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
