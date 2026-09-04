#!/usr/bin/env python3
"""Create one bounded redacted support archive."""

import argparse
from pathlib import Path

from studica_robot_platform.support import create_bundle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-root", type=Path, default=Path("/var/lib/studica/support")
    )
    options = parser.parse_args()
    print(create_bundle(options.output_root))


if __name__ == "__main__":
    main()
