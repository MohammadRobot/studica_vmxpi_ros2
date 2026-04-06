#!/usr/bin/env python3
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
"""Validate all robot profile YAML files."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _prepare_profile_validation_import() -> None:
    """Locate bringup/launch/profile_validation.py for source and install layouts."""
    script_dir = Path(__file__).resolve().parent
    candidate_dirs = [script_dir.parent / "bringup" / "launch"]

    try:
        from ament_index_python.packages import get_package_share_directory

        candidate_dirs.append(
            Path(get_package_share_directory("studica_vmxpi_ros2")) / "launch"
        )
    except Exception:  # pragma: no cover
        # Source-tree execution is the common path for this script.
        pass

    for candidate in candidate_dirs:
        module_file = candidate / "profile_validation.py"
        if module_file.exists():
            sys.path.insert(0, str(candidate))
            return

    searched = "\n  - ".join(str(path) for path in candidate_dirs)
    raise RuntimeError(
        "Could not locate profile_validation.py. Searched:\n"
        f"  - {searched}"
    )


_prepare_profile_validation_import()


def _parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_profiles_dir = script_dir.parent / "bringup" / "config" / "profiles"
    default_template_dir = script_dir.parent / "bringup" / "config" / "profile_template"

    if not default_profiles_dir.exists():
        try:
            from ament_index_python.packages import get_package_share_directory

            package_share = Path(get_package_share_directory("studica_vmxpi_ros2"))
            default_profiles_dir = package_share / "config" / "profiles"
            default_template_dir = package_share / "config" / "profile_template"
        except Exception:  # pragma: no cover
            pass

    parser = argparse.ArgumentParser(description="Validate all robot profile YAML files.")
    parser.add_argument(
        "--profiles-dir",
        default=str(default_profiles_dir),
        help="Path to profiles directory (default: %(default)s)",
    )
    parser.add_argument(
        "--template-dir",
        default=str(default_template_dir),
        help="Path to template directory (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> int:
    from profile_validation import validate_profile_directory

    args = _parse_args()
    profiles_dir = Path(args.profiles_dir).resolve()
    template_dir = Path(args.template_dir).resolve()

    if not profiles_dir.exists():
        print(f"ERROR: profiles directory does not exist: {profiles_dir}", file=sys.stderr)
        return 2
    if not profiles_dir.is_dir():
        print(f"ERROR: profiles path is not a directory: {profiles_dir}", file=sys.stderr)
        return 2

    profile_dirs = sorted(
        profile_dir
        for profile_dir in profiles_dir.iterdir()
        if profile_dir.is_dir() and not profile_dir.name.startswith(".")
    )
    if not profile_dirs:
        print(f"ERROR: no profile directories found in {profiles_dir}", file=sys.stderr)
        return 2

    if template_dir.exists():
        if not template_dir.is_dir():
            print(
                f"ERROR: template path is not a directory: {template_dir}",
                file=sys.stderr,
            )
            return 2
        profile_dirs.append(template_dir)
    else:
        print(f"WARNING: template directory not found: {template_dir}", file=sys.stderr)

    all_errors: list[str] = []
    for profile_dir in profile_dirs:
        errors, _, _, _ = validate_profile_directory(profile_dir)
        if errors:
            all_errors.extend(errors)
            print(f"[FAIL] {profile_dir.name}")
        else:
            print(f"[OK]   {profile_dir.name}")

    if all_errors:
        print("\nProfile validation failed with the following errors:", file=sys.stderr)
        for error in all_errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print(f"\nValidated {len(profile_dirs)} profile(s) successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
