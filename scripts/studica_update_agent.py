#!/usr/bin/env python3
"""Check, inspect, or root-activate a signed Studica production update."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from studica_vmxpi_ros2.msg import PlatformStatus
from studica_robot_platform.updates import (
    UpdateError,
    extract_verified_release,
    finalize_activation,
    prepare_activation,
    read_https_json,
    recover_interrupted_activation,
    stage_update,
    staged_status,
    switch_current,
)

CURRENT_RELEASE = Path("/opt/studica/current")
RELEASES_ROOT = Path("/opt/studica/releases")
MAINTENANCE_LOCK = Path("/var/lib/studica/activation-in-progress")


def _wait_disarmed(timeout: float, request_disarm: bool) -> bool:
    rclpy.init()
    node = Node("studica_update_safety_gate")
    latest = None

    def status_callback(message):
        nonlocal latest
        latest = message

    node.create_subscription(PlatformStatus, "/robot/platform/status", status_callback, 10)
    client = node.create_client(Trigger, "/robot/disarm")
    deadline = time.monotonic() + timeout
    if request_disarm:
        while not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                node.destroy_node()
                rclpy.shutdown()
                return False
        future = client.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    safe = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (
            latest is not None
            and latest.safety_state == "READY_DISARMED"
            and not latest.armed
            and latest.mode_name == "IDLE"
            and latest.transition == "READY"
            and node.count_publishers("/robot/platform/status") == 1
        ):
            safe = True
            break
    node.destroy_node()
    rclpy.shutdown()
    return safe


def _systemctl(verb: str, unit: str) -> None:
    result = subprocess.run(
        ["/usr/bin/systemctl", verb, unit],
        check=False,
        capture_output=True,
        text=True,
        timeout=60,
    )
    if result.returncode != 0:
        raise UpdateError((result.stderr or result.stdout).strip()[:512])


def activate(version: str, state_root: Path, public_key: Path) -> None:
    if not _wait_disarmed(15.0, request_disarm=False):
        raise UpdateError("activation requires IDLE and READY_DISARMED")
    current = CURRENT_RELEASE
    releases_root = RELEASES_ROOT
    release = extract_verified_release(
        version, state_root, releases_root, public_key
    )
    previous = prepare_activation(
        version, state_root, releases_root, current, release
    )
    try:
        MAINTENANCE_LOCK.touch(mode=0o644, exist_ok=False)
        if not _wait_disarmed(15.0, request_disarm=False):
            raise UpdateError("robot left IDLE/READY_DISARMED during staging")
        _systemctl("stop", "studica-robot.target")
        switch_current(current, release)
        _systemctl("start", "studica-robot.target")
        if not _wait_disarmed(60.0, request_disarm=False):
            raise UpdateError("new release did not reach READY_DISARMED")
        finalize_activation(state_root, "ACTIVE_HEALTHY")
        MAINTENANCE_LOCK.unlink(missing_ok=True)
    except Exception as error:
        rollback_errors = []
        try:
            _systemctl("stop", "studica-robot.target")
        except UpdateError as rollback_error:
            rollback_errors.append(str(rollback_error))
        if rollback_errors:
            # Do not switch beneath a runtime that could still be running.
            raise UpdateError("could not stop failed candidate; recovery journal retained") from error
        try:
            if current.is_symlink() and current.resolve() != previous:
                switch_current(current, previous)
        except (OSError, UpdateError) as rollback_error:
            rollback_errors.append(str(rollback_error))
        if rollback_errors:
            raise UpdateError("could not restore previous pointer; recovery journal retained") from error
        try:
            _systemctl("start", "studica-robot.target")
            if not _wait_disarmed(30.0, request_disarm=False):
                raise UpdateError("previous release did not recover READY_DISARMED")
        except UpdateError as rollback_error:
            rollback_errors.append(str(rollback_error))
        if not rollback_errors:
            try:
                finalize_activation(state_root, "ROLLED_BACK_FAILED", str(error))
                MAINTENANCE_LOCK.unlink(missing_ok=True)
            except UpdateError as rollback_error:
                rollback_errors.append(str(rollback_error))
        if rollback_errors:
            raise UpdateError(
                f"activation failed: {error}; rollback errors: "
                + "; ".join(rollback_errors)
            ) from error
        raise


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--state-root", type=Path, default=Path("/var/lib/studica/updates")
    )
    parser.add_argument(
        "--public-key", type=Path, default=Path("/etc/studica/update-public.pem")
    )
    parser.add_argument(
        "--manifest-url-file",
        type=Path,
        default=Path("/etc/studica/update-manifest-url"),
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("check")
    subparsers.add_parser("status")
    subparsers.add_parser("recover")
    activate_parser = subparsers.add_parser("activate")
    activate_parser.add_argument("version")
    options = parser.parse_args()
    activation_lock = None
    try:
        if options.command in {"activate", "recover"}:
            if os.geteuid() != 0:
                raise UpdateError("activation and recovery must run as root")
            options.state_root.mkdir(parents=True, exist_ok=True)
            activation_lock = (options.state_root / "activation.lock").open("a")
            fcntl.flock(activation_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if options.command == "check":
            url = options.manifest_url_file.read_text(encoding="utf-8").strip()
            result = stage_update(
                read_https_json(url), options.public_key, options.state_root
            )
            print(json.dumps(result, indent=2, sort_keys=True))
        elif options.command == "status":
            print(json.dumps(staged_status(options.state_root), indent=2, sort_keys=True))
        elif options.command == "activate":
            if os.geteuid() != 0:
                raise UpdateError("activation must run as root")
            activate(options.version, options.state_root, options.public_key)
        else:
            if os.geteuid() != 0:
                raise UpdateError("activation recovery must run as root")
            print(
                recover_interrupted_activation(
                    options.state_root,
                    Path("/opt/studica/releases"),
                    Path("/opt/studica/current"),
                )
            )
            MAINTENANCE_LOCK.unlink(missing_ok=True)
    except (OSError, ValueError, UpdateError) as error:
        print(f"update failed: {error}", file=sys.stderr)
        return 1
    finally:
        if activation_lock is not None:
            activation_lock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
