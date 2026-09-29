#!/usr/bin/env python3
"""Install platform policy files into an offline root; autostart is gated."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory

from studica_robot_platform.provisioning import (
    canonical_root,
    provision,
    root_path,
    validate_qualification,
    configure_domain,
    validate_domain_id,
)


def _copy_tree(
    source: Path, destination: Path, excluded_names: frozenset[str] = frozenset(),
    preserve_existing: bool = False,
) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    for item in source.iterdir():
        if item.name in excluded_names:
            continue
        target = destination / item.name
        if item.is_dir():
            _copy_tree(item, target, excluded_names, preserve_existing)
        else:
            if preserve_existing and target.exists():
                continue
            shutil.copyfile(item, target)
            os.chmod(target, item.stat().st_mode & 0o777)


def _ensure_users(root: Path) -> None:
    if root != Path("/"):
        return
    definitions = (
        ("studica", ["dialout", "input", "video"]),
        ("studica-update", []),
    )
    for user, groups in definitions:
        exists = subprocess.run(
            ["/usr/bin/getent", "passwd", user],
            check=False,
            stdout=subprocess.DEVNULL,
        ).returncode == 0
        if exists:
            continue
        arguments = [
            "/usr/sbin/useradd",
            "--system",
            "--home-dir",
            "/var/lib/studica",
            "--shell",
            "/usr/sbin/nologin",
        ]
        if groups:
            arguments.extend(["--groups", ",".join(groups)])
        subprocess.run([*arguments, user], check=True)


def _assign_runtime_ownership(root: Path) -> None:
    if root != Path("/"):
        return
    import pwd

    studica = pwd.getpwnam("studica")
    updater = pwd.getpwnam("studica-update")
    for relative in ("maps", "pairing", "support", "secrets", "tls"):
        path = Path("/var/lib/studica") / relative
        for current, directories, files in os.walk(path):
            os.chown(current, studica.pw_uid, studica.pw_gid)
            for name in directories + files:
                os.chown(Path(current) / name, studica.pw_uid, studica.pw_gid)
    updates = Path("/var/lib/studica/updates")
    for current, directories, files in os.walk(updates):
        os.chown(current, updater.pw_uid, updater.pw_gid)
        for name in directories + files:
            os.chown(Path(current) / name, updater.pw_uid, updater.pw_gid)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path("/"))
    parser.add_argument("--assets-root", type=Path)
    parser.add_argument("--companion-address")
    parser.add_argument("--domain-id", type=int, help="unique classroom domain (0..100)")
    parser.add_argument("--enable-autostart", action="store_true")
    parser.add_argument("--safety-qualification", type=Path)
    parser.add_argument("--update-public-key", type=Path)
    parser.add_argument("--update-manifest-url")
    options = parser.parse_args()
    root = canonical_root(options.root)
    if options.domain_id is not None:
        validate_domain_id(options.domain_id)
    if (options.update_public_key is None) != (options.update_manifest_url is None):
        raise SystemExit(
            "update public key and manifest URL must be configured together"
        )
    if options.update_manifest_url and not options.update_manifest_url.startswith(
        "https://"
    ):
        raise SystemExit("update manifest URL must use HTTPS")
    _ensure_users(root)
    share = Path(get_package_share_directory("studica_vmxpi_ros2"))
    assets = options.assets_root or share / "deployment"
    if not (assets / "systemd/studica-robot.target").is_file():
        raise SystemExit(f"platform deployment assets are missing: {assets}")

    _copy_tree(
        assets / "systemd",
        root_path(root, "/etc/systemd/system"),
        frozenset({"studica-companion.service"}),
    )
    _copy_tree(assets / "config", root_path(root, "/etc/studica"), preserve_existing=True)
    if options.domain_id is not None:
        configure_domain(root, options.domain_id)
    _copy_tree(share / "config/profiles/stack_4wd",
               root_path(root, "/etc/studica/profiles/stack_4wd"), preserve_existing=True)
    journal_destination = root_path(
        root, "/etc/systemd/journald.conf.d/studica.conf"
    )
    journal_destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(
        assets / "config/journald-studica.conf",
        journal_destination,
    )
    resolved_destination = root_path(
        root, "/etc/systemd/resolved.conf.d/studica-mdns.conf"
    )
    resolved_destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(
        assets / "config/resolved-mdns.conf",
        resolved_destination,
    )
    sshd_destination = root_path(
        root, "/etc/ssh/sshd_config.d/90-studica.conf"
    )
    sshd_destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(
        assets / "config/sshd-studica.conf",
        sshd_destination,
    )
    os.chmod(sshd_destination, 0o644)
    provision(root, options.companion_address)
    if options.update_public_key:
        shutil.copyfile(
            options.update_public_key,
            root_path(root, "/etc/studica/update-public.pem"),
        )
        root_path(root, "/etc/studica/update-manifest-url").write_text(
            options.update_manifest_url + "\n", encoding="utf-8"
        )
    _assign_runtime_ownership(root)

    if options.enable_autostart:
        if options.safety_qualification is None:
            raise SystemExit(
                "--enable-autostart requires --safety-qualification after hardware testing"
            )
        validate_qualification(options.safety_qualification)
        if options.update_public_key is None:
            raise SystemExit(
                "production autostart requires the publisher update public key and manifest URL"
            )
        wants = root_path(root, "/etc/systemd/system/multi-user.target.wants")
        wants.mkdir(parents=True, exist_ok=True)
        link = wants / "studica-robot.target"
        link.unlink(missing_ok=True)
        link.symlink_to("../studica-robot.target")
        timers = root_path(root, "/etc/systemd/system/timers.target.wants")
        timers.mkdir(parents=True, exist_ok=True)
        timer_link = timers / "studica-update.timer"
        timer_link.unlink(missing_ok=True)
        timer_link.symlink_to("../studica-update.timer")
        print("installed and enabled studica-robot.target")
    else:
        print("installed locally; autostart remains disabled pending safety qualification")


if __name__ == "__main__":
    main()
