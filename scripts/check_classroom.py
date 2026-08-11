#!/usr/bin/env python3
"""Static classroom contracts that must not start ROS nodes or robot motion."""

from __future__ import annotations

import argparse
from pathlib import Path
import py_compile
import subprocess
import xml.etree.ElementTree as ET

import yaml


GZ_CONTROL_COMMIT = "a2d290e37be67ba082744e323339d82031f051c0"
PUBLIC_LAUNCHES = {
    "bringup.launch.py",
    "mapping.launch.py",
    "navigation.launch.py",
    "robot.launch.py",
    "sim.launch.py",
}
BACKUP_SUFFIXES = (".bak", ".orig", ".rej", ".save", "~")


def fail_if(condition: bool, message: str, failures: list[str]) -> None:
    if condition:
        failures.append(message)


def tracked_files(root: Path) -> list[Path]:
    if not root.joinpath(".git").exists():
        return []
    result = subprocess.run(
        ["git", "ls-files", "-z"], cwd=root, check=True, capture_output=True
    )
    return [root / name.decode() for name in result.stdout.split(b"\0") if name]


def check_structured_files(root: Path, failures: list[str]) -> None:
    yaml_paths = sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix in {".yaml", ".yml", ".repos"}
    )
    for path in yaml_paths:
        try:
            yaml.safe_load(path.read_text(encoding="utf-8"))
        except Exception as error:  # PyYAML provides useful, varied exception types.
            failures.append(f"invalid YAML {path.relative_to(root)}: {error}")

    xml_paths = sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix in {".xml", ".xacro"}
    )
    for path in xml_paths:
        try:
            ET.parse(path)
        except ET.ParseError as error:
            failures.append(f"invalid XML {path.relative_to(root)}: {error}")


def check_python(root: Path, failures: list[str]) -> None:
    roots = [root / "bringup" / "launch", root / "scripts", root / "examples" / "python"]
    for directory in roots:
        if not directory.exists():
            continue
        for path in sorted(directory.rglob("*.py")):
            try:
                py_compile.compile(str(path), doraise=True)
            except py_compile.PyCompileError as error:
                failures.append(f"invalid Python {path.relative_to(root)}: {error.msg}")


def check_repository_hygiene(root: Path, failures: list[str]) -> None:
    for path in tracked_files(root):
        relative = path.relative_to(root)
        parts = relative.parts
        fail_if(
            any(part in {"__pycache__", ".pytest_cache"} for part in parts),
            f"tracked cache: {relative}",
            failures,
        )
        fail_if(
            relative.name.endswith(BACKUP_SUFFIXES) or relative.suffix == ".pyc",
            f"tracked backup/build artifact: {relative}",
            failures,
        )
        if path.is_file():
            try:
                is_elf = path.read_bytes()[:4] == b"\x7fELF"
            except OSError as error:
                failures.append(f"cannot inspect tracked file {relative}: {error}")
                continue
            fail_if(is_elf, f"tracked ELF binary: {relative}", failures)


def check_public_contract(root: Path, failures: list[str]) -> None:
    launch_dir = root / "bringup" / "launch"
    public = {
        path.name for path in launch_dir.glob("*.launch.py") if not path.name.startswith("_")
    }
    fail_if(public != PUBLIC_LAUNCHES, f"unexpected public launches: {sorted(public)}", failures)

    readme_lines = root.joinpath("README.md").read_text(encoding="utf-8").splitlines()
    fail_if(
        not 150 <= len(readme_lines) <= 250,
        f"README must contain 150-250 lines (found {len(readme_lines)})",
        failures,
    )

    profiles = root / "bringup" / "config" / "profiles"
    fail_if(not profiles.joinpath("class_4wd").is_dir(), "class_4wd profile is missing", failures)
    fail_if(profiles.joinpath("training_2wd").exists(), "training_2wd was not removed", failures)
    fail_if(profiles.joinpath("training_4wd").exists(), "training_4wd was not removed", failures)

    bringup = launch_dir.joinpath("bringup.launch.py").read_text(encoding="utf-8")
    runtime = launch_dir.joinpath("_robot_runtime.launch.py").read_text(encoding="utf-8")
    fail_if('default_value="class_4wd"' not in bringup and '"class_4wd"' not in bringup,
            "advanced bringup does not default to class_4wd", failures)
    for marker in ('"input_cmd_vel_topic": "/cmd_vel"', '"output_odom_topic": "/odom"'):
        fail_if(marker not in runtime, f"runtime is missing standard API marker {marker}", failures)

    simulation = yaml.safe_load(root.joinpath("dependencies/simulation.repos").read_text())
    pinned = simulation["repositories"]["gz_ros2_control"]["version"]
    fail_if(pinned != GZ_CONTROL_COMMIT, "gz_ros2_control overlay pin changed", failures)

    public_docs = [root / "README.md", *root.joinpath("docs", "labs").glob("*.md")]
    for path in public_docs:
        if path.is_file():
            fail_if(
                "autotune" in path.read_text(encoding="utf-8").lower(),
                f"Titan autotune leaked into public documentation: {path.relative_to(root)}",
                failures,
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1]
    )
    arguments = parser.parse_args()
    root = arguments.root.resolve()
    failures: list[str] = []

    check_python(root, failures)
    check_structured_files(root, failures)
    check_repository_hygiene(root, failures)
    check_public_contract(root, failures)

    if failures:
        print("Classroom contract check failed:")
        for failure in failures:
            print(f"  - {failure}")
        return 1
    print("[check] Classroom contracts: Python, YAML/XML, API, and repository hygiene")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
