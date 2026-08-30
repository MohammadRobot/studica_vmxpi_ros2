#!/usr/bin/env python3
"""Verify that an imported hardware workspace exactly matches hardware.repos."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

import yaml


GIT_SHA_RE = re.compile(r"[0-9a-f]{40}")


class CheckoutError(RuntimeError):
    """The imported source workspace differs from its release manifest."""


def git_output(repository: Path, arguments: list[str]) -> str:
    result = subprocess.run(
        ["git", *arguments],
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise CheckoutError(
            result.stderr.strip()
            or f"git {' '.join(arguments)} failed in {repository}"
        )
    return result.stdout.strip()


def normalize_git_url(url: str) -> str:
    return url.removesuffix("/").removesuffix(".git")


def load_repositories(manifest: Path) -> dict[str, dict[str, Any]]:
    try:
        document = yaml.safe_load(manifest.read_text(encoding="utf-8"))
        repositories = document["repositories"]
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise CheckoutError(f"cannot read {manifest}: {error}") from error
    if not isinstance(repositories, dict) or not repositories:
        raise CheckoutError(f"{manifest} has no repositories")
    return repositories


def verify_checkout(workspace: Path, manifest: Path) -> list[str]:
    workspace = workspace.resolve()
    source_dir = workspace / "src"
    repositories = load_repositories(manifest)
    verified: list[str] = []
    for name, metadata in sorted(repositories.items()):
        if not isinstance(metadata, dict) or metadata.get("type") != "git":
            raise CheckoutError(f"{name} must be a Git repository")
        expected_commit = str(metadata.get("version", ""))
        expected_url = str(metadata.get("url", ""))
        if GIT_SHA_RE.fullmatch(expected_commit) is None:
            raise CheckoutError(f"{name} is not pinned to a full Git commit")
        if not expected_url.startswith("https://"):
            raise CheckoutError(f"{name} does not use an HTTPS source URL")

        repository = source_dir / name
        if not repository.joinpath(".git").exists():
            raise CheckoutError(f"imported repository is missing: {repository}")
        actual_commit = git_output(repository, ["rev-parse", "HEAD"])
        if actual_commit != expected_commit:
            raise CheckoutError(
                f"{name} commit differs: expected {expected_commit}, found {actual_commit}"
            )
        dirty = git_output(
            repository,
            ["status", "--porcelain", "--untracked-files=all"],
        )
        if dirty:
            raise CheckoutError(f"{name} contains local or generated changes")
        actual_url = git_output(repository, ["remote", "get-url", "origin"])
        if normalize_git_url(actual_url) != normalize_git_url(expected_url):
            raise CheckoutError(
                f"{name} origin differs: expected {expected_url}, found {actual_url}"
            )
        verified.append(str(name))
    return verified


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument(
        "--manifest",
        type=Path,
        help="defaults to WORKSPACE/src/studica_vmxpi_ros2/dependencies/hardware.repos",
    )
    arguments = parser.parse_args()
    workspace = arguments.workspace.resolve()
    manifest = arguments.manifest or workspace.joinpath(
        "src/studica_vmxpi_ros2/dependencies/hardware.repos"
    )
    try:
        verified = verify_checkout(workspace, manifest.resolve())
    except (CheckoutError, OSError) as error:
        print(f"Hardware checkout verification failed: {error}", file=sys.stderr)
        return 1
    print(f"Verified {len(verified)} pinned hardware repositories: {', '.join(verified)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
