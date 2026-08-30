#!/usr/bin/env python3
"""Contracts for the isolated, development-only ARM64 release builder."""

from copy import deepcopy
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


VALIDATOR = load_module(
    "validate_arm64_builder",
    ROOT / "scripts/validate_arm64_builder.py",
)
CHECKOUT = load_module(
    "verify_hardware_checkout",
    ROOT / "scripts/verify_hardware_checkout.py",
)
MANIFEST_PATH = ROOT / "deployment/arm64-builder-v1.json"


def git(repository, *arguments):
    result = subprocess.run(
        ["git", *arguments],
        cwd=repository,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise AssertionError(result.stdout + result.stderr)
    return result.stdout.strip()


class Arm64BuilderTest(unittest.TestCase):

    def setUp(self):
        self.manifest = VALIDATOR.read_json(MANIFEST_PATH)

    def test_checked_in_builder_contract_passes(self):
        self.assertEqual(VALIDATOR.validate_builder(ROOT, self.manifest), [])

    def test_floating_base_or_activation_is_rejected(self):
        changed = deepcopy(self.manifest)
        changed["base_image"]["reference"] = "ubuntu:22.04"
        changed["artifact_policy"]["activation_authorized"] = True
        failures = VALIDATOR.validate_builder(ROOT, changed)
        self.assertTrue(any("immutable digest" in failure for failure in failures))
        self.assertTrue(any("non-activatable" in failure for failure in failures))

    def test_host_builder_help_does_not_access_docker(self):
        result = subprocess.run(
            [str(ROOT / "scripts/build_arm64_release.sh"), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--allow-emulation", result.stdout)
        self.assertIn("never connects to the robot", result.stdout)

    def test_imported_hardware_checkout_must_match_commit_origin_and_cleanliness(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary) / "workspace"
            repository = workspace / "src/example_driver"
            repository.mkdir(parents=True)
            git(repository, "init", "--initial-branch=main")
            git(repository, "config", "user.name", "Studica Test")
            git(repository, "config", "user.email", "test@example.com")
            repository.joinpath("README.md").write_text(
                "fixture\n", encoding="utf-8"
            )
            git(repository, "add", "README.md")
            git(repository, "commit", "-m", "fixture")
            git(repository, "remote", "add", "origin", "https://example.com/example.git")
            commit = git(repository, "rev-parse", "HEAD")
            manifest = Path(temporary) / "hardware.repos"
            manifest.write_text(
                yaml.safe_dump(
                    {
                        "repositories": {
                            "example_driver": {
                                "type": "git",
                                "url": "https://example.com/example.git",
                                "version": commit,
                            }
                        }
                    },
                    sort_keys=True,
                ),
                encoding="utf-8",
            )
            self.assertEqual(
                CHECKOUT.verify_checkout(workspace, manifest),
                ["example_driver"],
            )

            repository.joinpath("README.md").write_text(
                "dirty fixture\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(CHECKOUT.CheckoutError, "local or generated"):
                CHECKOUT.verify_checkout(workspace, manifest)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
