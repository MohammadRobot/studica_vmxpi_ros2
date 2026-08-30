#!/usr/bin/env python3
"""Fail-closed contracts for staging, but never activating, a release."""

import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest

from test_release_bundle import COMMIT, RELEASE_VERSION, ReleaseFixture


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
INSTALLER_PATH = ROOT / "scripts" / "install_inactive_release.py"
sys.path.insert(0, str(INSTALLER_PATH.parent))
SPEC = importlib.util.spec_from_file_location(
    "install_inactive_release", INSTALLER_PATH
)
assert SPEC is not None and SPEC.loader is not None
INSTALLER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = INSTALLER
SPEC.loader.exec_module(INSTALLER)


class InactiveReleaseInstallerTest(unittest.TestCase):

    def test_verified_release_is_staged_without_activation(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.build()
            target = Path(temporary) / "target"
            target.mkdir()

            result = INSTALLER.install_inactive_release(
                fixture.output, COMMIT, target
            )

            release_root = (
                target / "opt/studica/releases" / RELEASE_VERSION
            )
            self.assertTrue(release_root.is_dir())
            self.assertEqual(result["release_version"], RELEASE_VERSION)
            self.assertFalse(result["activation_authorized"])
            self.assertFalse((target / "opt/studica/current").exists())
            self.assertFalse(
                (target / "var/lib/studica/update-state/previous-release").exists()
            )
            marker = release_root / "metadata/DO_NOT_ACTIVATE"
            self.assertIn("Development artifact only", marker.read_text())

            record_path = (
                target
                / "var/lib/studica/update-state/inactive-releases"
                / f"{RELEASE_VERSION}.json"
            )
            record = json.loads(record_path.read_text(encoding="utf-8"))
            self.assertEqual(record["profile"], INSTALLER.INSTALL_PROFILE)
            self.assertEqual(record["source_commit"], COMMIT)
            self.assertFalse(record["cryptographic_signature_verified"])
            self.assertFalse(record["current_link_observed"]["exists"])
            self.assertFalse(
                record["previous_release_state_observed"]["exists"]
            )

            status = INSTALLER.inspect_inactive_releases(target)
            self.assertFalse(status["current"]["exists"])
            self.assertEqual(len(status["installed_releases"]), 1)
            self.assertTrue(status["installed_releases"][0]["valid"])

    def test_checksummed_internal_library_symlink_is_staged_safely(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.install.joinpath("lib/libfixture-alias.so").symlink_to(
                "libstudica_drivers.so"
            )
            fixture.build()
            target = Path(temporary) / "target"
            target.mkdir()

            INSTALLER.install_inactive_release(fixture.output, COMMIT, target)

            alias = (
                target
                / "opt/studica/releases"
                / RELEASE_VERSION
                / "install/lib/libfixture-alias.so"
            )
            self.assertTrue(alias.is_symlink())
            self.assertEqual(os.readlink(alias), "libstudica_drivers.so")

    def test_existing_current_and_previous_state_are_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.build()
            target = Path(temporary) / "target"
            known_good = target / "opt/studica/releases/known-good"
            known_good.mkdir(parents=True)
            current = target / "opt/studica/current"
            current.symlink_to("releases/known-good")
            previous = target / "var/lib/studica/update-state/previous-release"
            previous.parent.mkdir(parents=True)
            previous.write_text("known-good\n", encoding="utf-8")
            current_before = os.lstat(current)
            previous_before = previous.read_bytes()

            INSTALLER.install_inactive_release(fixture.output, COMMIT, target)

            current_after = os.lstat(current)
            self.assertEqual(os.readlink(current), "releases/known-good")
            self.assertEqual(current_before.st_ino, current_after.st_ino)
            self.assertEqual(previous.read_bytes(), previous_before)

    def test_bad_checksum_is_rejected_before_release_directory_exists(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            _, checksum = fixture.build()
            checksum.write_text(
                f"{'0' * 64}  {checksum.name.removesuffix('.sha256')}\n",
                encoding="utf-8",
            )
            target = Path(temporary) / "target"
            target.mkdir()

            with self.assertRaisesRegex(
                INSTALLER.InstallationError, "verification failed"
            ):
                INSTALLER.install_inactive_release(fixture.output, COMMIT, target)

            releases = target / "opt/studica/releases"
            self.assertEqual(list(releases.iterdir()), [])
            self.assertFalse((target / "opt/studica/current").exists())

    def test_wrong_commit_and_existing_destination_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.build()
            wrong_target = Path(temporary) / "wrong-target"
            wrong_target.mkdir()
            with self.assertRaisesRegex(
                INSTALLER.InstallationError, "does not match"
            ):
                INSTALLER.install_inactive_release(
                    fixture.output, "c" * 40, wrong_target
                )

            target = Path(temporary) / "target"
            target.mkdir()
            INSTALLER.install_inactive_release(fixture.output, COMMIT, target)
            with self.assertRaisesRegex(
                INSTALLER.InstallationError, "already exists"
            ):
                INSTALLER.install_inactive_release(fixture.output, COMMIT, target)

    def test_symlinked_managed_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.build()
            target = Path(temporary) / "target"
            outside = Path(temporary) / "outside"
            target.joinpath("opt").mkdir(parents=True)
            outside.mkdir()
            target.joinpath("opt/studica").symlink_to(outside)

            with self.assertRaisesRegex(
                INSTALLER.InstallationError, "not a real directory"
            ):
                INSTALLER.install_inactive_release(fixture.output, COMMIT, target)
            self.assertEqual(list(outside.iterdir()), [])

    def test_machine_readable_contract_preserves_activation_pointers(self):
        contract = json.loads(
            ROOT.joinpath(
                "deployment/inactive-release-install-v1.json"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(contract["profile"], INSTALLER.INSTALL_PROFILE)
        self.assertFalse(
            contract["accepted_artifact"]["cryptographic_signature_required"]
        )
        self.assertFalse(
            contract["accepted_artifact"]["activation_authorized"]
        )
        self.assertEqual(
            set(contract["preserved_paths"]),
            {
                "/opt/studica/current",
                "/var/lib/studica/update-state/previous-release",
            },
        )


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
