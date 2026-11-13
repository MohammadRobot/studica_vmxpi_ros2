#!/usr/bin/env python3
"""Contracts for post-build development release artifact verification."""

from copy import deepcopy
from unittest.mock import patch
import io
import importlib.util
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

from test_release_bundle import BUILDER, COMMIT, ReleaseFixture


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
VERIFIER_PATH = ROOT / "scripts" / "verify_release_artifacts.py"
SPEC = importlib.util.spec_from_file_location("verify_release_artifacts", VERIFIER_PATH)
assert SPEC is not None and SPEC.loader is not None
VERIFIER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = VERIFIER
SPEC.loader.exec_module(VERIFIER)


class ReleaseArtifactVerificationTest(unittest.TestCase):

    @staticmethod
    def fixture_inventory(install, sdk):
        # Synthetic bundle fixtures are headers, not executable ARM binaries.
        # Actual ELF parsing is exercised independently in test_runtime_libraries.
        del sdk
        return {
            'schema_version': 1,
            'evidence_scope': "static_elf_dependency_availability",
            'elf_machine': 183, 'runtime_tested': False, 'activation_authorized': False,
            'artifacts': {
                "install/" + name: {
                    'sha256': BUILDER.sha256_file(install / name),
                    'needed': {}, 'search_paths': [],
                }
                for name in BUILDER.REQUIRED_ARM64_FILES
            },
        }

    def test_native_release_round_trip(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            with patch.object(BUILDER.platform, "machine", return_value="aarch64"), \
                    patch.object(BUILDER.platform, "freedesktop_os_release",
                                 return_value={"ID": "ubuntu", "VERSION_ID": "22.04"}), \
                    patch.object(BUILDER, "native_runtime_inventory", side_effect=self.fixture_inventory):
                fixture.build(builder_image_id="", native=True)
            VERIFIER.verify_release_artifacts(fixture.output, expected_commit=COMMIT)

    def test_native_inventory_cannot_omit_files_or_claim_runtime_acceptance(self):
        for fault in ("missing", "hash", "qualified"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as temporary:
                fixture = ReleaseFixture(temporary)

                def inventory(install, sdk):
                    value = self.fixture_inventory(install, sdk)
                    key = next(iter(value["artifacts"]))
                    if fault == "missing":
                        del value["artifacts"][key]
                    elif fault == "hash":
                        value["artifacts"][key]["sha256"] = "0" * 64
                    else:
                        value["runtime_tested"] = True
                    return value

                with patch.object(BUILDER.platform, "machine", return_value="aarch64"), \
                        patch.object(BUILDER.platform, "freedesktop_os_release",
                                     return_value={"ID": "ubuntu", "VERSION_ID": "22.04"}), \
                        patch.object(BUILDER, "native_runtime_inventory", side_effect=inventory):
                    fixture.build(builder_image_id="", native=True)
                with self.assertRaises(VERIFIER.VerificationError):
                    VERIFIER.verify_release_artifacts(fixture.output)

    def test_native_rejects_pc_packaging(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            with patch.object(BUILDER.platform, "machine", return_value="x86_64"):
                with self.assertRaisesRegex(BUILDER.BundleError, "native packaging"):
                    fixture.build(builder_image_id="", native=True)

    def test_valid_bundle_and_checksums_pass(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            archive, _ = fixture.build()
            result = VERIFIER.verify_release_artifacts(
                fixture.output,
                expected_commit=COMMIT,
            )
            self.assertEqual(result["archive"], archive.name)
            self.assertEqual(result["source_commit"], COMMIT)

    def test_safe_internal_symlink_mode_is_not_treated_as_writable_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.install.joinpath("lib/libfixture-alias.so").symlink_to(
                "libstudica_drivers.so"
            )
            fixture.build()
            result = VERIFIER.verify_release_artifacts(
                fixture.output,
                expected_commit=COMMIT,
            )
            self.assertEqual(result["source_commit"], COMMIT)

    def test_external_checksum_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            _, checksum = fixture.build()
            checksum.write_text(
                f"{'0' * 64}  {checksum.name.removesuffix('.sha256')}\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                VERIFIER.VerificationError,
                "external archive checksum",
            ):
                VERIFIER.verify_release_artifacts(fixture.output)

    def test_internal_checksum_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            archive_path, checksum_path = fixture.build()
            changed_path = archive_path.with_suffix(".changed")
            with tarfile.open(archive_path, "r:gz") as source_archive:
                with tarfile.open(changed_path, "w:gz") as changed_archive:
                    for member in source_archive.getmembers():
                        stream = source_archive.extractfile(member)
                        if stream is None:
                            data = None
                        else:
                            with stream:
                                data = stream.read()
                        if member.name.endswith("/metadata/DO_NOT_ACTIVATE"):
                            data = data + b"tampered\n"
                            member.size = len(data)
                        changed_archive.addfile(
                            member,
                            io.BytesIO(data) if data is not None else None,
                        )
            changed_path.replace(archive_path)
            checksum_path.write_text(
                f"{VERIFIER.sha256_file(archive_path)}  {archive_path.name}\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(
                VERIFIER.VerificationError,
                "internal checksum mismatch",
            ):
                VERIFIER.verify_release_artifacts(fixture.output)

    def test_activation_or_wrong_commit_is_rejected(self):
        metadata = {
            "schema_version": 1,
            "product": VERIFIER.PRODUCT,
            "release_version": "0.1.0-dev.fixture",
            "channel": "development",
            "activation_authorized": False,
            "activation_blocker": "hardware gate pending",
            "source": {"commit": COMMIT},
            "platform": {
                "architecture": "arm64",
                "os_id": "ubuntu",
                "ros_distro": "humble",
                "version_id": "22.04",
            },
            "builder": {
                "profile": VERIFIER.BUILDER_PROFILE,
                "image_id": "sha256:" + "b" * 64,
            },
        }
        rollback = {"activation_authorized": False}
        changed = deepcopy(metadata)
        changed["activation_authorized"] = True
        with self.assertRaisesRegex(VERIFIER.VerificationError, "authorizes activation"):
            VERIFIER.validate_release_policy(
                changed,
                rollback,
                metadata["release_version"],
                COMMIT,
            )
        with self.assertRaisesRegex(VERIFIER.VerificationError, "does not match"):
            VERIFIER.validate_release_policy(
                metadata,
                rollback,
                metadata["release_version"],
                "c" * 40,
            )

    def test_unexpected_output_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.build()
            fixture.output.joinpath("unexpected.txt").write_text(
                "unexpected\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(
                VERIFIER.VerificationError,
                "exactly one archive",
            ):
                VERIFIER.verify_release_artifacts(fixture.output)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
