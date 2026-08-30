#!/usr/bin/env python3
"""Contracts for post-build development release artifact verification."""

from copy import deepcopy
import io
import importlib.util
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

from test_release_bundle import COMMIT, ReleaseFixture


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
VERIFIER_PATH = ROOT / "scripts" / "verify_release_artifacts.py"
SPEC = importlib.util.spec_from_file_location("verify_release_artifacts", VERIFIER_PATH)
assert SPEC is not None and SPEC.loader is not None
VERIFIER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = VERIFIER
SPEC.loader.exec_module(VERIFIER)


class ReleaseArtifactVerificationTest(unittest.TestCase):

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
