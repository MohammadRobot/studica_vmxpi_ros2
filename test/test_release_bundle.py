#!/usr/bin/env python3
"""Contracts for the deterministic VMXPi ARM64 application release bundle."""

import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tarfile
import tempfile
import unittest


ROOT = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).parents[1]
BUILDER_PATH = ROOT / "scripts" / "build_release_bundle.py"
SPEC = importlib.util.spec_from_file_location("build_release_bundle", BUILDER_PATH)
assert SPEC is not None and SPEC.loader is not None
BUILDER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BUILDER
SPEC.loader.exec_module(BUILDER)
COMMIT = "a" * 40
EPOCH = 1_788_000_000
RELEASE_VERSION = "0.1.0-dev.fixture"


def write_fake_aarch64_elf(path):
    header = bytearray(64)
    header[:4] = b"\x7fELF"
    header[4] = 2
    header[5] = 1
    struct.pack_into("<H", header, 18, BUILDER.AARCH64_ELF_MACHINE)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(header + b"studica-fixture\n")
    path.chmod(0o755)


def package_xml(name, version="1.0.0"):
    return (
        '<?xml version="1.0"?>\n'
        '<package format="3">\n'
        f"  <name>{name}</name>\n"
        f"  <version>{version}</version>\n"
        "  <description>Release fixture</description>\n"
        "  <maintainer email=\"test@example.com\">Test</maintainer>\n"
        "  <license>Apache-2.0</license>\n"
        "</package>\n"
    )


class ReleaseFixture:

    def __init__(self, root):
        self.root = Path(root)
        self.install = self.root / "install"
        self.output = self.root / "output"
        self.inventory = self.root / "dpkg-inventory.tsv"
        self.profile = BUILDER.read_json(
            ROOT / "deployment" / "vmxpi-runtime-packages-v1.json"
        )
        self._write_install()
        self._write_inventory()

    def _write_install(self):
        self.install.mkdir()
        self.install.joinpath("setup.bash").write_text(
            "# production fixture\n", encoding="utf-8"
        )
        marker_source = ROOT / "deployment" / "vmxpi-production-install-v1.json.in"
        marker_target = self.install / (
            "share/studica_vmxpi_ros2/deployment/vmxpi-production-install-v1.json"
        )
        marker_target.parent.mkdir(parents=True)
        marker_target.write_text(marker_source.read_text(encoding="utf-8"), encoding="utf-8")

        for name in self.profile["overlay_ros_packages"]:
            marker = self.install / "share" / name / "package.xml"
            marker.parent.mkdir(parents=True, exist_ok=True)
            version = "0.1.0" if name == "studica_vmxpi_ros2" else "1.0.0"
            marker.write_text(package_xml(name, version), encoding="utf-8")

        for relative_path in BUILDER.REQUIRED_ARM64_FILES:
            write_fake_aarch64_elf(self.install / relative_path)
        header = self.install / "include/vendor/example.hpp"
        header.parent.mkdir(parents=True)
        header.write_text("// build-only fixture\n", encoding="utf-8")
        self.install.joinpath("lib/libfixture.a").write_bytes(b"!<arch>\n")
        cmake_file = self.install / (
            "share/studica_vmxpi_ros2/cmake/fixture.cmake"
        )
        cmake_file.parent.mkdir()
        cmake_file.write_text("# build-only fixture\n", encoding="utf-8")

    def _write_inventory(self):
        package_names = BUILDER.enabled_apt_packages(self.profile) | {"base-files"}
        lines = [f"{name}\t1.0-1\tarm64" for name in sorted(package_names)]
        self.inventory.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def build(self, output=None):
        return BUILDER.build_release_bundle(
            source_root=ROOT,
            install_prefix=self.install,
            dpkg_inventory_path=self.inventory,
            output_dir=output or self.output,
            commit=COMMIT,
            epoch=EPOCH,
            release_version=RELEASE_VERSION,
        )


class ReleaseBundleTest(unittest.TestCase):

    def test_bundle_contains_release_metadata_sbom_and_valid_checksums(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            archive, checksum = fixture.build()
            self.assertTrue(archive.is_file())
            self.assertTrue(checksum.is_file())
            self.assertIn(BUILDER.sha256_file(archive), checksum.read_text())

            extracted = Path(temporary) / "extracted"
            with tarfile.open(archive, "r:gz") as bundle:
                bundle.extractall(extracted, filter="data")
            release_root = (
                extracted / "opt" / "studica" / "releases" / RELEASE_VERSION
            )
            metadata = json.loads(
                release_root.joinpath("metadata/release.json").read_text()
            )
            self.assertEqual(metadata["channel"], "development")
            self.assertFalse(metadata["activation_authorized"])
            self.assertEqual(metadata["source"]["commit"], COMMIT)
            self.assertEqual(
                metadata["install"]["ros_packages"],
                fixture.profile["overlay_ros_packages"],
            )
            self.assertIn(
                "include",
                metadata["install"]["pruned_build_artifacts"],
            )
            self.assertFalse(release_root.joinpath("install/include").exists())
            self.assertFalse(
                release_root.joinpath("install/lib/libfixture.a").exists()
            )
            self.assertFalse(
                release_root.joinpath(
                    "install/share/studica_vmxpi_ros2/cmake"
                ).exists()
            )
            sbom = json.loads(
                release_root.joinpath("metadata/sbom.spdx.json").read_text()
            )
            self.assertEqual(sbom["spdxVersion"], "SPDX-2.3")
            self.assertTrue(release_root.joinpath("metadata/DO_NOT_ACTIVATE").is_file())

            for line in release_root.joinpath("SHA256SUMS").read_text().splitlines():
                expected, relative = line.split("  ", maxsplit=1)
                self.assertEqual(BUILDER.sha256_file(release_root / relative), expected)

    def test_same_inputs_produce_identical_archive(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            first, _ = fixture.build(Path(temporary) / "output-a")
            second, _ = fixture.build(Path(temporary) / "output-b")
            self.assertEqual(first.read_bytes(), second.read_bytes())

    def test_prohibited_target_package_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            lines = fixture.inventory.read_text(encoding="utf-8").splitlines()
            lines.append("git\t1:2.34.1\tarm64")
            fixture.inventory.write_text(
                "\n".join(sorted(lines)) + "\n", encoding="utf-8"
            )
            with self.assertRaisesRegex(BUILDER.BundleError, "non-production packages"):
                fixture.build()

    def test_normal_developer_install_without_production_marker_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.install.joinpath(
                "share/studica_vmxpi_ros2/deployment/vmxpi-production-install-v1.json"
            ).unlink()
            with self.assertRaisesRegex(
                BUILDER.BundleError,
                "STUDICA_PRODUCTION_INSTALL=ON",
            ):
                fixture.build()

    def test_output_inside_source_tree_is_rejected_before_creation(self):
        output = ROOT / "release-artifacts-test"
        self.assertFalse(output.exists())
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            with self.assertRaisesRegex(BUILDER.BundleError, "outside the Git source"):
                fixture.build(output)
        self.assertFalse(output.exists())

    def test_non_aarch64_binary_and_escaping_symlink_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            binary = fixture.install / BUILDER.REQUIRED_ARM64_FILES[0]
            image = bytearray(binary.read_bytes())
            struct.pack_into("<H", image, 18, 62)
            binary.write_bytes(image)
            with self.assertRaisesRegex(BUILDER.BundleError, "not AArch64"):
                fixture.build()

        with tempfile.TemporaryDirectory() as temporary:
            fixture = ReleaseFixture(temporary)
            fixture.install.joinpath("escaping-link").symlink_to("/tmp")
            with self.assertRaisesRegex(BUILDER.BundleError, "symlink escapes"):
                fixture.build()

    def test_builder_help_and_production_cmake_gate_are_present(self):
        help_result = subprocess.run(
            [sys.executable, str(BUILDER_PATH), "--help"],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(help_result.returncode, 0, help_result.stderr)
        self.assertIn("--dpkg-inventory", help_result.stdout)
        cmake = ROOT.joinpath("CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("STUDICA_PRODUCTION_INSTALL", cmake)
        self.assertIn("requires the VMXPi hardware interface", cmake)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
