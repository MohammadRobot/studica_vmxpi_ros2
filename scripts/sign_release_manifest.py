#!/usr/bin/env python3
"""Offline publisher tool for an Ed25519 production-release envelope."""

import argparse
import base64
import json
from pathlib import Path

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey

from studica_robot_platform.updates import (
    PLATFORM, PRODUCT, canonical_json, sha256_file, validate_signed_qualification,
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--version", required=True)
    parser.add_argument("--artifact-url", required=True)
    parser.add_argument("--private-key", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--qualification", type=Path,
                        help="hardware acceptance report for this exact development bundle")
    options = parser.parse_args()
    key = serialization.load_pem_private_key(
        options.private_key.read_bytes(), password=None
    )
    if not isinstance(key, Ed25519PrivateKey):
        raise SystemExit("private key must be Ed25519")
    signed = {
        "artifact_bytes": options.artifact.stat().st_size,
        "artifact_sha256": sha256_file(options.artifact),
        "artifact_url": options.artifact_url,
        "platform": PLATFORM,
        "product": PRODUCT,
        "release_root": f"opt/studica/releases/{options.version}",
        "version": options.version,
    }
    if options.qualification:
        report = json.loads(options.qualification.read_text())
        validate_signed_qualification(report, signed["artifact_sha256"])
        signed["qualification"] = report
    envelope = {
        "schema_version": 1,
        "signed": signed,
        "signature": base64.b64encode(key.sign(canonical_json(signed))).decode("ascii"),
    }
    options.output.write_text(json.dumps(envelope, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
