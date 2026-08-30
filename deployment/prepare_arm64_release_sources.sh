#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

readonly source_mount="/source"
readonly prepared_root="/prepared"
readonly source_root="${prepared_root}/src/studica_vmxpi_ros2"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

[[ "$(uname -m)" == "aarch64" ]] || die "source preparation must execute as AArch64"
[[ -d "${source_mount}/.git" ]] || die "read-only Git source mount is missing"
[[ -d "${prepared_root}" && -w "${prepared_root}" ]] || die \
  "prepared-source mount is not writable"
[[ -z "$(find "${prepared_root}" -mindepth 1 -print -quit)" ]] || die \
  "prepared-source mount must be empty"

export GIT_OPTIONAL_LOCKS=0
git -C "${source_mount}" diff --quiet --ignore-submodules --
git -C "${source_mount}" diff --cached --quiet --ignore-submodules --
[[ -z "$(git -C "${source_mount}" status --porcelain --untracked-files=normal)" ]] || die \
  "source repository must be clean"

mkdir -p "${prepared_root}/src"
git clone --no-local --no-hardlinks "${source_mount}" "${source_root}"
vcs import --recursive "${prepared_root}/src" \
  < "${source_root}/dependencies/hardware.repos"
python3 "${source_root}/scripts/verify_hardware_checkout.py" \
  --workspace "${prepared_root}"

git -C "${source_root}" diff --quiet --ignore-submodules --
git -C "${source_root}" diff --cached --quiet --ignore-submodules --
[[ -z "$(git -C "${source_root}" status --porcelain --untracked-files=normal)" ]] || die \
  "prepared product source is not clean"
git -C "${source_root}" rev-parse HEAD > "${prepared_root}/source-commit"

printf '[builder] prepared pinned sources without mounting the VMXPi SDK\n'
