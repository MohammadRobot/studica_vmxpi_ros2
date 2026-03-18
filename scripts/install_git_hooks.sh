#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git is not installed." >&2
  exit 1
fi

if [[ ! -d "${repo_root}/.git" ]]; then
  echo "ERROR: not a git repository: ${repo_root}" >&2
  exit 1
fi

if [[ ! -f "${repo_root}/.githooks/pre-commit" ]]; then
  echo "ERROR: missing pre-commit hook file: ${repo_root}/.githooks/pre-commit" >&2
  exit 1
fi

chmod +x "${repo_root}/.githooks/pre-commit"
git -C "${repo_root}" config core.hooksPath .githooks

echo "Installed repository hooks."
echo "  core.hooksPath=$(git -C "${repo_root}" config --get core.hooksPath)"
echo
echo "Pre-commit will now run scripts/check_project.sh when relevant files are staged."
