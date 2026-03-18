#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

profiles_dir="${repo_root}/bringup/config/profiles"
template_dir="${repo_root}/bringup/config/profile_template"

mapfile -t launch_py_files < <(find "${repo_root}/bringup/launch" -maxdepth 1 -type f -name '*.py' | sort)

if [[ "${#launch_py_files[@]}" -eq 0 ]]; then
  echo "ERROR: no launch python files found under ${repo_root}/bringup/launch" >&2
  exit 2
fi

echo "[check] Python syntax: launch + validation scripts"
python3 -m py_compile "${launch_py_files[@]}" "${repo_root}/scripts/validate_profiles.py"

echo "[check] Profile schema validation"
python3 "${repo_root}/scripts/validate_profiles.py" \
  --profiles-dir "${profiles_dir}" \
  --template-dir "${template_dir}"

echo "[check] All checks passed."
