#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/create_profile.sh <profile_name> [--from-profile <existing_profile>] [--force]

Examples:
  scripts/create_profile.sh training_6wd
  scripts/create_profile.sh training_2wd_clone --from-profile training_2wd
  scripts/create_profile.sh training_4wd --force
EOF
}

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

profile_name=""
from_profile=""
force="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --from-profile)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --from-profile requires a value." >&2
        exit 1
      fi
      from_profile="$2"
      shift 2
      ;;
    --force)
      force="true"
      shift
      ;;
    *)
      if [[ -n "$profile_name" ]]; then
        echo "ERROR: multiple profile names provided." >&2
        usage
        exit 1
      fi
      profile_name="$1"
      shift
      ;;
  esac
done

if [[ -z "$profile_name" ]]; then
  echo "ERROR: profile name is required." >&2
  usage
  exit 1
fi

if ! [[ "$profile_name" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "ERROR: invalid profile name '$profile_name'. Use letters, numbers, '_' or '-'." >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
profiles_dir="${repo_root}/bringup/config/profiles"
template_dir="${repo_root}/bringup/config/profile_template"
target_dir="${profiles_dir}/${profile_name}"

if [[ -n "$from_profile" ]] && ! [[ "$from_profile" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "ERROR: invalid source profile name '$from_profile'." >&2
  exit 1
fi

if [[ -n "$from_profile" ]]; then
  source_dir="${profiles_dir}/${from_profile}"
else
  source_dir="${template_dir}"
fi

if [[ ! -d "$source_dir" ]]; then
  echo "ERROR: source profile directory not found: $source_dir" >&2
  exit 1
fi

if [[ ! -f "${source_dir}/robot_profile.yaml" || ! -f "${source_dir}/robot_controllers.yaml" ]]; then
  echo "ERROR: source directory must contain robot_profile.yaml and robot_controllers.yaml: $source_dir" >&2
  exit 1
fi

if [[ -d "$target_dir" ]]; then
  if [[ "$force" != "true" ]]; then
    echo "ERROR: target profile already exists: $target_dir" >&2
    echo "Use --force to overwrite it." >&2
    exit 1
  fi
  rm -r "$target_dir"
fi

mkdir -p "$target_dir"
cp "${source_dir}/robot_profile.yaml" "${target_dir}/robot_profile.yaml"
cp "${source_dir}/robot_controllers.yaml" "${target_dir}/robot_controllers.yaml"

echo "Created profile: ${profile_name}"
echo "  target: ${target_dir}"
if [[ -n "$from_profile" ]]; then
  echo "  source: ${source_dir}"
else
  echo "  source: ${template_dir}"
fi

validator="${repo_root}/scripts/validate_profiles.py"
if command -v python3 >/dev/null 2>&1 && [[ -f "$validator" ]]; then
  echo
  echo "Running profile validation..."
  python3 "$validator" --profiles-dir "$profiles_dir"
fi

cat <<EOF

Next steps:
  1. Edit:
     ${target_dir}/robot_profile.yaml
     ${target_dir}/robot_controllers.yaml
  2. Test:
     ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=${profile_name} gui:=true
EOF
