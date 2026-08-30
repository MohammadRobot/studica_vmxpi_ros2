#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
dockerfile="${repo_root}/deployment/arm64-builder.Dockerfile"

sdk_root=""
output_dir=""
allow_emulation=false
check_only=false
build_workers="$(nproc)"
build_workspace=""
prepared_sources=""
offline_workspace=""

usage() {
  cat <<'EOF'
Build a development-only Studica ARM64 release in an isolated container.

Usage:
  ./scripts/build_arm64_release.sh --sdk-root PATH --output-dir PATH [options]

Required:
  --sdk-root PATH     Root containing include/vmxpi and lib/vmxpi.
  --output-dir PATH   Artifact directory outside this Git repository.

Options:
  --build-workers N  Limit SDK-enabled compile/test work to N CPU workers.
  --allow-emulation   Permit an amd64 Docker host with registered ARM64 QEMU.
  --check-only        Validate prerequisites without building images or artifacts.
  -h, --help          Show this help.

Native ARM64 is required by default. This script never connects to the robot,
installs or activates an artifact, or enables ROS services. Dependency source
is acquired without the SDK; compilation with the SDK runs without networking.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

cleanup() {
  if [[ -n "${build_workspace}" && -d "${build_workspace}" ]]; then
    case "${build_workspace}" in
      /tmp/studica-arm64-build.*) rm -rf -- "${build_workspace}" ;;
      *) printf 'WARN: refusing to remove unexpected path %s\n' "${build_workspace}" >&2 ;;
    esac
  fi
}

trap cleanup EXIT

while (($# > 0)); do
  case "$1" in
    --sdk-root)
      (($# >= 2)) || die "--sdk-root needs a path"
      sdk_root="$2"
      shift 2
      ;;
    --output-dir)
      (($# >= 2)) || die "--output-dir needs a path"
      output_dir="$2"
      shift 2
      ;;
    --build-workers)
      (($# >= 2)) || die "--build-workers needs a positive integer"
      build_workers="$2"
      shift 2
      ;;
    --allow-emulation)
      allow_emulation=true
      shift
      ;;
    --check-only)
      check_only=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *) die "unknown option: $1" ;;
  esac
done

[[ "${build_workers}" =~ ^[1-9][0-9]*$ ]] || die \
  "--build-workers must be a positive integer"
((build_workers <= $(nproc))) || die \
  "--build-workers cannot exceed the host CPU count ($(nproc))"
[[ -n "${sdk_root}" ]] || die "--sdk-root is required"
[[ -n "${output_dir}" ]] || die "--output-dir is required"
sdk_root="$(realpath -e "${sdk_root}")"
output_dir="$(realpath -m "${output_dir}")"

[[ -r "${sdk_root}/include/vmxpi/VMXPi.h" ]] || die \
  "${sdk_root}/include/vmxpi/VMXPi.h is missing"
[[ -r "${sdk_root}/lib/vmxpi/libvmxpi_hal_cpp.so" ]] || die \
  "${sdk_root}/lib/vmxpi/libvmxpi_hal_cpp.so is missing"
case "${sdk_root}/" in
  "${repo_root}/"*) die "VMXPi SDK root must be outside the Git repository" ;;
esac
[[ "${output_dir}" != "/" ]] || die "filesystem root cannot be the output directory"
case "${output_dir}/" in
  "${sdk_root}/"*) die "output directory must not overlap the VMXPi SDK root" ;;
esac
case "${sdk_root}/" in
  "${output_dir}/"*) die "output directory must not contain the VMXPi SDK root" ;;
esac
case "${output_dir}/" in
  "${repo_root}/"*) die "output directory must be outside the Git repository" ;;
esac

python3 "${repo_root}/scripts/validate_arm64_builder.py" --root "${repo_root}"
[[ -z "$(git -C "${repo_root}" status --porcelain --untracked-files=normal)" ]] || die \
  "source repository is dirty"
command -v docker >/dev/null 2>&1 || die "Docker with Buildx is required"
docker version >/dev/null 2>&1 || die \
  "cannot access the Docker daemon; use a dedicated builder account with Docker access"
docker buildx version >/dev/null 2>&1 || die "Docker Buildx is required"

docker_architecture="$(docker info --format '{{.Architecture}}')"
case "${docker_architecture}" in
  arm64|aarch64) ;;
  *)
    ${allow_emulation} || die \
      "Docker host is ${docker_architecture}; use native ARM64 or pass --allow-emulation"
    docker buildx inspect --bootstrap | grep -q 'linux/arm64' || die \
      "the active Buildx builder does not advertise linux/arm64"
    ;;
esac

printf '[builder] prerequisites passed: Docker host=%s, target=linux/arm64, workers=%s\n' \
  "${docker_architecture}" "${build_workers}"
if ${check_only}; then
  printf '[builder] check-only complete; no image or artifact was created\n'
  exit 0
fi

mkdir -p "${output_dir}"
build_workspace="$(mktemp -d -t studica-arm64-build.XXXXXXXX)"
prepared_sources="${build_workspace}/prepared"
offline_workspace="${build_workspace}/offline"
mkdir -p "${prepared_sources}" "${offline_workspace}"
commit="$(git -C "${repo_root}" rev-parse HEAD)"
builder_tag="studica-arm64-builder:${commit:0:12}"
runtime_tag="studica-arm64-runtime-inventory:${commit:0:12}"

docker buildx build \
  --file "${dockerfile}" \
  --platform linux/arm64 \
  --target build-env \
  --tag "${builder_tag}" \
  --load \
  "${repo_root}"
docker buildx build \
  --file "${dockerfile}" \
  --platform linux/arm64 \
  --target runtime-inventory \
  --tag "${runtime_tag}" \
  --load \
  "${repo_root}"

inventory="${build_workspace}/dpkg-inventory.tsv"
docker run --rm \
  --platform linux/arm64 \
  --network none \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  "${runtime_tag}" | LC_ALL=C sort > "${inventory}"
builder_image_id="$(docker image inspect --format '{{.Id}}' "${builder_tag}")"
[[ "${builder_image_id}" =~ ^sha256:[0-9a-f]{64}$ ]] || die \
  "Docker returned an invalid builder image ID"

# Dependency acquisition is intentionally separated from the proprietary SDK.
# This container has network access but receives no VMXPi SDK mount.
docker run --rm \
  --platform linux/arm64 \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" \
  --entrypoint /usr/local/bin/prepare-studica-arm64-sources \
  --mount "type=bind,src=${repo_root},dst=/source,readonly" \
  --mount "type=bind,src=${prepared_sources},dst=/prepared" \
  --tmpfs /tmp:rw,nosuid,nodev,size=1g \
  "${builder_tag}"

# Source execution starts only after networking is disabled. The SDK and the
# complete prepared source tree are both mounted read-only.
docker run --rm \
  --platform linux/arm64 \
  --network none \
  --read-only \
  --cap-drop ALL \
  --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" \
  --cpus "${build_workers}" \
  --env "STUDICA_BUILDER_IMAGE_ID=${builder_image_id}" \
  --env "STUDICA_BUILD_WORKERS=${build_workers}" \
  --mount "type=bind,src=${prepared_sources},dst=/prepared,readonly" \
  --mount "type=bind,src=${sdk_root}/include/vmxpi,dst=/usr/local/include/vmxpi,readonly" \
  --mount "type=bind,src=${sdk_root}/lib/vmxpi,dst=/usr/local/lib/vmxpi,readonly" \
  --mount "type=bind,src=${inventory},dst=/inputs/dpkg-inventory.tsv,readonly" \
  --mount "type=bind,src=${output_dir},dst=/output" \
  --mount "type=bind,src=${offline_workspace},dst=/workspace" \
  --tmpfs /tmp:rw,nosuid,nodev,size=1g \
  "${builder_tag}"

printf '[builder] development artifact written beneath %s\n' "${output_dir}"
printf '[builder] activation remains blocked\n'
