#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

readonly source_mount="/source"
readonly workspace="/workspace"
readonly source_root="${workspace}/src/studica_vmxpi_ros2"
readonly ydlidar_prefix="${workspace}/vendor/ydlidar_sdk"
readonly inventory="/inputs/dpkg-inventory.tsv"
readonly output_dir="/output"

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

source_setup_file() {
  local setup_file="$1"
  [[ -r "${setup_file}" ]] || die "ROS setup file is missing: ${setup_file}"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

[[ "$(uname -m)" == "aarch64" ]] || die "builder must execute as AArch64"
[[ -d "${source_mount}/.git" ]] || die "read-only Git source mount is missing"
[[ -r /usr/local/include/vmxpi/VMXPi.h ]] || die "VMXPi SDK headers are missing"
[[ -r /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so ]] || die \
  "VMXPi SDK shared library is missing"
[[ -r "${inventory}" ]] || die "runtime dpkg inventory is missing"
[[ -d "${output_dir}" && -w "${output_dir}" ]] || die \
  "release output mount is not writable"
[[ "${STUDICA_BUILDER_IMAGE_ID:-}" =~ ^sha256:[0-9a-f]{64}$ ]] || die \
  "STUDICA_BUILDER_IMAGE_ID must be an immutable Docker image ID"

mkdir -p "${workspace}/src" "${workspace}/home" "${workspace}/vendor"
export HOME="${workspace}/home"
export LD_LIBRARY_PATH="/usr/local/lib/vmxpi${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

git -C "${source_mount}" diff --quiet --ignore-submodules --
git -C "${source_mount}" diff --cached --quiet --ignore-submodules --
[[ -z "$(git -C "${source_mount}" status --porcelain --untracked-files=normal)" ]] || die \
  "source repository must be clean"

git clone --no-local --no-hardlinks "${source_mount}" "${source_root}"
vcs import --recursive "${workspace}/src" \
  < "${source_root}/dependencies/hardware.repos"
python3 "${source_root}/scripts/verify_hardware_checkout.py" \
  --workspace "${workspace}"

rosdep --sources-cache-dir /opt/studica/rosdep-cache check \
  --from-paths "${workspace}/src" \
  --ignore-src \
  --rosdistro humble \
  --skip-keys "gz_ros2_control ros_gz_bridge ros_gz_sim"

cmake \
  -S "${workspace}/src/YDLidar-SDK" \
  -B "${workspace}/ydlidar-sdk-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ydlidar_prefix}" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TEST=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE
cmake --build "${workspace}/ydlidar-sdk-build" --parallel "$(nproc)"
cmake --install "${workspace}/ydlidar-sdk-build"

export CMAKE_PREFIX_PATH="${ydlidar_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${ydlidar_prefix}/lib:${LD_LIBRARY_PATH}"

source_setup_file /opt/ros/humble/setup.bash
colcon --log-base "${workspace}/log" build \
  --base-paths "${workspace}/src" \
  --build-base "${workspace}/build" \
  --install-base "${workspace}/install" \
  --merge-install \
  --packages-ignore ydlidar_sdk \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DSTUDICA_PRODUCTION_INSTALL=ON

source_setup_file "${workspace}/install/setup.bash"
colcon --log-base "${workspace}/log" test \
  --base-paths "${workspace}/src" \
  --build-base "${workspace}/build" \
  --install-base "${workspace}/install" \
  --packages-select \
    studica_drivers \
    studica_robot_monitor \
    studica_ros2_control \
    studica_vmxpi_ros2 \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --test-result-base "${workspace}/build" --verbose
python3 "${source_root}/scripts/verify_hardware_checkout.py" \
  --workspace "${workspace}"

python3 "${source_root}/scripts/build_release_bundle.py" \
  --source-root "${source_root}" \
  --install-prefix "${workspace}/install" \
  --vmxpi-sdk-root /usr/local \
  --dpkg-inventory "${inventory}" \
  --builder-image-id "${STUDICA_BUILDER_IMAGE_ID}" \
  --output-dir "${output_dir}"
