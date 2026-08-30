#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

readonly prepared_root="/prepared"
readonly workspace="/workspace"
readonly source_tree="${prepared_root}/src"
readonly source_root="${source_tree}/studica_vmxpi_ros2"
readonly ydlidar_prefix="${workspace}/vendor/ydlidar_sdk"
readonly inventory="/inputs/dpkg-inventory.tsv"
readonly output_dir="/output"
readonly build_workers="${STUDICA_BUILD_WORKERS:-$(nproc)}"

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
[[ -d "${source_root}/.git" ]] || die "prepared Git source mount is missing"
[[ -r "${prepared_root}/source-commit" ]] || die "prepared source commit is missing"
[[ -r /usr/local/include/vmxpi/VMXPi.h ]] || die "VMXPi SDK headers are missing"
[[ -r /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so ]] || die \
  "VMXPi SDK shared library is missing"
[[ -r "${inventory}" ]] || die "runtime dpkg inventory is missing"
[[ -d "${output_dir}" && -w "${output_dir}" ]] || die \
  "release output mount is not writable"
[[ "${STUDICA_BUILDER_IMAGE_ID:-}" =~ ^sha256:[0-9a-f]{64}$ ]] || die \
  "STUDICA_BUILDER_IMAGE_ID must be an immutable Docker image ID"
[[ "${build_workers}" =~ ^[1-9][0-9]*$ ]] || die \
  "STUDICA_BUILD_WORKERS must be a positive integer"

mkdir -p "${workspace}/build" "${workspace}/home" "${workspace}/install" \
  "${workspace}/log/ros" "${workspace}/pycache" "${workspace}/vendor"
export GIT_OPTIONAL_LOCKS=0
export HOME="${workspace}/home"
export CMAKE_BUILD_PARALLEL_LEVEL="${build_workers}"
export CTEST_PARALLEL_LEVEL="${build_workers}"
export LD_LIBRARY_PATH="/usr/local/lib/vmxpi${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export MAKEFLAGS="-j${build_workers}"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPYCACHEPREFIX="${workspace}/pycache"
export ROS_LOG_DIR="${workspace}/log/ros"
export ROS_LOCALHOST_ONLY=1
export XML_CATALOG_FILES="/opt/studica/ros-schema/catalog.xml"

[[ "$(git -C "${source_root}" rev-parse HEAD)" == \
  "$(tr -d '\n' < "${prepared_root}/source-commit")" ]] || die \
  "prepared source commit does not match its recorded identity"
git -C "${source_root}" diff --quiet --ignore-submodules --
git -C "${source_root}" diff --cached --quiet --ignore-submodules --
[[ -z "$(git -C "${source_root}" status --porcelain --untracked-files=normal)" ]] || die \
  "prepared product source must be clean"
python3 "${source_root}/scripts/verify_hardware_checkout.py" \
  --workspace "${prepared_root}"

rosdep --sources-cache-dir /opt/studica/rosdep-cache check \
  --from-paths "${source_tree}" \
  --ignore-src \
  --rosdistro humble \
  --skip-keys "gz_ros2_control ros_gz_bridge ros_gz_sim"

cmake \
  -S "${source_tree}/YDLidar-SDK" \
  -B "${workspace}/ydlidar-sdk-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${ydlidar_prefix}" \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_TEST=OFF \
  -DSWIG_FOUND=FALSE \
  -DPYTHONLIBS_FOUND=FALSE \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=TRUE
cmake --build "${workspace}/ydlidar-sdk-build" --parallel "${build_workers}"
cmake --install "${workspace}/ydlidar-sdk-build"

export CMAKE_PREFIX_PATH="${ydlidar_prefix}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${ydlidar_prefix}/lib:${LD_LIBRARY_PATH}"
# The pinned YDLidar package config exports ``-lydlidar_sdk`` but leaves
# YDLIDAR_SDK_LIBRARY_DIRS empty.  Keep the SDK in its isolated prefix and give
# GCC's linker the matching search directory instead of installing it globally.
export LIBRARY_PATH="${ydlidar_prefix}/lib${LIBRARY_PATH:+:${LIBRARY_PATH}}"

source_setup_file /opt/ros/humble/setup.bash
colcon --log-base "${workspace}/log" build \
  --executor sequential \
  --parallel-workers 1 \
  --base-paths "${source_tree}" \
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
  --executor sequential \
  --parallel-workers 1 \
  --base-paths "${source_tree}" \
  --build-base "${workspace}/build" \
  --install-base "${workspace}/install" \
  --merge-install \
  --packages-select \
    studica_drivers \
    studica_robot_monitor \
    studica_ros2_control \
    studica_vmxpi_ros2 \
  --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon --log-base "${workspace}/log" test-result \
  --test-result-base "${workspace}/build" \
  --verbose
python3 "${source_root}/scripts/verify_hardware_checkout.py" \
  --workspace "${prepared_root}"

# Some third-party launch tests import modules from the merged install prefix.
# Keep generated bytecode out of the immutable release even if a dependency
# overrides Python's cache-prefix setting.
find "${workspace}/install" -xdev -depth \
  \( -type f \( -name '*.pyc' -o -name '*.pyo' \) \
     -o -type d -name '__pycache__' \) \
  -delete

python3 "${source_root}/scripts/build_release_bundle.py" \
  --source-root "${source_root}" \
  --install-prefix "${workspace}/install" \
  --vmxpi-sdk-root /usr/local \
  --dpkg-inventory "${inventory}" \
  --builder-image-id "${STUDICA_BUILDER_IMAGE_ID}" \
  --output-dir "${output_dir}"
