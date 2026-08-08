#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

readonly ROS_DISTRO_NAME="humble"
readonly GZ_ROS2_CONTROL_COMMIT="a2d290e37be67ba082744e323339d82031f051c0"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
default_workspace="$(cd "${repo_root}/../.." && pwd)"

mode="simulation"
workspace="${default_workspace}"
check_only=false
non_interactive=false
temporary_dir=""

usage() {
  cat <<'EOF'
Set up the Studica ROS 2 classroom workspace on Ubuntu 22.04.

Usage:
  ./scripts/setup_ubuntu.sh [options]

Options:
  --mode simulation|hardware  Install simulation (default) or robot dependencies.
  --workspace PATH             Workspace containing this repository under src/.
  --check-only                 Check the computer without changing it.
  --non-interactive            Never prompt; requires passwordless sudo.
  -h, --help                   Show this help.

Run this script as your normal user, not with sudo. It never edits ~/.bashrc and
never starts the robot or sends a motion command.
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 2
}

info() {
  printf '[setup] %s\n' "$*"
}

cleanup() {
  if [[ -n "${temporary_dir}" && -d "${temporary_dir}" ]]; then
    rm -rf -- "${temporary_dir}"
  fi
}

trap cleanup EXIT

while (($# > 0)); do
  case "$1" in
    --mode)
      (($# >= 2)) || die "--mode needs simulation or hardware"
      mode="$2"
      shift 2
      ;;
    --workspace)
      (($# >= 2)) || die "--workspace needs a path"
      workspace="$2"
      shift 2
      ;;
    --check-only)
      check_only=true
      shift
      ;;
    --non-interactive)
      non_interactive=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
done

[[ "${mode}" == "simulation" || "${mode}" == "hardware" ]] || \
  die "--mode must be simulation or hardware"
[[ ${EUID} -ne 0 ]] || die "run this script as your normal user, not with sudo"

if [[ "${mode}" == "simulation" ]]; then
  # Humble defaults to Gazebo Fortress. The pinned overlay is intentionally
  # built against Harmonic instead.
  export GZ_VERSION=harmonic
fi

workspace="$(realpath -m "${workspace}")"
expected_repo="$(realpath -m "${workspace}/src/studica_vmxpi_ros2")"
[[ "${repo_root}" == "${expected_repo}" ]] || die \
  "this repository must be ${expected_repo}; use --workspace if needed"

check_host() {
  [[ -r /etc/os-release ]] || die "cannot identify this operating system"
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]] || die \
    "Ubuntu 22.04 is required (found ${PRETTY_NAME:-unknown})"

  local architecture
  architecture="$(dpkg --print-architecture)"
  case "${architecture}" in
    amd64|arm64) ;;
    *) die "unsupported architecture ${architecture}; use amd64 or arm64" ;;
  esac

  if [[ "${mode}" == "hardware" && "${architecture}" != "arm64" ]]; then
    die "hardware mode must run on the arm64 VMXPi robot computer"
  fi

  local required_gib available_kib required_kib
  if [[ "${mode}" == "simulation" ]]; then
    required_gib=12
  else
    required_gib=6
  fi
  available_kib="$(df -Pk "$(dirname "${workspace}")" | awk 'NR == 2 {print $4}')"
  required_kib=$((required_gib * 1024 * 1024))
  ((available_kib >= required_kib)) || die \
    "at least ${required_gib} GiB free is required; free some disk space first"

  command -v sudo >/dev/null || die "sudo is required"
  if ! sudo -n true 2>/dev/null && ! id -nG | tr ' ' '\n' | grep -qx sudo; then
    die "${USER} must be allowed to use sudo"
  fi

  local network_check_url
  network_check_url="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
  if command -v curl >/dev/null; then
    curl -fsS --max-time 10 -o /dev/null "${network_check_url}" || die \
      "internet access to GitHub is required"
  elif command -v wget >/dev/null; then
    wget -q --spider --timeout=10 "${network_check_url}" || die \
      "internet access to GitHub is required"
    info "curl is not installed yet; setup will install it"
  else
    getent hosts raw.githubusercontent.com >/dev/null || die \
      "internet access to GitHub is required"
    info "curl is not installed yet; setup will install it"
  fi

  if [[ "${mode}" == "hardware" ]]; then
    [[ -r /usr/local/include/vmxpi/VMXPi.h ]] || die \
      "VMXPi SDK headers are missing; install the supported Studica VMXPi image first"
    [[ -r /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so || \
       -r /usr/local/lib/vmxpi/libvmxpi_hal_cpp.a ]] || die \
      "VMXPi HAL library is missing; install the supported Studica VMXPi image first"
  fi

  info "host check passed: Ubuntu 22.04, ${architecture}, ${mode} mode"
}

as_root() {
  if ${non_interactive}; then
    sudo -n "$@"
  else
    sudo "$@"
  fi
}

install_ros_repository() {
  info "configuring the ROS 2 Humble package repository"
  as_root apt-get update
  as_root apt-get install -y curl gnupg lsb-release locales software-properties-common
  as_root add-apt-repository -y universe

  curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o "${temporary_dir}/ros-archive-keyring.gpg"
  as_root install -m 0644 "${temporary_dir}/ros-archive-keyring.gpg" \
    /usr/share/keyrings/ros-archive-keyring.gpg

  local architecture codename source_line
  architecture="$(dpkg --print-architecture)"
  codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME}")"
  source_line="deb [arch=${architecture} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main"
  printf '%s\n' "${source_line}" | as_root tee /etc/apt/sources.list.d/ros2.list >/dev/null
}

install_gazebo_repository() {
  info "configuring Gazebo Harmonic packages"
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o "${temporary_dir}/gazebo-keyring.gpg"
  as_root install -m 0644 "${temporary_dir}/gazebo-keyring.gpg" \
    /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

  local architecture codename source_line
  architecture="$(dpkg --print-architecture)"
  codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME}")"
  source_line="deb [arch=${architecture} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${codename} main"
  printf '%s\n' "${source_line}" | as_root tee \
    /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
}

install_packages() {
  install_ros_repository
  if [[ "${mode}" == "simulation" ]]; then
    install_gazebo_repository
  fi
  as_root apt-get update

  local packages=(
    build-essential
    cmake
    git
    python3-colcon-common-extensions
    python3-pip
    python3-pytest
    python3-rosdep
    python3-vcstool
    python3-yaml
    ros-dev-tools
    ros-humble-backward-ros
    ros-humble-controller-manager
    ros-humble-desktop
    ros-humble-diagnostic-aggregator
    ros-humble-foxglove-bridge
    ros-humble-forward-command-controller
    ros-humble-imu-sensor-broadcaster
    ros-humble-joy
    ros-humble-nav2-bringup
    ros-humble-navigation2
    ros-humble-robot-state-publisher
    ros-humble-ros2-control
    ros-humble-ros2-controllers
    ros-humble-rosbag2-storage-mcap
    ros-humble-rqt-common-plugins
    ros-humble-slam-toolbox
    ros-humble-teleop-twist-joy
    ros-humble-teleop-twist-keyboard
    ros-humble-xacro
    shellcheck
  )
  if [[ "${mode}" == "simulation" ]]; then
    packages+=(
      gz-harmonic
      ros-humble-ros-gzharmonic
      ros-humble-ros-gzharmonic-bridge
    )
  fi

  info "installing ROS 2 classroom packages"
  if ${non_interactive}; then
    as_root env DEBIAN_FRONTEND=noninteractive apt-get install -y "${packages[@]}"
  else
    as_root apt-get install -y "${packages[@]}"
  fi
}

import_sources() {
  local manifest="${repo_root}/dependencies/${mode}.repos"
  [[ -r "${manifest}" ]] || die "dependency manifest not found: ${manifest}"
  mkdir -p "${workspace}/src"
  info "importing ${mode} dependencies (existing repositories are kept)"
  vcs import --skip-existing "${workspace}/src" < "${manifest}"

  if [[ "${mode}" == "simulation" ]]; then
    local overlay="${workspace}/src/gz_ros2_control"
    [[ -d "${overlay}/.git" ]] || die "gz_ros2_control overlay was not imported"
    if [[ "$(git -C "${overlay}" rev-parse HEAD)" != "${GZ_ROS2_CONTROL_COMMIT}" ]]; then
      [[ -z "$(git -C "${overlay}" status --porcelain)" ]] || die \
        "gz_ros2_control has local changes; save them before setup can pin the overlay"
      info "pinning gz_ros2_control to ${GZ_ROS2_CONTROL_COMMIT}"
      git -C "${overlay}" fetch origin "${GZ_ROS2_CONTROL_COMMIT}"
      git -C "${overlay}" switch --detach "${GZ_ROS2_CONTROL_COMMIT}"
    fi
  fi
}

install_ydlidar_sdk() {
  [[ "${mode}" == "hardware" ]] || return 0
  if [[ -r /usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake || \
        -r /usr/local/share/YDLIDAR_SDK/FindYDLIDAR_SDK.cmake ]]; then
    info "YDLidar SDK already installed"
    return 0
  fi

  local sdk_source="${workspace}/src/YDLidar-SDK"
  local sdk_build="${workspace}/.setup/ydlidar_sdk-build"
  [[ -d "${sdk_source}" ]] || die "YDLidar SDK source was not imported"
  info "building the pinned YDLidar SDK"
  cmake -S "${sdk_source}" -B "${sdk_build}" -DCMAKE_BUILD_TYPE=Release
  cmake --build "${sdk_build}" --parallel "$(nproc)"
  as_root cmake --install "${sdk_build}"
}

install_workspace_dependencies() {
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  if [[ ! -r /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    info "initializing rosdep"
    as_root rosdep init
  fi
  rosdep update --rosdistro "${ROS_DISTRO_NAME}"

  local skip_keys=""
  if [[ "${mode}" == "simulation" ]]; then
    # The versioned Harmonic packages above provide these two dependencies.
    # Resolving their unversioned rosdep keys would install Humble's default
    # Fortress packages alongside Harmonic.
    skip_keys="orbbec_camera ydlidar_ros2_driver ros_gz_bridge ros_gz_sim"
  else
    skip_keys="gz_ros2_control ros_gz_bridge ros_gz_sim"
  fi
  info "installing source-package dependencies with rosdep"
  rosdep install --from-paths "${workspace}/src" --ignore-src -r -y \
    --rosdistro "${ROS_DISTRO_NAME}" --skip-keys "${skip_keys}"
}

build_and_validate() {
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  info "running source validation"
  "${repo_root}/scripts/check_project.sh"

  info "building the workspace with symlink install"
  colcon --log-base "${workspace}/log" build \
    --base-paths "${workspace}/src" \
    --build-base "${workspace}/build" \
    --install-base "${workspace}/install" \
    --symlink-install \
    --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

  # shellcheck disable=SC1091
  source "${workspace}/install/setup.bash"
  info "running first-party package tests"
  local test_packages=(
    studica_drivers
    studica_robot_monitor
    studica_vmxpi_ros2
  )
  if colcon list --base-paths "${workspace}/src" --names-only | \
      grep -qx studica_ros2_control; then
    test_packages+=(studica_ros2_control)
  fi
  colcon --log-base "${workspace}/log" test \
    --base-paths "${workspace}/src" \
    --build-base "${workspace}/build" \
    --install-base "${workspace}/install" \
    --packages-select "${test_packages[@]}" \
    --event-handlers console_direct+
  colcon test-result --test-result-base "${workspace}/build" --verbose
}

check_host
if ${check_only}; then
  info "check-only complete; no files or packages were changed"
  exit 0
fi

if ${non_interactive}; then
  sudo -n true 2>/dev/null || die \
    "--non-interactive requires passwordless sudo; run without it to enter a password"
else
  info "sudo is needed to install system packages"
  sudo -v
fi

temporary_dir="$(mktemp -d -t studica_setup.XXXXXXXX)"
install_packages
import_sources
install_ydlidar_sdk
install_workspace_dependencies
build_and_validate

printf '\nSetup complete. In every new terminal, run:\n  source %q\n' \
  "${workspace}/install/setup.bash"
printf 'Your shell startup files were not changed. No robot motion was started.\n'
