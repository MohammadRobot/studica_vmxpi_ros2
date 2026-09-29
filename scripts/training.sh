#!/usr/bin/env bash
# Copyright (c) 2026 studica_vmxpi_ros2 contributors
# SPDX-License-Identifier: Apache-2.0
# Source-checkout launcher for the isolated, simulation-only classroom track.
set -euo pipefail

usage() {
  echo 'Usage: bash scripts/training.sh COMMAND [--headless] [--dry-run] [MAP_NAME]'
  echo 'Commands: check, sim, slam, nav, teleop, arm, disarm, status, save-map, env'
  echo 'nav [MAP_NAME] uses the bundled office map, or a map saved by save-map.'
  echo 'save-map MAP_NAME creates a new project_maps/training/MAP_NAME directory.'
  echo 'Only sim, slam and nav accept --headless. No hardware mode is supported.'
  echo 'env prints Bash setup commands for the SAME isolated classroom ROS graph.'
}

fail() { echo "[training] $*" >&2; exit 2; }

if [[ $# == 0 || ${1:-} == --help || ${1:-} == -h ]]; then
  usage
  exit 0
fi
action="$1"
shift
case "$action" in
  check|sim|slam|nav|teleop|arm|disarm|status|save-map|env) ;;
  *) fail "Unknown command: $action. Hardware operation is not supported." ;;
esac

headless=false
dry_run=false
map_name=''
for argument in "$@"; do
  case "$argument" in
    --headless) headless=true ;;
    --dry-run) dry_run=true ;;
    *)
      [[ $action == nav || $action == save-map ]] || fail "Unexpected argument: $argument"
      [[ -z $map_name && $argument =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$ ]] ||
        fail 'Use one map name containing only letters, digits, underscores or hyphens.'
      map_name="$argument"
      ;;
  esac
done
if $headless && [[ $action != sim && $action != slam && $action != nav ]]; then
  fail '--headless is only valid for sim, slam or nav.'
fi
[[ $action != save-map || -n $map_name ]] || fail 'save-map requires a NEW map name.'

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"
workspace="$(realpath -m "${STUDICA_WS:-$repo_root/../..}")"
setup="$workspace/install/local_setup.bash"
network_config="$repo_root/bringup/config/network/cyclonedds_sim.xml"
session_dir="$workspace/robot_test_results/training-session"
map_dir="$workspace/project_maps/training/$map_name"
gui=true
$headless && gui=false

# Loopback is selected explicitly by this repository's Cyclone profile.
# ROS_LOCALHOST_ONLY=1 would select lo twice with the installed Cyclone version.
environment() {
  printf 'source %q\n' /opt/ros/humble/setup.bash "$setup"
  echo 'unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT'
  printf 'export STUDICA_WS=%q\n' "$workspace"
  echo 'export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
  printf 'export CYCLONEDDS_URI=%q\n' "file://$network_config"
  printf 'export ROS_LOG_DIR=%q\n' "$session_dir/log"
  echo 'export GZ_VERSION=harmonic'
}

command=()
case "$action" in
  check) command=(bash "$script_dir/check_project.sh") ;;
  sim)
    command=(ros2 launch studica_vmxpi_ros2 sim.launch.py "gui:=$gui"
      "gz_headless:=$headless" use_joystick:=false use_camera:=false use_point_cloud:=false)
    ;;
  slam)
    command=(ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=gz_sim
      "gui:=$gui" "gz_headless:=$headless" use_joystick:=false)
    ;;
  nav)
    command=(ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=gz_sim
      "gui:=$gui" "gz_headless:=$headless" use_joystick:=false use_point_cloud:=false)
    [[ -z $map_name ]] || command+=("map:=$map_dir/map.yaml")
    ;;
  teleop)
    command=(ros2 run teleop_twist_keyboard teleop_twist_keyboard
      --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.10 -p turn:=0.25)
    ;;
  arm|disarm) command=(timeout 15 ros2 service call "/robot/$action" std_srvs/srv/Trigger '{}') ;;
  status) command=(ros2 run studica_robot_monitor robot_check --mode simulation --strict) ;;
  save-map)
    command=(timeout 30 ros2 run nav2_map_server map_saver_cli -f "$map_dir/map"
      --ros-args -p use_sim_time:=true)
    ;;
  env) ;;
esac

if $dry_run; then
  environment
  if [[ ${#command[@]} -gt 0 ]]; then
    printf '%q ' "${command[@]}"
    printf '\n'
  fi
  exit 0
fi
[[ -r /opt/ros/humble/setup.bash && -r $setup && -r $network_config ]] ||
  fail 'ROS Humble or the workspace is not built. Follow docs/INSTALL.md first.'
if [[ $action == env ]]; then
  environment
  exit 0
fi

# Set isolation AFTER sourcing so a previous hardware terminal cannot override it.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$setup"
set -u
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$network_config" ROS_LOG_DIR="$session_dir/log"
export GZ_VERSION=harmonic
prefix="$(ros2 pkg prefix studica_vmxpi_ros2)"
[[ $(realpath "$prefix") == "$(realpath "$workspace/install/studica_vmxpi_ros2")" ]] ||
  fail 'The active platform package does not belong to the selected workspace.'
echo '[training] SIMULATION ONLY. ROS domain 77, explicit loopback DDS; no VMXPi connection.'
if [[ $action == nav ]]; then
  echo '[training] Set 2D Pose Estimate in RViz before arming or sending a goal.'
  echo '[training] status checks base health; it does not certify Nav2 localization/readiness.'
fi

if [[ $action == check ]]; then
  for package in rmw_cyclonedds_cpp ros_gz_sim ros_gz_bridge gz_ros2_control \
    slam_toolbox nav2_bringup nav2_map_server teleop_twist_keyboard studica_robot_monitor; do
    ros2 pkg prefix "$package" >/dev/null || fail "Missing package: $package"
  done
  command -v gz >/dev/null || fail 'Gazebo is not installed.'
fi
if [[ $action == sim || $action == slam || $action == nav ]]; then
  mkdir -p "$session_dir"
  exec 9>"$session_dir/session.lock"
  flock -n 9 || fail 'Another training launch is active. Stop it with Ctrl+C before switching.'
fi
if [[ $action == nav && -n $map_name ]]; then
  [[ -r $map_dir/map.yaml && -r $map_dir/map.pgm ]] || fail "Saved map pair missing: $map_dir"
fi
if [[ $action == save-map ]]; then
  mkdir -p "$workspace/project_maps/training"
  # Exclusive creation prevents overwriting a previous map, including failed saves.
  mkdir "$map_dir" || fail 'Map name already exists; choose a new name.'
fi
exec "${command[@]}"
