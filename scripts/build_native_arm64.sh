#!/usr/bin/env bash
# Build only. Never installs packages, starts ROS nodes, or changes a release pointer.
set -euo pipefail
if [[ $# != 1 ]]; then
  echo "usage: $0 /absolute/path/to/dedicated-hardware-workspace" >&2
  exit 2
fi
workspace="$(realpath "$1")"
source_root="$workspace/src/studica_vmxpi_ros2"
[[ "$(uname -m)" == aarch64 ]] || { echo 'Native ARM64 host required' >&2; exit 2; }
# shellcheck source=/dev/null
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 22.04 ]] || exit 2
[[ -z "${AMENT_PREFIX_PATH:-}" && -z "${COLCON_PREFIX_PATH:-}" && -z "${ROS_DISTRO:-}" ]] || {
  echo 'Use a fresh shell without ROS or student overlays' >&2; exit 2;
}
[[ -f /usr/local/include/vmxpi/VMXPi.h && -f /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so ]]
[[ -z "$(git -C "$source_root" status --porcelain --untracked-files=all)" ]] || {
  echo 'Commit reviewed source changes before a reproducible native build' >&2; exit 2;
}
python3 "$source_root/scripts/verify_hardware_checkout.py" --workspace "$workspace"
source_commit="$(git -C "$source_root" rev-parse HEAD)"
cd "$workspace"
for generated in build install log ydlidar-sdk-build vendor native-runtime-libraries.json; do
  [[ ! -e "$generated" && ! -L "$generated" ]] || {
    echo "Use a fresh workspace: $generated already exists; nothing removed" >&2; exit 2;
  }
done
# ROS setup scripts are not nounset-compatible.
set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=1 CTEST_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export LD_LIBRARY_PATH="/usr/local/lib/vmxpi:${LD_LIBRARY_PATH:-}"
cmake -S src/YDLidar-SDK -B ydlidar-sdk-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$workspace/vendor/ydlidar_sdk" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DSWIG_FOUND=FALSE -DPYTHONLIBS_FOUND=FALSE \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=TRUE
nice -n 10 cmake --build ydlidar-sdk-build --parallel 1
cmake --install ydlidar-sdk-build
export CMAKE_PREFIX_PATH="$workspace/vendor/ydlidar_sdk:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$workspace/vendor/ydlidar_sdk/lib:$LD_LIBRARY_PATH"
export LIBRARY_PATH="$workspace/vendor/ydlidar_sdk/lib:${LIBRARY_PATH:-}"
nice -n 10 colcon build --executor sequential --parallel-workers 1 --merge-install \
  --packages-ignore ydlidar_sdk --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DSTUDICA_PRODUCTION_INSTALL=ON
python3 "$source_root/scripts/verify_hardware_checkout.py" --workspace "$workspace"
[[ "$(git -C "$source_root" rev-parse HEAD)" == "$source_commit" && \
   -z "$(git -C "$source_root" status --porcelain --untracked-files=all)" ]] || {
  echo 'Platform source changed during build; artifacts are not release inputs' >&2; exit 2;
}
python3 "$source_root/scripts/inspect_runtime_libraries.py" \
  --install-prefix "$workspace/install" \
  --library-root ros=/opt/ros/humble/lib \
  --library-root system=/usr/lib/aarch64-linux-gnu \
  --library-root vmx=/usr/local/lib/vmxpi \
  --output "$workspace/native-runtime-libraries.json"
vcs export --exact src > native-sources.repos
dpkg-query -W -f='${Package}\t${Version}\t${Architecture}\n' > native-dpkg-inventory.tsv
printf '%s\n' 'Build complete; no runtime started or release activated.'
