# Direct ARM64 build on VMXPi

This recipe is for the existing Ubuntu 22.04/Humble Studica image with the real
VMXPi SDK installed. It builds in a new directory and does not replace the tested
installation or enable motors. No Docker is used. First stop robot motion and
hardware processes as described in [ROBOT_MANUAL.md](ROBOT_MANUAL.md).

## 1. Preserve local source revisions on the PC

The tested monitor fix was committed locally and may not be on GitHub. Bundle it
and the product repository so the robot can build the exact local commits. Git
bundles do not include uncommitted edits: review and commit intended source changes
first; do not discard or blindly stage unrelated work.

**PC:**

```bash
cd "$HOME/studica_ws"
git -C src/studica_vmxpi_ros2 status --short
git -C src/studica_robot_monitor status --short
mkdir -p native-source-transfer
git -C src/studica_vmxpi_ros2 rev-parse HEAD > native-source-transfer/product.sha
git -C src/studica_robot_monitor rev-parse HEAD > native-source-transfer/monitor.sha
git -C src/studica_vmxpi_ros2 bundle create \
  "$HOME/studica_ws/native-source-transfer/product.bundle" --all HEAD
git -C src/studica_robot_monitor bundle create \
  "$HOME/studica_ws/native-source-transfer/monitor.bundle" --all HEAD
ssh vmx@192.168.1.173 'mkdir -p ~/native-source-transfer'
scp native-source-transfer/* vmx@192.168.1.173:~/native-source-transfer/
```

The monitor SHA must match `dependencies/hardware.repos` in the chosen product
revision. If not, resolve the intended dependency pin before building; do not
silently substitute a different monitor. Do not transfer private configuration
archives or signing keys with source.

## 2. Create a separate robot workspace

**Robot (SSH), fresh shell:**

```bash
source /opt/ros/humble/setup.bash
export NATIVE_WS="$HOME/studica-native-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$NATIVE_WS/src"
echo "$NATIVE_WS"
test -r /usr/local/include/vmxpi/VMXPi.h
test -r /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  python3-vcstool python3-rosdep python3-pytest python3-yaml

git clone "$HOME/native-source-transfer/product.bundle" "$NATIVE_WS/src/studica_vmxpi_ros2"
git -C "$NATIVE_WS/src/studica_vmxpi_ros2" checkout --detach \
  "$(cat "$HOME/native-source-transfer/product.sha")"
git clone "$HOME/native-source-transfer/monitor.bundle" "$NATIVE_WS/src/studica_robot_monitor"
git -C "$NATIVE_WS/src/studica_robot_monitor" checkout --detach \
  "$(cat "$HOME/native-source-transfer/monitor.sha")"
vcs import --skip-existing "$NATIVE_WS/src" \
  < "$NATIVE_WS/src/studica_vmxpi_ros2/dependencies/hardware.repos"
python3 "$NATIVE_WS/src/studica_vmxpi_ros2/scripts/verify_hardware_checkout.py" \
  --workspace "$NATIVE_WS"
```

Stop on missing SDK files or a verification failure. This is not an SDK installer;
retain the working vendor SDK and image. A missing local revision on GitHub is why
the two bundles are supplied before importing remaining pinned dependencies.

Install ROS dependency packages; no simulation stack is needed on Pi:

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update --rosdistro humble
rosdep install --from-paths "$NATIVE_WS/src" --ignore-src -y \
  --rosdistro humble --skip-keys 'gz_ros2_control ros_gz_bridge ros_gz_sim'
```

Resolve any dependency failure before compilation. Do not run the full simulation
installer on Pi. Keep sufficient free space for sources, build intermediates and
logs; inspect `df -h /` before starting. Do not delete the working release to make
room for an untested one.

## 3. Build the vendor LiDAR SDK and ROS packages

Same **Robot shell** (`NATIVE_WS` still set):

```bash
cd "$NATIVE_WS"
export CMAKE_BUILD_PARALLEL_LEVEL=1 CTEST_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
export LD_LIBRARY_PATH=/usr/local/lib/vmxpi
cmake -S src/YDLidar-SDK -B ydlidar-sdk-build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$NATIVE_WS/vendor/ydlidar_sdk" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TEST=OFF -DSWIG_FOUND=FALSE -DPYTHONLIBS_FOUND=FALSE \
  -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonInterp=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_PythonLibs=TRUE
nice -n 10 cmake --build ydlidar-sdk-build --parallel 1
cmake --install ydlidar-sdk-build
export CMAKE_PREFIX_PATH="$NATIVE_WS/vendor/ydlidar_sdk:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$NATIVE_WS/vendor/ydlidar_sdk/lib:$LD_LIBRARY_PATH"
export LIBRARY_PATH="$NATIVE_WS/vendor/ydlidar_sdk/lib"
source /opt/ros/humble/setup.bash
set -o pipefail
nice -n 10 ionice -c 2 -n 7 taskset -c 3 \
  colcon build --executor sequential --parallel-workers 1 --merge-install \
  --packages-ignore ydlidar_sdk --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DSTUDICA_PRODUCTION_INSTALL=ON \
  2>&1 | tee native-build.log
```

`taskset -c 3` is for this four-core Pi; select an existing core on different
hardware. The prior build took about 33 minutes for eight ROS packages plus the
SDK. This is a reference measurement, not a deadline. No root is needed to compile.
The production-install flag trims runtime assets; it does not activate a release.

## 4. Test without commanding hardware

Only continue after build success:

```bash
source "$NATIVE_WS/install/setup.bash"
export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=1
unset CYCLONEDDS_URI
colcon test --executor sequential --parallel-workers 1 --merge-install \
  --packages-select studica_drivers studica_robot_monitor studica_ros2_control studica_vmxpi_ros2 \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
python3 src/studica_vmxpi_ros2/scripts/verify_hardware_checkout.py --workspace "$NATIVE_WS"
vcs export --exact src > native-sources.repos
```

Review skipped tests: Gazebo contracts cannot run on a minimal Pi without Gazebo.
The earlier build also skipped cppcheck checks due to the ament/cppcheck version
guard. Software tests do not establish E-stop torque removal, boot safety or floor
navigation acceptance. Domain 99 is intentional; earlier use of domain 197 failed
classroom domain-range tests.

## 5. Try the new build and keep recovery available

Keep both old tested prefixes until the new build passes supervised hardware tests.
For a trial, use the manual launch in [ROBOT_MANUAL.md](ROBOT_MANUAL.md), replacing
the base and overlay source lines with the new workspace's `install/setup.bash`,
and replacing its vendor library path. The new build already includes the monitor,
so do not layer the old monitor over it. Set domain 78, the robot Wi-Fi XML and
`ROS_LOCALHOST_ONLY=0` again before connecting to the classroom runtime.

Stop every old hardware owner before launching the new one. Record the new source
SHAs, command, test outcome and logs. Rollback means stopping the trial and using
the old manual launch again. Do not replace `/opt/studica/current`, fabricate release
provenance or enable production services as part of a development build.

PC x86 builds are for PC. Release engineering can later build ARM64 artifacts on
a native worker or a qualified PC builder; the existing Docker-specific release
bundler is not a native-build activation command. See [RELEASE_PROCESS.md](RELEASE_PROCESS.md).
