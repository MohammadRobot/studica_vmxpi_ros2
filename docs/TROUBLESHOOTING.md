# Troubleshooting

Use the first error, not the last line of a long launch log. Copy the exact
command, the first error, and the result of `robot_check` when asking for help.

## Start with a clean terminal

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"  # simulation; use the selected peer env for hardware
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
printenv ROS_DISTRO
ros2 pkg prefix studica_vmxpi_ros2
```

Expected: `humble` and a path under `$STUDICA_WS/install`. If `source` says the
file is missing, the workspace has not built successfully.

## Package or launch file not found

Rebuild, then source the result again:

```bash
cd "$STUDICA_WS"
colcon build --symlink-install --packages-select studica_vmxpi_ros2
source install/setup.bash
ros2 launch studica_vmxpi_ros2 sim.launch.py --show-args
```

Do not source another ROS distribution or an older workspace afterward; later
setup files take precedence.

## Colcon cannot create a symbolic link

An error such as:

```text
failed to create symbolic link ... because existing path cannot be removed:
Is a directory
```

means the workspace was previously built with normal copied installs and a
later command switched to `--symlink-install`. It is not an Orbbec or source-code
failure. Do not alternate build modes in the same generated trees. For an
existing copied-install workspace, reuse its original mode:

```bash
cd "$STUDICA_WS"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
colcon build
```

A fresh workspace created by `setup_ubuntu.sh` uses `--symlink-install` and
should keep that flag. Intentionally converting an existing workspace requires
a separate clean rebuild; preserve application source and generated maps before
doing so.

## `diagnostic_aggregator` or another runtime package is missing

The package manifests declare runtime dependencies, but `colcon build` does not
install their Ubuntu packages. Resolve the whole workspace from a sourced ROS
terminal:

```bash
sudo apt update
rosdep install --from-paths "$STUDICA_WS/src" --ignore-src \
  --rosdistro humble -r -y
source "$STUDICA_WS/install/setup.bash"
```

Then verify the package named by the first launch error:

```bash
ros2 pkg prefix diagnostic_aggregator
```

No rebuild is normally needed for a newly installed runtime package. See
[Installation](INSTALL.md#recover-missing-system-dependencies).

## APT reports conflicting `Signed-By` values for the ROS repository

Current ROS installations can use `/etc/apt/sources.list.d/ros2.sources`,
managed by the `ros2-apt-source` package with an embedded signing key. An older
`ros2.list` entry for the same URL conflicts with that managed source.

The project installer detects this exact combination and moves the legacy file
to `/etc/apt/ros2.list.disabled-by-studica-setup` before continuing. The file
is preserved outside `sources.list.d` so the change is reversible without
causing an APT filename warning. Rerun the installer normally:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode hardware
```

Do not delete signing keys or paste an inline key into another source file. If
setup reports that the backup already exists, inspect the two named files and
resolve them deliberately rather than overwriting the backup.

## A build reports “compute resource limit exceeded”

Close Gazebo, RViz, Foxglove, browsers, and camera programs. Build sequentially:

```bash
cd "$STUDICA_WS"
export MAKEFLAGS="-j1"
colcon build --symlink-install --executor sequential \
  --parallel-workers 1
```

On a VMXPi, build only the needed first-party packages and keep the camera off
until the build finishes. Do not run a large build while controlling motors.

Check memory and disk:

```bash
free -h
df -h "$STUDICA_WS"
```

## Gazebo does not open

Confirm the Harmonic executable and bridge packages:

```bash
gz sim --versions
ros2 pkg prefix ros_gz_sim
ros2 pkg prefix gz_ros2_control
```

If `gz` is missing, rerun the simulation installer. If the computer has no
display, use the headless argument:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py \
  gui:=false gz_headless:=true use_joystick:=false
```

The pinned overlay must report this checkout:

```bash
git -C "$STUDICA_WS/src/gz_ros2_control" rev-parse HEAD
```

Expected:

```text
a2d290e37be67ba082744e323339d82031f051c0
```

An error looking for `ignition-gazebo6Config.cmake` on this Harmonic setup
means `GZ_VERSION` was missing during a manual build. Rebuild the affected
packages with:

```bash
export GZ_VERSION=harmonic
cd "$STUDICA_WS"
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install \
  --packages-select gz_ros2_control studica_vmxpi_ros2 \
  --allow-overriding gz_ros2_control
```

## Cyclone DDS rejects the configured interface

Errors containing `does not match an available interface`, `failed to create
domain`, or `rcl node's rmw handle is invalid` usually come from a stale
`CYCLONEDDS_URI`, not from the node named in the stack trace.

The setup script runs its non-motion tests with local-only Cyclone DDS so an
operator's hardware-network profile cannot invalidate installation tests. It
also regenerates test-result XML on each run. This isolation affects only the
installer process and does not replace the runtime DDS configuration.

Compare the configured address with the machine:

```bash
ip -br address
printenv CYCLONEDDS_URI ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION ROS_DOMAIN_ID
grep -n 'CYCLONEDDS_URI\|setup.bash' "$HOME/.bashrc" "$HOME/.profile" 2>/dev/null
```

For local simulation, generate and source the dedicated loopback environment,
then restart discovery:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/configure_cyclonedds.py sim --domain-id 1
source "$HOME/.ros/studica_sim.env"
ros2 daemon stop
ros2 daemon start
```

For robot Wi-Fi or Ethernet, rerun `configure_cyclonedds.py peer` on each
machine with its current addresses. Add `--force` only after reviewing an
intentional address change. The helper rejects a local address that is not
assigned to the named interface and prints the domain-specific UFW range.

If `ros2 daemon stop` hangs because the old daemon started with the broken
profile, inspect it before terminating only that exact process:

```bash
pgrep -a -f 'ros2cli.daemon.daemonize'
kill -TERM <DAEMON_PID>
ros2 daemon start
```

Do not use a broad ROS or Python process-kill command; the simulator or another
application nodes may be running.

If a shell startup file exports the hardware profile, remove that line and open
a new login session. Source the generated lesson environment only in terminals
that need it. For a deterministic local profile that supports the simulator's
many processes, use
[Keep simulation and robot DDS profiles separate](NETWORKING.md#keep-simulation-and-robot-dds-profiles-separate).

## A topic exists but prints no data

Check publishers and rate:

```bash
ros2 topic info /scan --verbose
ros2 topic hz /scan
```

Some sensor topics use best-effort QoS. Match that policy for a command-line
check:

```bash
ros2 topic echo /scan --once --qos-reliability best_effort
```

Camera topics are absent by default. Relaunch simulation with
`use_camera:=true` for RGB/depth images or `use_point_cloud:=true` to enable the
camera and publish `/camera/depth/points` plus the ground-referenced
`/camera/depth/points_filtered`. If only the raw topic exists, verify the
`base_link <- camera_optical_frame` transform. Point-cloud subscribers should
use best-effort sensor QoS. See [Camera and point cloud](CAMERA_POINT_CLOUD.md).

On physical hardware, `robot.launch.py` intentionally starts depth-only
320×240 at 5 Hz. Add `use_camera_color:=true` only when RGB is required, or
`use_point_cloud:=true` to start both the Gemini E cloud and the floor filter.
If `vcgencmd get_throttled` contains active low-order bit `0x8`, stop camera
processing and cool the VMXPi; reducing message size alone does not establish a
sustained thermal pass.

If navigation publishes both clouds but ignores them, inspect the local layer:

```bash
ros2 param get /local_costmap/local_costmap \
  voxel_layer.observation_sources
ros2 topic info /camera/depth/points_filtered --verbose
```

The default list includes `scan depth_mark depth_clear`, and the local costmap
node should subscribe to the filtered topic. A custom Nav2 file must contain a
configured local VoxelLayer or ObstacleLayer for the overlay to target. Use
`navigation.launch.py use_point_cloud:=false` to isolate LiDAR behavior.

## The simulated robot does not move

Verify the controller and public command topic:

```bash
ros2 control list_controllers
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
```

The base controller should be `active`; `/cmd_vel` should receive
`geometry_msgs/msg/Twist`. `sim.launch.py` and `mapping.launch.py` start the
deadman-protected joystick nodes by default; hold L1 while moving a stick. For
keyboard control, relaunch with `use_joystick:=false` so only one teleop source
owns `/cmd_vel`. Do not publish to controller-internal stamped topics.

One command is intentionally temporary because the controller timeout stops the
robot. A continuous teleop or navigation node must publish repeatedly.

For joystick control, check the signal in order:

```bash
ls -l /dev/input/js*
ros2 topic echo /joy --field axes
ros2 topic echo /cmd_vel
ros2 topic info /cmd_vel --verbose
```

If `/joy` repeats neutral values while the sticks move, stop and restart
`joy_node`; a Bluetooth reconnect can leave it attached to the previous device
handle. If `/joy` changes but `/cmd_vel` stays zero, hold the configured deadman
button and verify the axis/button indexes. Stop keyboard teleop and other
external motion publishers during the joystick test. Follow the complete
[Joystick teleoperation](JOYSTICK.md) guide.

## RViz shows a TF error

List frames and inspect one transform:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

Wait a few seconds after launch. For navigation, set the initial pose in RViz so
the `map` to `odom` transform can be established. Never add a second static
publisher merely to hide a missing transform; find which owner is absent.

## Mapping walls move or appear more than once

Do not recalibrate wheel radius from a distorted map. First compare a measured
straight drive with `/odom`; a small distance error cannot explain large wall
duplication. Also remember that `map -> odom` is expected to change while SLAM
corrects local odometry. The failure is when a stationary wall slides or is
drawn repeatedly in the fixed `map` frame.

For the physical robot, start a fresh PC session with hardware mode and confirm
the X2-tuned profile is active:

```bash
ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=hardware

# In another PC terminal:
ros2 param get /slam_toolbox throttle_scans
ros2 param get /slam_toolbox resolution
ros2 param get /slam_toolbox minimum_travel_distance
ros2 param get /slam_toolbox use_scan_matching
ros2 param get /slam_toolbox do_loop_closing
```

Expected values are `1`, `0.05`, `0.02`, `False`, and `True`. The physical
profile trusts the separately calibrated wheel/IMU odometry for scan placement
and applies strict thresholds to loop closure. This prevents noisy local matches
from accumulating a large `map -> odom` rotation while still allowing a strong
return-to-start match to correct residual drift. If the values differ, the PC has not been
rebuilt/sourced or an old mapping process is still running. Stop it, rebuild
`studica_vmxpi_ros2`, source the current install, and restart mapping.

Use normal mode rather than turbo for the first map. Apply the sticks gently,
release L1 briefly before and after turns, then approach the starting area from
approximately the original direction and remain stopped for 2--3 seconds before
saving. One small map adjustment can indicate successful loop closure. Restart
the PC mapping launch after a badly distorted attempt because
SLAM cannot reliably repair a map built from many incorrect scan matches.

If walls still slide during straight motion, stop the robot and check the
physical LiDAR mount before tuning scan matcher penalties: the sensor must not
rotate or vibrate relative to `base_link`. Then verify one `/scan` publisher,
one `/odom` publisher, and one `map -> odom` publisher:

```bash
ros2 topic info /scan --verbose
ros2 topic info /odom --verbose
ros2 topic info /tf --verbose
timeout 10 ros2 topic hz /scan
timeout 10 ros2 topic hz /odom
```

The X2 should supply approximately 10--12 Hz and hardware odometry approximately
25 Hz. Multiple owners or a missing rate must be corrected before another map
is recorded.

## YDLidar reports an incompatible `/scan` QoS subscriber

The YDLidar driver intentionally publishes `/scan` with ROS sensor-data QoS:
best-effort reliability and volatile durability. A newly discovered subscriber
that requests reliable delivery is incompatible with that publisher, so only
that endpoint receives no scans. Other best-effort subscribers continue to
work; successful AMCL localization and a moving red scan in RViz prove the
navigation path is receiving data.

Identify the incompatible endpoint:

```bash
ros2 topic info /scan --verbose
```

The project RViz scan display uses `Best Effort`. For a manual sample, request
the sensor-data profile explicitly:

```bash
ros2 topic echo /scan --once --qos-profile sensor_data \
  --field header.frame_id
```

Stop or reconfigure an optional subscriber shown as `Reliability: RELIABLE`.
Do not change the YDLidar publisher to reliable or lower its confirmed 10 Hz
scan frequency merely to silence the warning.

## The hardware EKF misses its update rate

`hardware_odometry_filter` normally follows `hardware_control_rate_hz`, which
is 25 Hz in the supported physical launch. A warning such as `Failed to meet
update rate` means one cycle took longer than its 40 ms budget. An isolated
warning does not stop navigation, but repeated delays of hundreds of
milliseconds indicate CPU, USB, DDS, or network contention.

Check the actual data rate and computer load:

```bash
ros2 param get /hardware_odometry_filter frequency
timeout 10 ros2 topic hz /odom
ps -eo pid,%cpu,%mem,args --sort=-%cpu | head -n 10
sudo vcgencmd measure_temp
sudo vcgencmd get_throttled
```

When raw `/camera/depth/points` is displayed remotely, first disable the RViz
`Raw PointCloud` display and repeat the test. If warnings continue, restart the
VMXPi hardware launch with `use_camera:=false use_point_cloud:=false`. Keep
LiDAR enabled for navigation. Do not hide sustained overload by increasing
diagnostic thresholds or raising the control rate.

## The navigation map is grayscale

That is normal for the raw occupancy grid: black is occupied, white is free,
and gray is unknown. The project Nav2 RViz configuration keeps `/map` in the
`map` color scheme and displays `/global_costmap/costmap` plus
`/local_costmap/costmap` with the colored `costmap` scheme. Cyan, red, and
purple bands are inflation costs around obstacles, not physical map colors.

If all three layers appear grayscale, select **Global Costmap** and **Local
Costmap** in RViz and set **Color Scheme** to `costmap`. This changes only the
display, not planning behavior.

## `robot_check` reports FAIL

```bash
ros2 run studica_robot_monitor robot_check --mode auto
echo "Exit code: $?"
```

- `0`: all required checks pass; warnings may still need attention.
- `1`: at least one required runtime check failed.
- `2`: command usage or the local ROS setup is invalid.

Fix required failures from top to bottom. A stale sensor often explains several
downstream warnings. Use `--strict` only after the normal report is understood.

When hardware was deliberately launched with `use_lidar:=false`, run:

```bash
ros2 run studica_robot_monitor robot_check \
  --mode hardware --timeout 5 --skip-lidar
```

Do not use `--skip-lidar` to hide a failed LiDAR that was expected to be
running. The option skips the scan topic, LiDAR TF, and LiDAR diagnostic as one
explicit test configuration.

If `Robot/Compute/Pi` warns while `ros2_control_node` is running at 100 Hz,
return to the supported hardware default and verify it:

```bash
ros2 param get /controller_manager update_rate
# Expected from robot.launch.py: Integer value is: 25
```

Stop hardware bringup before changing the rate; launch again with
`hardware_control_rate_hz:=25`. Never raise the temperature warning to conceal
excess load.

## VMX HAL permission or pigpio error

Messages such as `you don't have permission`, `Error initializing pigpio`, or
`System Resources ... not available` mean hardware bringup was not started with
the VMXPi root environment. Stop the failed launch; do not retry motor commands.

Use the exact supervised root launch in [Hardware](HARDWARE.md). Running only one
node with an incomplete `sudo` environment can load the wrong libraries or lose
the workspace overlay.

## Foxglove handshake failed

Use a Foxglove WebSocket connection, not a ROS 2 native connection or an HTTP
browser tab:

```text
ws://<ROBOT_IP>:8765
```

The bridge must bind to the robot's actual trusted-LAN address. Confirm it is
listening:

```bash
ss -ltn | grep 8765
```

Repeated `handshake failed` messages usually mean the wrong client protocol,
wrong URL, a proxy, or a port scan. See [Networking](NETWORKING.md).

## Remote ROS 2 topics are absent

First prove ordinary IP reachability in both directions, then DDS multicast.
Both computers need the same `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0`, and RMW
implementation. Do not put experimental DDS settings in `.bashrc`.

Follow the ordered checks in [Networking](NETWORKING.md). Foxglove can show the
robot without exposing DDS discovery to the whole laptop and is the recommended
read-only dashboard path.

## Stop and clean up

Press `Ctrl+C` once and wait for shutdown. If a simulator remains after a failed
launch, inspect it before ending anything:

```bash
ps -ef | grep -E 'gz sim|ruby|rviz2|ros2_control_node' | grep -v grep
```

Never use a broad process-kill command on a shared robot computer. Ask the
instructor if a hardware process does not exit cleanly.
