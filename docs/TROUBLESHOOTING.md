# Troubleshooting

Use the first error, not the last line of a long launch log. Copy the exact
command, the first error, and the result of `robot_check` when asking for help.

## Start with a clean terminal

```bash
export STUDICA_WS="$HOME/ros2_ws"
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
  gui:=false gz_headless:=true
```

The pinned overlay must report this checkout:

```bash
git -C "$STUDICA_WS/src/gz_ros2_control" rev-parse HEAD
```

Expected:

```text
a2d290e37be67ba082744e323339d82031f051c0
```

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
`use_camera:=true` only for the camera lesson.

## The simulated robot does not move

Verify the controller and public command topic:

```bash
ros2 control list_controllers
ros2 topic info /cmd_vel --verbose
ros2 topic echo /cmd_vel
```

The base controller should be `active`; `/cmd_vel` should receive
`geometry_msgs/msg/Twist`. Use the external keyboard command from the README.
Do not publish to controller-internal stamped topics.

One command is intentionally temporary because the controller timeout stops the
robot. A continuous teleop or navigation node must publish repeatedly.

## RViz shows a TF error

List frames and inspect one transform:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base_link
```

Wait a few seconds after launch. For navigation, set the initial pose in RViz so
the `map` to `odom` transform can be established. Never add a second static
publisher merely to hide a missing transform; find which owner is absent.

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
