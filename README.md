# Studica ROS 2 Classroom Robot
Learn ROS 2 by driving and programming one robot in simulation before touching
real hardware. No robotics experience is required; Lab 1 introduces terminal
skills. The `class_4wd` beginner API is identical in simulation and hardware.

## What you will learn

You will learn how to:

- launch a ROS 2 system and inspect its nodes, topics, and TF frames;
- drive safely through `/cmd_vel` using `geometry_msgs/msg/Twist`;
- read wheel encoders, odometry, IMU, LiDAR, and camera data with hardware sensor fusion;
- write Python publishers, subscribers, services, parameters, and launch files;
- understand `ros2_control`, controller state, and robot diagnostics;
- create a map with SLAM Toolbox and navigate with Nav2;
- check a real robot without bypassing its safety gates.

## Supported classroom platform

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Harmonic
- `class_4wd` robot profile
- Python 3 for application development

The Humble `gz_ros2_control` overlay is pinned for Harmonic; do not replace it.
The default simulation and hardware launches share the measured `class_4wd`
body, wheel placement, sensor transforms, and navigation footprint.

## Safety boundary

Simulation is the safe default. Real hardware requires an instructor, a clear
work area, and a reachable physical emergency stop.

Joystick-enabled simulation remains stationary until L1 is held and a stick is
moved. Never run hardware commands on a desk or with people near the wheels.

## Fresh installation

Use the dedicated `~/studica_ws` workspace shown below. Keeping this project out
of a general ROS workspace prevents stale overlays from shadowing pinned packages.

```bash
export STUDICA_WS="$HOME/studica_ws"
mkdir -p "$STUDICA_WS/src"
git clone https://github.com/MohammadRobot/studica_vmxpi_ros2.git \
  "$STUDICA_WS/src/studica_vmxpi_ros2"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode simulation
```

The idempotent installer imports pinned dependencies, builds, and runs non-motion
validation. It does not edit `.bashrc`.

Preview the checks without changing the computer:

```bash
./scripts/setup_ubuntu.sh --mode simulation --check-only
```

See [Installation](docs/INSTALL.md) for modes, non-interactive use, and common
installation problems.

## Five-minute simulation

Every terminal must source ROS 2 and this workspace. Replace the workspace value
only if you chose a different directory.

Terminal 1 — launch the maze, robot, controllers, sensors, and RViz:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Expected result: Gazebo opens with the four-wheel robot in a maze, RViz shows
sensor data, the controllers become active, and `/joy` is available.

Terminal 2 — inspect the system:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 node list
ros2 topic list
ros2 topic hz /scan
```

Drive with the connected DualShock: hold L1, use the left stick vertically, and
use the right stick horizontally. R1 enables turbo while L1 remains held. To use
keyboard teleop instead, relaunch with `use_joystick:=false` and follow the
[joystick guide](docs/JOYSTICK.md) motion-publisher rules.

When finished, release L1 and press `Ctrl+C` in the launch terminal. Confirm
that Gazebo closes before starting another launch.

## One public motion interface

Application nodes publish motion only through this message:

```text
Topic: /cmd_vel
Type:  geometry_msgs/msg/Twist
```

Example one-second forward command followed by automatic timeout:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Internal controller topics vary by drive controller. A project adapter owns
that detail so application code stays portable. Do not publish directly to a
controller's internal command topic.

## Essential robot topics

| Topic | Message type | Meaning |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Requested robot speed |
| `/odom` | `nav_msgs/msg/Odometry` | Estimated motion and pose |
| `/imu` | `sensor_msgs/msg/Imu` | Orientation plus sensor-frame acceleration and rotation |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR distance scan |
| `/joint_states` | `sensor_msgs/msg/JointState` | Wheel position and velocity |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Coordinate-frame relationships |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Component health reports |
| `/robot_status/motors` | `studica_robot_monitor/msg/MotorTelemetryArray` | Detailed hardware motor state |

Camera topics are disabled in the beginner simulation to reduce computer load.
Enable them only when needed:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py use_camera:=true
```

## Five useful checks

```bash
ros2 control list_controllers
ros2 topic hz /odom
ros2 topic echo /imu --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run studica_robot_monitor robot_check --mode simulation
```

`robot_check` is read-only. PASS and WARN return exit code `0`; a missing
required component returns `1`; a usage or setup error returns `2`. Add
`--strict` when a warning should fail an automated classroom check.

## Beginner launches

| Launch file | Purpose | Starts a motion publisher? |
|---|---|---|
| `sim.launch.py` | Maze simulation and RViz | Joystick, L1 deadman required |
| `mapping.launch.py` | Office simulation or PC-side real-robot SLAM | Joystick, L1 deadman required |
| `navigation.launch.py` | Office simulation or PC-side physical Nav2 | Nav2 only; joystick off |
| `robot.launch.py` | Supervised `class_4wd` hardware | No; joystick opt-in |

`bringup.launch.py` remains available for instructors and advanced robot
profiles. Beginners should use the four small launch files above.

## Course path

Work through the labs in order:

1. Terminal, workspace, safety, and ROS vocabulary
2. Simulation and the ROS graph
3. Topics, teleoperation, and `/cmd_vel`
4. Sensors, TF, RViz, and QoS
5. Python nodes, parameters, services, and launch
6. `ros2_control`, odometry, and diagnostics
7. SLAM and map saving
8. Nav2 localization and goals
9. Supervised hardware readiness and low-speed control

Start at the [Course and lab index](docs/COURSE.md). Each lab identifies its
terminals, expected output, checkpoint, cleanup, and optional challenge. Python
TODO starters and separate solutions are under `examples/python/`.

## Real robot access

Hardware setup and motion require instructor supervision. The real robot adds
VMXPi root permissions, a measured wheel radius, Titan 2 MCV2 velocity PID,
encoder freshness checks, temperature limits, fault latching, and a physical
emergency stop.

Do not guess hardware values or disable a safety check to make a launch pass.
Follow [Supervised hardware](docs/HARDWARE.md), run the read-only health check,
and use the guarded lifted-wheel validator before any floor test.

## Documentation

- [Quick start](docs/QUICK_START.md)
- [Documentation index](docs/README.md)
- [Installation](docs/INSTALL.md)
- [Launch arguments](docs/LAUNCH_ARGUMENTS.md)
- [Course and labs](docs/COURSE.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Application development and deployment](docs/DEVELOPMENT.md)
- [Mapping and navigation](docs/MAPPING_NAVIGATION.md)
- [Supervised hardware](docs/HARDWARE.md)
- [Networking, Cyclone DDS, and remote Foxglove](docs/NETWORKING.md)
- [Advanced robot profiles](docs/PROFILE_AUTHORING.md)
- [Instructor guide](docs/INSTRUCTOR_GUIDE.md)

## Repository roles

- `studica_robot_apps`: developer-owned behaviors, launch files, config, and tests
- `studica_vmxpi_ros2`: robot model, bringup, `ros2_control`, safety, and labs
- `studica_robot_monitor`: diagnostics, `robot_check`, recordings, and guarded validation
- `studica_drivers`: low-level VMXPi/Titan infrastructure
- `studica_ros2_control`: optional accessory components for advanced projects
- Orbbec and YDLidar repositories: unchanged vendor sensor drivers

Developers normally edit only `studica_robot_apps`. The other repositories form
the platform and its dependencies; see the [development workflow](docs/DEVELOPMENT.md).

## Check a change

The classroom checker never starts a simulator or robot:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/check_project.sh
```

Before sharing a change, build and run tests from the workspace root:

```bash
cd "$STUDICA_WS"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --packages-select \
  studica_drivers studica_robot_monitor studica_ros2_control studica_vmxpi_ros2
colcon test-result --verbose
```

Keep the workspace's existing copied or symlink install mode consistent; see
[Installation](docs/INSTALL.md#manual-build-after-editing-code).

Tests and setup never initiate motion; physical regression is separate and
instructor-supervised.
