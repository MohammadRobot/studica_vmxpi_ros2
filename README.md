# Studica ROS 2 Classroom Robot

Learn ROS 2 by driving and programming one robot in simulation before touching
real hardware. The `class_4wd` beginner API is identical in simulation and hardware.

## What you will learn

You will learn how to:

- launch a ROS 2 system and inspect its nodes, topics, and TF frames;
- drive safely through `/cmd_vel` using `geometry_msgs/msg/Twist`;
- read wheel encoders, odometry, IMU, LiDAR, and camera data;
- write Python publishers, subscribers, services, parameters, and launch files;
- understand `ros2_control`, controller state, and robot diagnostics;
- create a map with SLAM Toolbox and navigate with Nav2;
- check a real robot without bypassing its safety gates.

No robotics experience is required. Lab 1 introduces the required terminal skills.

## Supported classroom platform

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Harmonic
- `class_4wd` robot profile
- Python 3 for student programs

The Humble `gz_ros2_control` overlay is pinned for Harmonic; do not replace it.

## Safety boundary

Simulation is the safe default. Real hardware requires an instructor, a clear
work area, and a reachable physical emergency stop.

Setup, tests, checks, mapping, and navigation never command motion automatically.
Never run hardware commands on a desk or with people near the wheels.

## Fresh installation

Choose a workspace location once. This README uses `STUDICA_WS`, so no username
or machine-specific path is built into the commands.

```bash
export STUDICA_WS="$HOME/ros2_ws"
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
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Expected result: Gazebo opens with the four-wheel robot in a maze, RViz shows
the robot and sensor data, and the terminal reports active controllers.

Terminal 2 — inspect the system:

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 node list
ros2 topic list
ros2 topic hz /scan
```

Press `Ctrl+C` after observing the scan rate.

Terminal 3 — drive slowly with the keyboard:

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Keep this terminal focused so it receives your keys. Start at the displayed
default speed, avoid walls, and press the space bar to stop. The robot also
stops when command messages time out.

When finished, press `Ctrl+C` in the teleop terminal and then in the launch
terminal. Confirm that Gazebo closes before starting another launch.

## One public motion interface

Student programs publish only this message:

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
that detail so student code stays portable. Do not publish directly to a
controller's internal command topic.

## Essential robot topics

| Topic | Message type | Meaning |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Requested robot speed |
| `/odom` | `nav_msgs/msg/Odometry` | Estimated motion and pose |
| `/imu` | `sensor_msgs/msg/Imu` | Orientation, acceleration, and rotation |
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
| `sim.launch.py` | Maze simulation and RViz | No |
| `mapping.launch.py` | Office simulation and SLAM Toolbox | No |
| `navigation.launch.py` | Office simulation, saved map, and Nav2 | No |
| `robot.launch.py` | Supervised `class_4wd` hardware | No |

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

- [Documentation index](docs/README.md)
- [Installation](docs/INSTALL.md)
- [Course and labs](docs/COURSE.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Mapping and navigation](docs/MAPPING_NAVIGATION.md)
- [Supervised hardware](docs/HARDWARE.md)
- [Networking and remote Foxglove](docs/NETWORKING.md)
- [Advanced robot profiles](docs/PROFILE_AUTHORING.md)
- [Instructor guide](docs/INSTRUCTOR_GUIDE.md)

## Repository roles

- `studica_vmxpi_ros2`: student launches, robot model, `ros2_control`, and labs
- `studica_robot_monitor`: diagnostics, `robot_check`, recordings, and guarded validation
- `studica_drivers`: low-level VMXPi/Titan infrastructure
- `studica_ros2_control`: optional accessory components for advanced projects
- Orbbec and YDLidar repositories: unchanged vendor sensor drivers

Students normally work in this repository. The other repositories are
dependencies, not separate course entry points.

## Check a change

The classroom checker never starts a simulator or robot:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/check_project.sh
```

Before sharing a change, build and run tests from the workspace root:

```bash
cd "$STUDICA_WS"
colcon build --symlink-install
colcon test --packages-select \
  studica_drivers studica_robot_monitor studica_ros2_control studica_vmxpi_ros2
colcon test-result --verbose
```

Tests and setup must never initiate motor motion. Physical regression tests are
separate, supervised activities documented for instructors.
