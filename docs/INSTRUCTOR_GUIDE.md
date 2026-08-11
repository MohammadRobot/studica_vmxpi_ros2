# Instructor Guide

This guide separates normal learner work from operations that require hardware,
network, or profile authority. Learners spend Labs 1–8 in simulation. Lab 9 is
optional and supervised.

## Course invariants

- Core robot: `class_4wd`.
- Application language: Python 3.
- Application motion API: `/cmd_vel` with `geometry_msgs/msg/Twist`.
- Standard feedback: `/odom`, `/imu`, `/scan`, `/joint_states`, and TF.
- Simulation and mapping start one L1-deadman joystick publisher by default.
- Navigation keeps joystick off and does not move until a human sends a goal.
- Setup, CI, checks, and documentation tests never initiate motor motion.
- Titan autotune is absent from the learning path.

Do not teach controller-internal command topics as alternatives. The adapter is
there so every lesson and robot variant keeps the same public API.

## Prepare development PCs

On a clean Ubuntu 22.04 image:

```bash
export STUDICA_WS="$HOME/studica_ws"
mkdir -p "$STUDICA_WS/src"
git clone https://github.com/MohammadRobot/studica_vmxpi_ros2.git \
  "$STUDICA_WS/src/studica_vmxpi_ros2"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode simulation --non-interactive
```

Run the same command twice when qualifying the image. Verify no `.bashrc`
change, duplicate repository entry, or overlay drift occurs.

For each prepared PC:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py \
  gui:=false gz_headless:=true use_joystick:=false
```

In another terminal, require:

```bash
ros2 run studica_robot_monitor robot_check --mode simulation --strict
```

Then test the graphical launch, keyboard focus, and available GPU performance.
Camera is intentionally off; enable it only on computers that can sustain the
lesson without rate or memory warnings.

## Classroom repository layout

- `studica_robot_apps`: the normal developer-owned application package;
- `studica_vmxpi_ros2`: platform, simulation, hardware safety, and course material;
- `studica_robot_monitor`: installed course health tools;
- `studica_drivers`: low-level infrastructure, not a beginner API;
- `studica_ros2_control`: optional accessory components for advanced work;
- vendor camera/LiDAR repositories: hardware dependencies, left unchanged.

Avoid assigning edits inside generated `build`, `install`, or `log` trees.
Python application work belongs in `studica_robot_apps` under the workspace
`src` directory. Keep the platform and vendor repositories unchanged during
normal application development.

## Lab delivery

Each lab has a prerequisite gate, named terminals, expected output, checkpoint,
cleanup, and challenge. Do not skip cleanup: many apparent ROS failures are old
launches publishing the same topic.

Suggested pacing:

| Lab | Sessions | Main evidence |
|---|---:|---|
| 1 | 1 | workspace and vocabulary check |
| 2 | 1 | annotated node/topic graph |
| 3 | 1 | safe teleop and Twist explanation |
| 4 | 1–2 | sensor/TF/QoS worksheet |
| 5 | 2–3 | Python package and tests |
| 6 | 1–2 | controller/odometry/diagnostic report |
| 7 | 1–2 | saved map and artifact analysis |
| 8 | 1–2 | localized goal, colored costmaps, and waypoint route |
| 9 | supervised | signed readiness and baseline report |

Reference solutions are separate from TODO starters. Release them after a
checkpoint or use them for instructor validation, not as the initial learner
workspace.

## Resource-limited computers

Keep beginner sensor rates at the launch defaults. Leave camera and point cloud
off outside the camera exercise. If a build exceeds resources:

```bash
export MAKEFLAGS="-j1"
colcon build --symlink-install --executor sequential --parallel-workers 1
```

Do not build while a VMXPi is controlling motors. Schedule compile time and
physical robot time as different activities.

## Network planning

Prefer isolated team networks and assign one `ROS_DOMAIN_ID` per team. Keep DDS
overrides in lesson terminals rather than `.bashrc`. Foxglove is the default
remote observation path and remains read-only.

Record `<ROBOT_IP>`, `<LAPTOP_IP>`, and interfaces on the team worksheet. Do not
publish fixed school addresses in the repository. See [Networking](NETWORKING.md).

## Hardware readiness gate

Before Lab 9, the instructor must have:

- a supported arm64 VMXPi image and working HAL;
- measured and reviewed `drive.wheel_radius_m`;
- a tested physical emergency stop and stable wheel lift;
- Titan firmware with confirmed MCV2 PID support;
- four fresh encoder channels with correct signs;
- a healthy read-only `robot_check` report;
- a known-safe battery, wiring, CAN, and test area;
- a report storage and incident procedure.

Use [Supervised hardware](HARDWARE.md) exactly. A wheel-radius calibration flag
is evidence of measurement, not a workaround. A latched hardware fault requires
cause investigation and deliberate reactivation.

## Software acceptance before physical regression

From the workspace root:

```bash
./src/studica_vmxpi_ros2/scripts/check_project.sh
colcon build --symlink-install --packages-select \
  studica_drivers studica_robot_monitor studica_ros2_control studica_vmxpi_ros2
colcon test --packages-select \
  studica_drivers studica_robot_monitor studica_ros2_control studica_vmxpi_ros2
colcon test-result --verbose
```

Also require:

- documentation links resolve;
- shell scripts parse and pass ShellCheck in CI;
- YAML, XML, and Xacro inputs parse;
- no ELF binaries, caches, backup files, or generated maps are tracked;
- headless launch tests see active controllers and standard topics;
- timeout testing confirms motion commands decay to zero;
- mapping has one deadman teleop publisher and navigation has no non-Nav2
  command publisher;

If Harmonic cannot run on the build host, mark the graphical/headless simulator
test as an environmental limitation and run it on the qualified classroom PC.
Do not substitute a physical motor run for missing simulation evidence.

## Physical regression order

Only after software acceptance:

1. mechanical/electrical inspection with motor power off;
2. root HAL launch with camera and Foxglove off;
3. read-only controllers, diagnostics, PID, temperature, and latch checks;
4. guarded lifted-wheel validation and report review;
5. low-speed measured straight-line test;
6. low-speed measured rotation calibration;
7. final lifted validation after geometry/inversion changes;
8. baseline MCAP/Foxglove recording and sign-off.

Documentation-only or artifact cleanup does not require rerunning physical
motors. Any change to command conversion, inversion, PID, timeouts, hardware
state, or profile defaults does.

## Advanced variants

`class_2wd`, `class_mecanum`, and `class_omni` are retained for an advanced
variants lesson. They are not implicit defaults and should not be introduced
before developers understand `/cmd_vel`, TF, and `ros2_control`. Use
[Advanced profiles](PROFILE_AUTHORING.md) and create a separate acceptance sheet
for any physical variant.
