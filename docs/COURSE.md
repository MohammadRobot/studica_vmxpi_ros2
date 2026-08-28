# ROS 2 Classroom Course

This course teaches ROS 2 and mobile robotics with one `class_4wd` robot. Labs
1–8 use simulation. Lab 9 is optional and may run only on instructor-approved
hardware with a safety operator beside the emergency stop.

## Before the first lab

Use Ubuntu 22.04, ROS 2 Humble, and Gazebo Harmonic. Complete
[Installation](INSTALL.md), including the second idempotence run, before class.
Each new terminal needs:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
```

Commands use `$STUDICA_WS` instead of a username-specific path. Do not type
placeholder angle brackets such as `<ROBOT_IP>`; replace the whole placeholder
with the value supplied by the instructor.

## Course path

| Lab | Main idea | Evidence to save |
|---:|---|---|
| [1](labs/01_terminal_and_safety.md) | Terminal, workspace, safety, ROS vocabulary | Environment and vocabulary check |
| [2](labs/02_simulation_graph.md) | Launch simulation and inspect the graph | Node/topic sketch |
| [3](labs/03_topics_teleop.md) | Topics, `Twist`, and keyboard teleop | Annotated `/cmd_vel` message |
| [4](labs/04_sensors_tf_qos.md) | Sensors, TF, RViz, and QoS | Sensor/TF worksheet |
| [5](labs/05_python_nodes.md) | Python publisher, subscriber, parameters, service, launch | Working application package |
| [6](labs/06_control_odometry_diagnostics.md) | `ros2_control`, odometry, diagnostics | Controller health report |
| [7](labs/07_slam.md) | SLAM and map saving | PGM/YAML map pair |
| [8](labs/08_navigation.md) | AMCL, costmaps, goals, and waypoints | Goal and route evidence |
| [9](labs/09_supervised_hardware.md) | Supervised hardware readiness | Signed check and validation report |

Complete the labs in order. A checkpoint is a gate, not an optional review: fix
the current lab before continuing. Each challenge is optional and must preserve
the public interfaces below.

## Stable application interfaces

| Topic | Type | Application use |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Publish requested robot speed |
| `/robot/state` | `std_msgs/msg/String` | Observe the safety state |
| `/odom` | `nav_msgs/msg/Odometry` | Observe estimated pose and velocity |
| `/imu` | `sensor_msgs/msg/Imu` | Observe orientation and inertial motion |
| `/scan` | `sensor_msgs/msg/LaserScan` | Observe LiDAR ranges |
| `/joint_states` | `sensor_msgs/msg/JointState` | Observe wheel position and velocity |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Observe frame relationships |

Never publish directly to a controller-internal topic. The same `/cmd_vel`
interface works in simulation and on the real robot. Application code must not add a
second odometry or TF publisher to “fix” a display problem.

## Working habits

- Label terminals in notes exactly as each lab does.
- Start one launch at a time and stop command publishers before launch files.
- Read the first error, not only the final process summary.
- Keep generated maps and application packages outside the course source package.
- Rebuild only after source changes, then source `install/setup.bash` again.
- Save checkpoint evidence before attempting a challenge.

The Python starter and reference packages for Lab 5 are under
`examples/python/`. Copy a package into the workspace `src` directory; do not
edit generated files under `build`, `install`, or `log`.

## Safety boundary

Simulation is the independent development environment. Setup, checks, and launch
startup do not move the robot. Simulation first requires an explicit `/robot/arm`
call; movement then requires a person to hold the joystick deadman and move a
stick, publish `/cmd_vel`, send a Nav2 goal, or explicitly start a reviewed
waypoint route with `--start`.

Real hardware adds stored energy, pinch points, network delay, and persistent
fault handling. Lab 9 requires all of the following:

- Labs 1–8 completed;
- instructor authorization;
- a stable four-wheel lift for initial checks;
- a safety operator beside a tested physical emergency stop;
- measured wheel radius and supported Titan MCV2 firmware;
- a clear test area and agreed stop conditions.

Never weaken a safety check, clear a fault automatically, or run an unverified
automatic tuning procedure as a classroom shortcut.

## Help

Use [Troubleshooting](TROUBLESHOOTING.md) for build, launch, topic, TF, and
simulator failures. When asking for help, include:

1. the lab and checkpoint;
2. the exact command;
3. the first error message;
4. `ros2 node list` and `ros2 topic list` when ROS is running;
5. the read-only `robot_check` result when monitoring is running.

Instructors should also use [Instructor guide](INSTRUCTOR_GUIDE.md). Hardware
work follows [Supervised hardware](HARDWARE.md) without improvising commands.
