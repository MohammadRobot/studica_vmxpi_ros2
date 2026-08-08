# Documentation

Start with simulation and follow the course in order. Hardware, PID tuning,
networking, and custom profiles are intentionally separated from the beginner
path.

## Students

| Guide | Use it when |
|---|---|
| [Installation](INSTALL.md) | Preparing an Ubuntu 22.04 computer |
| [Course and labs](COURSE.md) | Learning ROS 2 from Lab 1 through Lab 9 |
| [Troubleshooting](TROUBLESHOOTING.md) | A build, launch, topic, TF, or simulator check fails |
| [Mapping and navigation](MAPPING_NAVIGATION.md) | Starting the SLAM and Nav2 lessons |
| [Supervised hardware](HARDWARE.md) | An instructor has approved real-robot work |

## Instructors and advanced projects

| Guide | Scope |
|---|---|
| [System architecture](ARCHITECTURE.md) | Runtime layers and stable public interfaces |
| [Robot health and velocity PID](ROBOT_HEALTH_AND_PID.md) | Titan MCV2 safety and validation details |
| [Networking](NETWORKING.md) | DDS interfaces and read-only Foxglove access |
| [Advanced profiles](PROFILE_AUTHORING.md) | 2WD, mecanum, omni, and custom robot configuration |
| [Instructor guide](INSTRUCTOR_GUIDE.md) | Classroom preparation and physical acceptance tests |

## Course files

- `docs/labs/`: nine guided labs with checkpoints and cleanup steps
- `examples/python/starters/`: files containing student TODOs
- `examples/python/solutions/`: separate reference implementations
- `bringup/launch/`: four beginner launches and the advanced bringup interface
- `bringup/config/profiles/`: the core `class_4wd` profile and advanced variants

## Command conventions

Commands use this workspace variable instead of a fixed username or home path:

```bash
export STUDICA_WS="$HOME/ros2_ws"
```

Every new terminal needs:

```bash
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
```

Labels such as **Terminal 1** mean open a separate terminal and source both
setup files there. Text such as `<ROBOT_IP>` is a placeholder; replace it with
the value provided by the instructor, without typing the angle brackets.

Stop a foreground ROS process with `Ctrl+C`. Stop teleoperation first, then the
main launch. Never close a terminal that is still commanding a real robot.

## Stable beginner API

Student code uses only standard ROS interfaces:

| Direction | Topic | Type |
|---|---|---|
| publish | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| subscribe | `/odom` | `nav_msgs/msg/Odometry` |
| subscribe | `/imu` | `sensor_msgs/msg/Imu` |
| subscribe | `/scan` | `sensor_msgs/msg/LaserScan` |
| subscribe | `/joint_states` | `sensor_msgs/msg/JointState` |
| subscribe | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` |

The same names are used in simulation, mock mode, and hardware. Controller
internals are advanced implementation details, not student command APIs.

## Safety promise

Documentation checks, setup, CI, health checks, mapping, and navigation launch
files do not publish motion commands. Hardware motion always requires a separate
human action after the robot is verified and the area is safe.
