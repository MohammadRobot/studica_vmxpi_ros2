# Documentation

Start with simulation and follow the course in order. Hardware, PID tuning,
networking, and custom profiles are intentionally separated from the beginner
path.

## Learning and development

| Guide | Use it when |
|---|---|
| [Quick start](QUICK_START.md) | Running simulation, physical mapping, or two-computer physical navigation with minimal setup |
| [Installation](INSTALL.md) | Preparing an Ubuntu 22.04 computer |
| [Launch arguments](LAUNCH_ARGUMENTS.md) | Defaults and examples for every supported public launch |
| [Camera and point cloud](CAMERA_POINT_CLOUD.md) | Testing RGB, depth, PointCloud2, and the read-only observer |
| [Application development](DEVELOPMENT.md) | Editing on a PC and deploying source to VMXPi |
| [Course and labs](COURSE.md) | Learning ROS 2 from Lab 1 through Lab 9 |
| [Joystick teleoperation](JOYSTICK.md) | Driving simulation with a DualShock 4 or compatible controller |
| [Troubleshooting](TROUBLESHOOTING.md) | A build, launch, topic, TF, or simulator check fails |
| [Mapping and navigation](MAPPING_NAVIGATION.md) | Starting the SLAM and Nav2 lessons |
| [Supervised hardware](HARDWARE.md) | An instructor has approved real-robot work |

## Instructors and advanced projects

| Guide | Scope |
|---|---|
| [System architecture](ARCHITECTURE.md) | Runtime layers and stable public interfaces |
| [Safety supervisor API](SAFETY_SUPERVISOR.md) | State, arm/disarm, command validation, and Phase 1 limits |
| [Robot health and velocity PID](ROBOT_HEALTH_AND_PID.md) | Titan MCV2 safety and validation details |
| [Networking](NETWORKING.md) | Generated Cyclone DDS profiles for simulation, Wi-Fi, Ethernet, and Foxglove |
| [Advanced profiles](PROFILE_AUTHORING.md) | 2WD, mecanum, omni, and custom robot configuration |
| [Instructor guide](INSTRUCTOR_GUIDE.md) | Classroom preparation and physical acceptance tests |

## Production engineering

| Guide | Scope |
|---|---|
| [Production architecture](PRODUCTION_ARCHITECTURE.md) | Target safety state model, command ownership, services, networking, and updates |
| [VMXPi production image](VMXPI_PRODUCTION_IMAGE.md) | Measured load, minimal OS/service profile, package split, and staged hardening |
| [Physical hardware safety gate](HARDWARE_SAFETY_GATE.md) | Phase 2 local enable, E-stop status, wiring gate, and acceptance fixture |
| [Release process](RELEASE_PROCESS.md) | Immutable inputs, artifacts, rollout channels, and release gates |

## Course files

- `docs/labs/`: nine guided labs with checkpoints and cleanup steps
- `examples/python/starters/`: files containing guided TODOs
- `examples/python/solutions/`: separate reference implementations
- `bringup/launch/`: four beginner launches and the advanced bringup interface
- `bringup/config/profiles/`: `class_4wd` simulation, `stack_4wd` hardware, and variants

## Command conventions

The project uses a dedicated workspace to avoid stale overlay conflicts. Commands
use this variable instead of a fixed username or absolute home path:

```bash
export STUDICA_WS="$HOME/studica_ws"
```

Use [Quick start](QUICK_START.md) for the copy/paste launch sequence. The
remaining pages explain configuration choices, alternatives, and recovery.

Every new simulation terminal needs:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
```

Labels such as **Terminal 1** mean open a separate terminal and source all three
files there. Hardware terminals use their generated Wi-Fi or Ethernet peer
environment instead of `studica_sim.env`. Text such as `<ROBOT_IP>` is a
placeholder; replace it without typing the angle brackets.

Stop a foreground ROS process with `Ctrl+C`. Stop teleoperation first, then the
main launch. Never close a terminal that is still commanding a real robot.

## Stable application API

Application code uses only standard ROS interfaces:

| Direction | Topic | Type |
|---|---|---|
| publish | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| subscribe | `/robot/state` | `std_msgs/msg/String` |
| subscribe | `/robot/safety_reason` | `std_msgs/msg/String` |
| subscribe | `/odom` | `nav_msgs/msg/Odometry` |
| subscribe | `/imu` | `sensor_msgs/msg/Imu` |
| subscribe | `/scan` | `sensor_msgs/msg/LaserScan` |
| subscribe | `/joint_states` | `sensor_msgs/msg/JointState` |
| subscribe | `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` |
| subscribe | `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` |

The same names are used in simulation, mock mode, and hardware. Controller
internals are advanced implementation details, not application command APIs.
Simulation must be explicitly armed through `/robot/arm`; `/robot/disarm` is
always a safe stop request. Hardware software-arming is intentionally disabled
in the current phase.

## Safety promise

Documentation checks, setup, CI, and health checks never initiate motion.
Simulation requires explicit arm plus a command source; mapping also requires
L1 plus stick input, and navigation requires a goal. Hardware software-arming is
disabled until the local production safety gate is implemented.
