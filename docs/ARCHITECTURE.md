# System Architecture

The classroom presents one stable ROS API while launch selects simulation, mock,
or physical hardware underneath it. `class_4wd` is the teaching default.

## Public launch surface

| Launch | Runtime | Added feature |
|---|---|---|
| `sim.launch.py` | Gazebo Harmonic, maze | RViz; low-rate LiDAR/IMU; camera optional |
| `mapping.launch.py` | Gazebo Harmonic, office | RViz and SLAM Toolbox |
| `navigation.launch.py` | Gazebo Harmonic, office | RViz, map server, AMCL, and Nav2 |
| `robot.launch.py` | VMXPi/Titan hardware | LiDAR, monitoring, optional camera/Foxglove |
| `bringup.launch.py` | simulation, mock, or hardware | advanced profile and sensor arguments |

The first four launches fix `robot_profile:=class_4wd` and expose only the few
choices needed for their lesson. `bringup.launch.py` validates an advanced
profile and delegates to the private `_robot_runtime.launch.py` module. Private
files beginning with `_` are implementation details and are not launched
directly.

No launch starts keyboard, gamepad, patrol, or another motion publisher.

## Runtime flow

```mermaid
flowchart TD
    S[Student or Nav2 publishes /cmd_vel Twist] --> A[Topic adapter]
    A --> C[Profile-selected ros2_control drive controller]
    C --> H{Runtime mode}
    H -->|gz_sim| G[gz_ros2_control + Gazebo Harmonic]
    H -->|mock| M[Safe in-process mock hardware]
    H -->|hardware| V[VMX system + Titan MCV2]
    G --> F[Controller odometry and joint feedback]
    M --> F
    V --> F
    F --> A
    A --> O[/odom and standard sensor aliases]
    G --> Z[/imu, /scan, optional camera]
    M --> Q[/imu fallback and clear mock /scan]
    V --> R[VMX IMU + YDLidar + Orbbec]
    V --> D[Motor state + diagnostics]
```

The adapter converts public `geometry_msgs/msg/Twist` into the stamped command
expected by the selected controller, and republishes controller odometry as
`/odom`. Student programs never need controller names or internal topic rules.

## Stable ROS interface

| Interface | Direction from student code | Owner |
|---|---|---|
| `/cmd_vel` (`Twist`) | publish | topic adapter subscription |
| `/odom` (`Odometry`) | subscribe | topic adapter alias |
| `/imu` (`Imu`) | subscribe | simulated/VMX sensor or mock fallback |
| `/scan` (`LaserScan`) | subscribe | Gazebo/YDLidar or clear mock scan |
| `/joint_states` (`JointState`) | subscribe | joint-state broadcaster |
| `/tf`, `/tf_static` | subscribe | controllers and state/static publishers |
| `/diagnostics` | subscribe | robot monitor and diagnostic publishers |
| `/robot_status/motors` | subscribe | hardware monitor |

The mock mode provides the same names for automated tests, even when no Gazebo
or hardware SDK is present. Its `/scan` represents a clear environment; it is a
contract fixture, not a physical sensor model.

## Mode ownership

| Concern | Gazebo simulation | Mock | Hardware |
|---|---|---|---|
| control manager | Gazebo plugin | local `ros2_control_node` | local `ros2_control_node` |
| hardware interface | `gz_ros2_control` | `RobotSystemHardware` | `VMXSystemHardware` |
| wheel feedback | simulator | command-following stub | Titan encoders/RPM |
| odometry | drive controller | drive controller | drive controller |
| IMU | Gazebo bridge | odometry-based fallback | VMX broadcaster |
| LiDAR | Gazebo bridge | clear fixture scan | YDLidar driver |
| camera | optional Gazebo sensors | absent | optional Orbbec driver |
| monitoring | optional/usually off | optional/off | on by default |
| Foxglove | off | off | read-only; loopback by default |

Only hardware mode loads the VMXPi/Titan plugin. Building or launching mock mode
on a PC cannot initialize pigpio or send CAN commands.

## Robot model and profiles

The model is assembled from:

- `description/urdf/robot.urdf.xacro`: top-level description;
- `description/robot/urdf/`: links, joints, materials, and sensors;
- `description/ros2_control/robot.ros2_control.xacro`: hardware/controller interfaces;
- `description/gz/robot.gz.xacro`: Harmonic plugins and simulated sensors;
- `bringup/config/profiles/<PROFILE>/`: geometry, mapping, safety, and controller YAML.

Launch validates the profile before invoking Xacro. `drive.wheel_radius_m` is
injected into geometry, encoder conversion, and controller parameters so no
second radius can drift.

The retained 2WD, mecanum, and omni profiles are explicit advanced variants.
They still use the public `/cmd_vel` and `/odom` adapter.

## Control and timeout

The drive controller owns four wheel velocity command interfaces for
`class_4wd`. A finite command timeout is configured in the controller YAML. If a
publisher stops, the controller writes zero; mock and hardware state then return
to zero.

Hardware adds stricter behavior inside `VMXSystemHardware`: MCV2 firmware/PID
probing, RPM conversion, command clamping, feedback freshness, controller
temperature, safe zeroing, and a reactivation-only fault latch. See
[Robot health and velocity PID](ROBOT_HEALTH_AND_PID.md).

## TF ownership

The expected mobile-base chain is:

```text
map -> odom -> base_footprint -> base_link -> wheels and sensors
```

- SLAM or localization owns `map -> odom`.
- The drive controller owns `odom -> base_footprint`.
- Runtime publishes the fixed base transform.
- `robot_state_publisher` owns link/joint transforms from the URDF.
- Sensor drivers may publish their calibrated optical frames.

Each transform has one owner. Adding a duplicate static transform can hide an
upstream error and create an unstable TF tree.

## Mapping and navigation

`mapping.launch.py` composes core simulation with SLAM Toolbox. External teleop
is the only motion source while mapping.

`navigation.launch.py` composes core simulation with the map server,
localization, planners, controller server, and RViz. Nav2 publishes `/cmd_vel`
only after a user sends a goal. It does not include a teleop publisher.

## Health and observation

On hardware, `studica_robot_monitor` observes sensor rates/content, TF,
controllers, wheel tracking, Titan state, and compute resources. It publishes
standard diagnostics and the structured motor topic. `diagnostic_aggregator`
groups health into Motors, Sensors, Control, and Compute.

Foxglove Bridge is configured with telemetry topic allowlists, empty service and
parameter allowlists, an empty client-publish allowlist, and only the connection
graph capability. Browser controls are not part of the motor command path.

## Repository boundaries

```text
studica_vmxpi_ros2       student app, model, runtime, control hardware, labs
studica_robot_monitor    diagnostics, read-only checks, guarded validation
studica_drivers          low-level VMXPi/Titan C++ infrastructure
studica_ros2_control     optional non-drive accessory container
Orbbec / YDLidar         unchanged vendor sensor drivers
```

Drivetrain ownership exists only in `studica_vmxpi_ros2`. The accessory package
contains no Titan, differential/mecanum drive, odometry, teleop, navigation, or
automatic tuning path.

## Extension rule

New classroom features should preserve the standard topics and add a focused
launch or Python node. Changes that require students to know controller-specific
topics, duplicate TF, or introduce an automatic command publisher belong in an
advanced design review rather than the beginner surface.
