# System Architecture

The project presents one stable ROS API while launch selects simulation, mock,
or physical hardware underneath it. `class_4wd` is the simulation default and
`stack_4wd` is the measured physical default.

## Public launch surface

| Launch | Runtime | Added feature |
|---|---|---|
| `sim.launch.py` | Gazebo Harmonic, maze | RViz, sensors, and deadman joystick teleop |
| `mapping.launch.py` | Gazebo office or PC connected to VMXPi | RViz, SLAM Toolbox, and deadman joystick teleop |
| `navigation.launch.py` | Office simulation or remote VMXPi | RViz, map server, AMCL, and Nav2 |
| `robot.launch.py` | VMXPi/Titan hardware | LiDAR, monitoring, optional camera/Foxglove |
| `bringup.launch.py` | simulation, mock, or hardware | advanced profile and sensor arguments |

The small launches select the correct profile for their runtime: simulation uses
`class_4wd`, while physical bringup and hardware navigation use `stack_4wd`.
`robot.launch.py` exposes `robot_profile` for an intentional hardware override.
`bringup.launch.py` validates an advanced profile and delegates to the private
`_robot_runtime.launch.py` module. Private files beginning with `_` are
implementation details and are not launched directly.

Simulation and mapping start the configured gamepad publisher by default, but
it commands motion only while L1 is held. Navigation and hardware keep it off.

## Runtime flow

```mermaid
flowchart TD
    S[Application or Nav2 publishes /cmd_vel Twist] --> A[Topic adapter]
    A --> C[Profile-selected ros2_control drive controller]
    C --> H{Runtime mode}
    H -->|gz_sim| G[gz_ros2_control + Gazebo Harmonic]
    H -->|mock| M[Safe in-process mock hardware]
    H -->|hardware| V[VMX system + Titan MCV2]
    G --> F[Controller odometry and joint feedback]
    M --> F
    V --> W[Encoder odometry]
    F --> A
    A --> O[/odom and standard sensor aliases]
    W --> WO[/wheel/odom: forward velocity]
    V --> I[/imu: yaw and yaw rate]
    WO --> E[Hardware odometry EKF]
    I --> E
    E --> O
    G --> Z[/imu, /scan, optional RGB-D camera]
    Z --> P[Optional depth-to-PointCloud2 converter]
    P --> PC[/camera/depth/points]
    PC --> PF[TF + height/range/body filter]
    PF --> PO[/camera/depth/points_filtered in base_link]
    M --> Q[/imu fallback and clear mock /scan]
    V --> R[VMX IMU + YDLidar + Orbbec]
    V --> D[Motor state + diagnostics]
```

The adapter converts public `geometry_msgs/msg/Twist` into the stamped command
expected by the selected controller. In simulation and mock mode it republishes
controller odometry directly as `/odom`. Hardware defaults to a VMXPi-local EKF:
the adapter exposes raw encoder odometry as `/wheel/odom`, and the EKF combines
only its calibrated forward velocity with IMU yaw and yaw rate to publish
`/odom`. Encoder-derived yaw is intentionally excluded because skid-steer tire
scrub makes it unreliable. Application nodes never need controller names or
internal topic rules.

## Stable ROS interface

| Interface | Direction from application code | Owner |
|---|---|---|
| `/cmd_vel` (`Twist`) | publish | topic adapter subscription |
| `/odom` (`Odometry`) | subscribe | topic adapter alias |
| `/imu` (`Imu`) | subscribe | simulated/VMX sensor or mock fallback |
| `/scan` (`LaserScan`) | subscribe | Gazebo/YDLidar or clear mock scan |
| `/joint_states` (`JointState`) | subscribe | joint-state broadcaster |
| `/camera/depth/points` (`PointCloud2`) | subscribe | optional simulator converter or camera driver |
| `/camera/depth/points_filtered` (`PointCloud2`) | subscribe | optional ground-referenced obstacle filter |
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
| odometry | drive controller | drive controller | EKF from encoder `vx` + IMU yaw |
| IMU | Gazebo bridge | odometry-based fallback | VMX broadcaster |
| LiDAR | Gazebo bridge | clear fixture scan | YDLidar driver |
| camera | optional RGB-D, raw cloud, and filtered obstacles | absent | optional Orbbec cloud; filter started separately |
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

Measured profiles keep body length/width/height, clearance, wheelbase,
physical wheel track, outer envelope, and chassis-relative sensor poses in the
same YAML. Wheel joints do not derive their spacing from body dimensions. The
outer envelope supplies the Nav2 footprint, while controller multipliers and
effective wheel separation remain empirical drivetrain calibration.

The retained 2WD, mecanum, and omni profiles are explicit advanced variants.
They still use the public `/cmd_vel` and `/odom` adapter.

## Control and timeout

The drive controller owns four wheel velocity command interfaces for
`class_4wd` and `stack_4wd`. A finite command timeout is configured in the controller YAML. If a
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
- In simulation/mock mode, the drive controller owns `odom -> base_footprint`.
- In hardware mode with the default `use_imu_odometry:=true`, the EKF owns
  `odom -> base_footprint`; launch disables the controller's odometry TF.
- Runtime publishes the fixed base transform.
- `robot_state_publisher` owns link/joint transforms from the URDF.
- Sensor drivers may publish their calibrated optical frames.

Each transform has one owner. The hardware launch materializes a private
controller YAML with `enable_odom_tf: false` before starting the EKF, so both
publishers cannot own the same edge. Adding a duplicate static transform can
hide an upstream error and create an unstable TF tree.

## Mapping and navigation

`mapping.launch.py` defaults to core simulation plus SLAM Toolbox and the
deadman-protected joystick nodes. With `mode:=hardware`, it becomes a PC-only
client for a separately running VMXPi: SLAM, RViz, and joystick start locally,
while controllers, sensors, odometry, robot model, and monitoring remain owned
by `robot.launch.py` on the VMXPi.

`navigation.launch.py` defaults to core simulation plus map server,
localization, planners, controller server, depth-point pipeline, and RViz. With
`mode:=hardware`, it becomes a PC-only client for a separately launched VMXPi:
map server, AMCL, Nav2, and RViz run locally while the VMXPi retains hardware,
sensor, fused-odometry, and TF ownership. Hardware mode disables the point cloud
by default and overlays conservative controller, velocity-smoother, recovery,
and AMCL settings. In either mode Nav2 publishes `/cmd_vel` only after a goal;
joystick remains disabled to prevent competing publishers.

When enabled, the filtered `base_link` cloud marks the local Nav2 costmap and
the raw optical cloud only ray-clears it. The global costmap remains
map/LiDAR-based.

The Nav2 RViz configuration separates the grayscale occupancy **Map** from the
colored **Global Costmap** and **Local Costmap** overlays. The optional
`run_waypoint_route.py` client previews the bundled office route by default and
requires `--start` before sending `FollowWaypoints`; it is never included in a
launch file.

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
studica_robot_apps       developer-owned behaviors and project configuration
studica_vmxpi_ros2       model, runtime, control hardware, safety, and labs
studica_robot_monitor    diagnostics, read-only checks, guarded validation
studica_drivers          low-level VMXPi/Titan C++ infrastructure
studica_ros2_control     optional non-drive accessory container
Orbbec / YDLidar         unchanged vendor sensor drivers
```

Drivetrain ownership exists only in `studica_vmxpi_ros2`. The accessory package
contains no Titan, differential/mecanum drive, odometry, teleop, navigation, or
automatic tuning path.

## Extension rule

New applications should preserve the standard topics and add a focused launch
or Python node in `studica_robot_apps`. Changes to controller-specific topics,
TF ownership, hardware safety, or automatic command publishing require a
platform design review. See [Application development](DEVELOPMENT.md).
