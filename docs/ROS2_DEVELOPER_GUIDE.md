# ROS 2 developer guide

Local candidate, 2026-11-10. Ubuntu 22.04 / ROS 2 Humble. Nothing in this
revision has been deployed or physically qualified. See the
[audit and remaining gates](ROS2_AUDIT_2026-11-10.md).

## Responsibilities and interfaces

The Pi owns drivers, local enable, command supervision, encoder/IMU odometry,
robot TF and acquisition. The PC owns student applications, joystick, RViz,
SLAM, Nav2 and RoboHiTec observation. Do not deploy PC applications to the Pi.
Platform source belongs in `~/studica_ws`; student packages belong in
`~/ros2_ws`; RoboHiTec belongs in `~/projects/robohitec-os`.

| Interface | Type | Contract |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Application input; m/s and rad/s, base coordinates; one publisher, reliable/volatile, small queue. Send continuously while authorized. |
| `/robot_base_controller/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Private supervisor output. Never publish here from an application. |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_link`, SI units; use sensor-data QoS for observation. Hardware EKF owns fused odometry. |
| `/imu` | `sensor_msgs/msg/Imu` | Sensor frame from URDF; rad/s and m/s²; best-effort/volatile. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Laser frame from URDF; metres; sensor-data subscription works with reliable or best-effort offers. |
| `/joint_states` | `sensor_msgs/msg/JointState` | URDF joint names, radians and rad/s. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Standard TF dynamic/static QoS; one owner per child frame. |
| `/robot/state`, `/robot/safety_reason` | `std_msgs/msg/String` | Reliable, transient-local depth 1; cached values are not readiness evidence. |
| `/robot_status/motors` | `studica_robot_monitor/msg/MotorTelemetryArray` | Per-channel targets, feedback age, firmware and fault flags; telemetry is not independent stopping evidence. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Health observations; subscribers must check freshness. |

Use REP-103 SI/frame conventions and REP-105 `map` → `odom` → `base_link`.
SLAM or AMCL on the PC owns `map` → `odom`, never both simultaneously.
The hardware control rate stays **25 Hz**. The supervisor deadline remains
**250 ms**, drive timeout **500 ms**, and provenance acceptance **80 ms**.
These are distinct gates; the status observer's receive-age budgets do not
replace them. Network/PC loss must not be relied on as a hardware stop test.

Hardware uses wall time and synchronized PC/Pi clocks. Simulation uses its own
`/clock` and `use_sim_time:=true`. Assign a separate ROS domain to each physical
robot and each simulation session. Configure discovery per terminal; domain IDs
are routing isolation, not authentication. Do not globally source `.bashrc`.
Use loopback/network namespaces for automated fixtures, not domain selection alone.

## PC build and validation

Fresh terminal, outside Conda and student overlays:

```bash
source /opt/ros/humble/setup.bash
cd ~/studica_ws
export GZ_VERSION=harmonic CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --packages-up-to studica_vmxpi_ros2 --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/local_setup.bash
```

Keep the existing workspace's install layout; do not switch it to symlink or
merge install incidentally. On x86 without VMX SDK this builds the PC interfaces
and driver stubs, **not** the ARM64 hardware plugin. A production build rejects
stubs. Dependency pins are in `dependencies/sources.repos` (PC) and
`dependencies/hardware.repos` (hardware). After reviewed cross-repository changes
are committed, update the pins before constructing a release.

Run non-actuating tests with no network route or device passthrough:

```bash
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  unshare --user --map-root-user --net bash --noprofile --norc -c '
    ip link set lo up
    source /opt/ros/humble/setup.bash
    source "$HOME/studica_ws/install/local_setup.bash"
    export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    cd "$HOME/studica_ws"
    colcon test --packages-select studica_drivers studica_robot_monitor studica_vmxpi_ros2 \
      --executor sequential --ctest-args -E "cppcheck|headless" --output-on-failure
    colcon test-result --verbose
  '
```

This scoped gate excludes cppcheck and headless Gazebo; it is not the full suite.
If user namespaces are unavailable, use a separate isolated fixture host.
The [native build guide](NATIVE_BUILD.md) covers ARM64 compilation; PC binaries
must never be copied to the Pi.

## Standard launch commands

After the appropriate **per-terminal** network setup, use one PC launch:

```bash
# Existing robot runtime, already running at power-on:
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=drive
# Alternatives, after stopping the previous launch and physical motion:
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=slam
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=navigation \
  map:=$HOME/studica_ws/project_maps/my_room.yaml

# Separate loopback-only simulation session:
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Use [Robot Manual](ROBOT_MANUAL.md) for joystick controls, map saving and existing
robot installation details. The new minimal service is a replacement candidate,
not another runtime to run beside the installed classroom service.

## Small Python and C++ applications

These installed examples subscribe to odometry by default and exit after a few
seconds. They do not arm a robot:

```bash
ros2 run studica_vmxpi_ros2 developer_example.py
ros2 run studica_vmxpi_ros2 developer_example_cpp
```

Only in an isolated, explicitly armed simulation with the joystick disabled and
no other command owner, request a bounded 0.05 m/s pulse:

```bash
ros2 run studica_vmxpi_ros2 developer_example.py --publish-motion --duration 1
ros2 run studica_vmxpi_ros2 developer_example_cpp --ros-args \
  -p publish_motion:=true -p duration:=1.0
```

Run these alternatives separately. The examples use monotonic durations, reject
conflicting publishers and finish with zero commands. They are educational code,
not a replacement for the supervisor or an independent emergency stop. Sensor
message timestamps still follow the selected ROS clock.

## RoboHiTec OS

The vendor adapter under `~/projects/robohitec-os/adapters/studica_vmxpi` converts
allowlisted telemetry into standard diagnostics. The core worker consumes those
observations. Mobility remains **unavailable**, qualification **unqualified**;
no enable, velocity, goal or vendor service route is added. Follow that adapter's
README for its explicit robot identity/domain and fixture tests.
