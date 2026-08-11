# Quick Start

This page is the shortest supported path from an installed workspace to a
running simulation or physical mapping session. Follow the linked detailed
guides when installing a new computer, changing networking, or diagnosing a
failed check.

## Command convention

The project uses one dedicated workspace:

```bash
export STUDICA_WS="$HOME/studica_ws"
```

Every terminal must source exactly one project environment, ROS 2 Humble, and
the current workspace overlay. Do not source an older ROS workspace in the same
terminal.

## First installation

On an Ubuntu 22.04 development PC:

```bash
export STUDICA_WS="$HOME/studica_ws"
mkdir -p "$STUDICA_WS/src"

git clone https://github.com/MohammadRobot/studica_vmxpi_ros2.git \
  "$STUDICA_WS/src/studica_vmxpi_ros2"

cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode simulation
```

The installer imports pinned dependencies, installs declared packages, builds
with symlink install, and runs non-motion checks. For an existing installation,
skip this section. See [Installation](INSTALL.md) for hardware mode, recovery,
and manual-build details.

## Run simulation

Connect the joystick before launching if it will be used.

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Expected: Gazebo and RViz open, the controllers become active, and `/scan`,
`/odom`, `/imu`, and `/joint_states` publish. Hold L1 while moving the sticks;
releasing L1 stops joystick commands.

In another terminal:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

ros2 control list_controllers
ros2 run studica_robot_monitor robot_check --mode simulation
```

Use `use_camera:=true` for image topics or `use_point_cloud:=true` for the raw
and floor-filtered point-cloud pipeline:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py use_point_cloud:=true
```

## Fast edit, build, and test loop

Edit source below `$STUDICA_WS/src`, never generated files below `build`,
`install`, or `log`. Build only the package being changed:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
cd "$STUDICA_WS"

colcon build --symlink-install \
  --packages-select studica_vmxpi_ros2

source "$STUDICA_WS/install/setup.bash"

cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/check_project.sh
```

For application code, select `studica_robot_apps` instead. See
[Application development](DEVELOPMENT.md) for the PC-to-VMXPi deployment loop.

## Physical robot prerequisites

Before hardware motion:

- clear the work area and keep the physical emergency stop reachable;
- confirm the VMXPi and PC use matching Cyclone DDS domain and peer profiles;
- stop previous simulation, mapping, navigation, and teleoperation processes;
- keep only one external `/cmd_vel` publisher.

Generate the Wi-Fi or Ethernet profiles once by following
[Networking and Cyclone DDS](NETWORKING.md). The commands below assume the
generated Wi-Fi files are named `studica_vmxpi_wifi.env` on the VMXPi and
`studica_pc_wifi.env` on the PC.

## Start the physical robot

On the VMXPi:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_vmxpi_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

sudo --preserve-env=ROS_DOMAIN_ID,RMW_IMPLEMENTATION,ROS_LOCALHOST_ONLY,CYCLONEDDS_URI \
  env STUDICA_WS="$STUDICA_WS" bash -lc '
    cd "$STUDICA_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec ros2 launch studica_vmxpi_ros2 robot.launch.py \
      use_lidar:=true use_camera:=false use_point_cloud:=false \
      use_camera_color:=false use_foxglove:=false use_joystick:=false
  '
```

The validated hardware control default is 25 Hz. Use
`hardware_control_rate_hz:=50` only for a deliberate comparison while watching
CPU temperature and throttling.

In a second VMXPi terminal, verify the stationary robot:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_vmxpi_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

ros2 run studica_robot_monitor robot_check \
  --mode hardware --timeout 5

ros2 param get /controller_manager update_rate
sudo vcgencmd get_throttled
```

Do not drive after a required `FAIL` or active throttling indication.

## Map the physical environment

Keep the VMXPi launch running. On the PC, connect the joystick and start the
single PC-side mapping launch:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=hardware
```

This starts only SLAM Toolbox, RViz, and deadman joystick teleoperation on the
PC. The physical profile uses calibrated wheel/IMU odometry for scan placement,
keeps continuous X2 scan matching disabled, and permits only conservative loop
closure.

Verify the installed policy in another PC terminal:

```bash
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"

ros2 param get /slam_toolbox use_scan_matching
ros2 param get /slam_toolbox do_loop_closing
ros2 param get /slam_toolbox throttle_scans
```

Expected:

```text
Boolean value is: False
Boolean value is: True
Integer value is: 1
```

Use normal speed rather than turbo. Drive with gentle stick input, pause before
and after turns, and approach the starting area slowly from approximately the
original direction. Remain stopped there for 2--3 seconds. One small map
adjustment can indicate successful loop closure; progressive wall duplication
is a fault.

Restart the PC mapping launch before retrying a distorted map. A corrupted
in-memory map is not a useful starting point.

Save an accepted map from another PC terminal:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

mkdir -p "$STUDICA_WS/project_maps"

ros2 run nav2_map_server map_saver_cli \
  -f "$STUDICA_WS/project_maps/real_robot_map"
```

Keep `real_robot_map.pgm` and `real_robot_map.yaml` together.

## Navigate the physical robot

Stop the PC mapping launch first. Keep the VMXPi `robot.launch.py` process
running with LiDAR enabled, or start it with the command in **Start the physical
robot** above. Do not run SLAM Toolbox, joystick teleop, or keyboard teleop
during navigation.

Confirm the saved map pair exists on the PC:

```bash
export STUDICA_WS="$HOME/studica_ws"
test -s "$STUDICA_WS/project_maps/real_robot_map.yaml"
test -s "$STUDICA_WS/project_maps/real_robot_map.pgm"
```

Then start the single PC-side navigation launch:

```bash
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=hardware
```

This starts the saved-map server, AMCL, Nav2, and RViz on the PC. Hardware mode
uses wall-clock time, the measured 0.350 x 0.385 m footprint, LiDAR-only
costmaps, and limits of 0.20 m/s and 0.35 rad/s. It does not start
Gazebo or joystick teleoperation.

The confirmed default can be reduced or increased within the supported bounds
without editing a parameter file:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  mode:=hardware \
  hardware_max_linear_speed:=0.20 \
  hardware_max_angular_speed:=0.35
```

Accepted limits are greater than zero through 0.30 m/s and 0.60 rad/s. Change
one value at a time and repeat a nearby-goal test after increasing it.

In RViz, select **2D Pose Estimate**, place the robot accurately on the map,
and drag the arrow in its physical heading. Wait until the red scan overlaps
the mapped walls. Send only a nearby, clear **Nav2 Goal** for the first test and
keep the emergency stop reachable. Cancel the goal immediately if localization
jumps, the scan separates from the walls, or the planned path is unsafe.

In another PC terminal, verify localization and command ownership before the
first goal:

```bash
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"

ros2 lifecycle nodes
ros2 action list | grep -E 'navigate|follow'
timeout 5 ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cmd_vel --verbose
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_vel_theta
```

## Mapping and navigation in simulation

Stop the previous launch before switching modes. Simulation mapping is one
command:

```bash
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$HOME/studica_ws/install/setup.bash"

ros2 launch studica_vmxpi_ros2 mapping.launch.py
```

Simulation navigation uses the bundled office map:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py
```

Set **2D Pose Estimate** in RViz before sending a navigation goal. The public
`navigation.launch.py` defaults to the office simulation; `mode:=hardware`
selects the PC-side physical navigation client.

## Stop cleanly

1. Release L1 and center the joystick.
2. Confirm the robot is stationary.
3. Stop the PC motion/mapping launch with `Ctrl+C`.
4. Stop the VMXPi robot launch with `Ctrl+C`.
5. Wait for controller shutdown before closing either terminal.

For the complete interfaces and alternatives, continue with
[Launch arguments](LAUNCH_ARGUMENTS.md),
[Mapping and navigation](MAPPING_NAVIGATION.md), and
[Troubleshooting](TROUBLESHOOTING.md).
