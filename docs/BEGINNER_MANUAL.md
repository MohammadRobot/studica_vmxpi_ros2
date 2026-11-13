# Beginner ROS 2 manual

Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic. Local candidate, 2026-11-13.
These commands describe the source package; this change does not provision or
change a robot. The recorded robot uses domain **78**, not the example domain 10.
The instructor must provide the robot's actual domain and PC DDS profile.

## 1. Understand the settings

| Concept | Meaning |
|---|---|
| IP address | Network address of the PC or robot; each interface needs a unique address |
| Hostname | Readable computer name, such as `robot-01`; it does not isolate ROS |
| ROS domain | Group of ROS nodes that can discover and communicate with each other |
| Topic | Named data channel, such as `/scan` or `/cmd_vel` |
| Namespace | Prefix for ROS names; leave it at `/` for these independent sessions |
| Simulation time | Simulation nodes use `/clock`; physical robot nodes use system time |

Suggested **new classroom** allocation: robot-01 domain 10, robot-02 domain 11,
simulation domain 20. Allocate unique domains in 1–100. Existing robot 78 can keep
78. Robot and its PC applications must match. Different domains allow the same
`/scan`, `/odom`, `/tf` and `/cmd_vel` names. Domains are not access control.
Use a trusted classroom LAN and instructor-managed firewall rules.
The launch files select `use_sim_time:=true` internally for simulation and `false`
for hardware; students do not need to set clocks manually.

Connect the PC and robots to one router/AP. Ethernet and Wi-Fi can be mixed.
Use DHCP reservations; do not connect several robots with the same factory fixed
Ethernet address. A robot hotspot is an optional standalone connection, not needed
for the shared classroom LAN. See [instructor networking](NETWORKING.md).

## 2. Build on the PC

Use a fresh terminal with no student or Jazzy overlay sourced. For an existing
workspace with dependencies installed:

```bash
source /opt/ros/humble/setup.bash
export GZ_VERSION=harmonic
cd ~/studica_ws
colcon build --packages-up-to studica_vmxpi_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source ~/studica_ws/install/setup.bash
```

For first installation use [Installation](INSTALL.md). Robot ARM64 builds follow
[Native build](NATIVE_BUILD.md); never copy PC binaries to the Pi. Building locally
does not update the running robot. Deployment is a separate instructor operation.

## 3. Prepare simulation once

This writes a new terminal environment and a loopback-only DDS configuration.
It does not start ROS, change the firewall or edit `.bashrc`.

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
ros2 run studica_vmxpi_ros2 configure_cyclonedds.py sim \
  --name beginner_sim --domain-id 20
```

If a different profile already exists, inspect it rather than overwriting it.
In **every simulation terminal**, including map-saving and inspection terminals:

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.ros/studica_beginner_sim.env
export ROS_DOMAIN_ID=20
export GZ_PARTITION=studica_sim_20
```

The sourced file sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI`
and `GZ_VERSION=harmonic`. Loopback selection confines DDS to this PC; the domain
separates local sessions. `GZ_PARTITION` separates Gazebo transport sessions.
Do not reuse these simulation settings for hardware.

## 4. Simulation with joystick and RViz

Plug the supported DualShock-style joystick into the PC. From a prepared
simulation terminal:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

In a second prepared simulation terminal, explicitly arm **simulation only**:

```bash
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
```

Hold L1 and use the sticks; release L1 to stop. In this course, left stick drives
and right stick turns. Other controllers may require axis/button configuration.
For observation without a joystick:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py use_joystick:=false
```

Run only one of these launches. Stop the previous launch with Ctrl+C first.
Do not use the simulation arm command on physical hardware.

## 5. Prepare a real-robot PC session

Instructor, once per PC/robot connection: generate the peer file using the actual
PC interface/address, robot address and provisioned domain. Example values below
are for a **new classroom**, not the existing installation:

```bash
ip -br address
ip route get 192.168.50.101
ros2 run studica_vmxpi_ros2 configure_cyclonedds.py peer \
  --name robot_01 --interface enp3s0 \
  --local-address 192.168.50.10 --peer-address 192.168.50.101 \
  --domain-id 10
```

The generator rejects an address absent from the selected interface. Use the
actual interface from `ip -br address`; it may be Ethernet or Wi-Fi. The robot
needs a matching peer/profile and boot-service domain, provisioned separately by
the instructor. This PC command alone does not configure the robot. The generator
prints narrow firewall rules for instructor review; it does not apply them.
For a PC with two networks, select only the intended robotics connection.

For the newly provisioned robot-01, **every real-robot PC terminal** uses:

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.ros/studica_robot_01.env
export ROS_DOMAIN_ID=10
ros2 topic list --no-daemon
```

For the **existing domain-78 workstation installation**, instead use its already
provisioned environment (do not mix this with robot-01's environment):

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.config/studica/robot-wifi.sh
export ROS_DOMAIN_ID=78
ros2 topic list --no-daemon
```

If that existing file is absent or stale, ask the instructor to regenerate it
from verified addresses. Do not change domains or DDS files in a running session.

## 6. Real robot: power on and drive

On a commissioned robot, power-on starts hardware, sensors and odometry disarmed.
No SSH mode-switch command is needed. This repository's new boot candidate is
still undeployed; the instructor must confirm which runtime is installed.
Physical operation requires instructor qualification and an approved stop setup;
the current VMX-input-only E-stop does not establish an independent stop path.

In a prepared real-robot PC terminal:

```bash
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=drive
```

This starts RViz and joystick control on the PC. Follow the instructor's local
Reset/Start procedure only after checking fresh data and a clear test area.
L1 is the joystick deadman; left stick drives, right stick turns. Release L1 to
stop. Default limits are 0.20 m/s and 0.60 rad/s; L1+R1 turbo is 0.30 m/s and
0.90 rad/s. For a slower supervised session, stop the previous launch and use:

```bash
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=drive \
  linear_speed:=0.10 angular_speed:=0.30 \
  turbo_linear_speed:=0.10 turbo_angular_speed:=0.30
```

Only one PC session may command a given robot. Never run separate joystick
publishers alongside this launch. Do not increase safety limits for a lesson.

## 7. SLAM and save a map

Stop the previous launch; for hardware, stop the robot physically first.
Choose one command in the corresponding prepared terminal:

```bash
# Simulation: starts Gazebo, SLAM, joystick and RViz.
ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=gz_sim
```

```bash
# Real robot: starts PC applications; uses the already-running robot.
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=slam
```

For simulation, call `/robot/arm` from section 4 after startup. Move slowly and
observe the map in RViz. Stop moving before saving. Keep SLAM running and open
another terminal prepared for the **same target and domain**:

```bash
mkdir -p ~/studica_ws/project_maps
# Real robot:
ros2 run nav2_map_server map_saver_cli \
  -f ~/studica_ws/project_maps/my_room
```

For simulation use a separate filename and simulation time:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/studica_ws/project_maps/sim_room --ros-args -p use_sim_time:=true
```

Success creates `.yaml` and `.pgm` files. Keep both together. Choose a new name to
preserve an earlier map; these are occupancy maps, not saved SLAM pose graphs.

## 8. Navigate using the saved map

Stop SLAM before navigation. Use a map made for the selected environment.

```bash
# Prepared simulation terminal:
ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=gz_sim \
  map:=$HOME/studica_ws/project_maps/sim_room.yaml
```

```bash
# Prepared real-robot terminal:
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=navigation \
  map:=$HOME/studica_ws/project_maps/my_room.yaml
```

In RViz, set **2D Pose Estimate** to the robot's actual position and heading.
Check that the laser scan aligns with the map, then use **Nav2 Goal**. Simulation
requires the explicit simulation arm call; hardware uses its local enable.

Hardware navigation includes joystick takeover: holding L1 cancels the goal and
selects manual control. Releasing L1 stops; send a **new goal** to resume navigation.
Do not start a second joystick node. See the [recorded robot manual](ROBOT_MANUAL.md)
for takeover acceptance limits and status topics.

## 9. Inspect and stop

Use a terminal prepared for the same target:

```bash
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI
ros2 node list
ros2 topic list --no-daemon
ros2 topic echo /robot/state --once --no-daemon
ros2 topic echo /robot/safety_reason --once --qos-durability transient_local --no-daemon
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
# During SLAM/navigation:
ros2 run tf2_ros tf2_echo map odom
```

Stop continuous inspection commands with Ctrl+C. To end operation: cancel goals,
release L1, use the physical Stop for hardware, then Ctrl+C the PC launch.
For simulation only, disarm before closing:

```bash
ros2 service call /robot/disarm std_srvs/srv/Trigger '{}'
```

A physical fault requires investigation; do not keep arming or sending goals.
No topics: check target domain, selected interface/IP, route, peer configuration
and firewall. A stale CLI node list can be refreshed with `ros2 daemon stop` and
`ros2 daemon start` in that prepared terminal. Never disable the whole firewall.

## 10. Two robots and simulation together

Open separate terminals with separate environments: robot-01 uses domain 10,
robot-02 uses its instructor-provided peer file and domain 11, simulation uses its
loopback file and domain 20. Each launches its own RViz/application processes.
The root namespace and standard topic names remain unchanged. Changing only a
hostname or namespace does not select a robot.

Keep one physical joystick assigned to one active control session. Do not launch
two joystick-enabled sessions against the same controller. Observe simulation
with `sim.launch.py use_joystick:=false` while controlling a physical robot.
For another independent simulation, generate a differently named sim profile with
another allocated domain and use a different `GZ_PARTITION` in all its terminals.

See supported arguments without starting a session:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py --show-args
ros2 launch studica_vmxpi_ros2 remote.launch.py --show-args
ros2 launch studica_vmxpi_ros2 mapping.launch.py --show-args
ros2 launch studica_vmxpi_ros2 navigation.launch.py --show-args
```

## 11. Two simulated robots: SLAM, then navigation

Use **four terminals**. Each robot has its own Gazebo world and RViz window.
They use the same topic names but cannot normally discover each other because
the ROS domains differ. Gazebo partitions also differ.

| Session | Launch terminal | Control/map terminal | ROS domain | Gazebo partition |
|---|---|---|---|---|
| Sim A | 1 | 3 | 20 | `studica_sim_a` |
| Sim B | 2 | 4 | 21 | `studica_sim_b` |

Run SLAM for both robots first. Save both maps, stop both SLAM launches, then run
navigation for both robots. **Do not run SLAM and AMCL navigation for the same
robot together.** Close earlier simulation launches before beginning.

### A. Create two simulation profiles once

In any fresh PC terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
ros2 run studica_vmxpi_ros2 configure_cyclonedds.py sim \
  --name sim_a --domain-id 20
ros2 run studica_vmxpi_ros2 configure_cyclonedds.py sim \
  --name sim_b --domain-id 21
mkdir -p ~/studica_ws/project_maps
```

These profiles use loopback only. Do not source a physical robot environment.
If a conflicting profile exists, inspect it instead of adding `--force` blindly.

### B. Prepare all four terminals

Paste this into **terminals 1 and 3** (Sim A):

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.ros/studica_sim_a.env
export GZ_PARTITION=studica_sim_a
printenv ROS_DOMAIN_ID GZ_PARTITION
```

Expect `20` and `studica_sim_a`.

Paste this into **terminals 2 and 4** (Sim B):

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.ros/studica_sim_b.env
export GZ_PARTITION=studica_sim_b
printenv ROS_DOMAIN_ID GZ_PARTITION
```

Expect `21` and `studica_sim_b`. Keep these terminals open for the whole lesson.
Never change their environment while a launch is running.

### C. Start both SLAM sessions

Run the following command in **terminal 1**, then in **terminal 2**:

```bash
ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=gz_sim \
  gz_headless:=true use_joystick:=false
```

Both worlds run without Gazebo GUI windows, and each session opens RViz. Identify
which RViz belongs to each terminal. No physical joystick is opened by either
session. Add `gui:=false` if you do not need RViz on a lower-powered PC.

In **terminal 3**, check Sim A, arm it and make a short forward pass:

```bash
ros2 topic echo /robot/state --once --no-daemon
# Wait for READY_DISARMED, then:
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
timeout --signal=INT 5s ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.10}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 service call /robot/disarm std_srvs/srv/Trigger '{}'
```

In **terminal 4**, check Sim B, arm it and make a short backward pass:

```bash
ros2 topic echo /robot/state --once --no-daemon
# Wait for READY_DISARMED, then:
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
timeout --signal=INT 5s ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: -0.10}, angular: {z: 0.0}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
ros2 service call /robot/disarm std_srvs/srv/Trigger '{}'
```

`timeout` deliberately stops publication after five seconds; exit status 124 is
normal. Wait for each arm request to succeed. These small passes build local maps
for a short navigation exercise, not complete maps of the whole office. These
commands are **simulation only**. Do not add a second command publisher.

### D. Save both maps while SLAM is still running

Wait until each RViz map is visible. In **terminal 3**:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/studica_ws/project_maps/sim_a_office --ros-args \
  -p use_sim_time:=true -p save_map_timeout:=10.0
```

In **terminal 4**:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/studica_ws/project_maps/sim_b_office --ros-args \
  -p use_sim_time:=true -p save_map_timeout:=10.0
```

Both must report success. Keep each `.yaml` with its `.pgm`; use new names on a
repeat lesson if you want to preserve previous maps. Check the files:

```bash
ls -l ~/studica_ws/project_maps/sim_a_office.{yaml,pgm}
ls -l ~/studica_ws/project_maps/sim_b_office.{yaml,pgm}
```

Now press **Ctrl+C in terminals 1 and 2**. Wait for both SLAM/Gazebo launches to
exit before starting navigation. Leave terminals 3 and 4 in their original domains.

### E. Start both navigation sessions

In **terminal 1**:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=gz_sim \
  gz_headless:=true use_joystick:=false use_point_cloud:=false \
  map:=$HOME/studica_ws/project_maps/sim_a_office.yaml
```

In **terminal 2**:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=gz_sim \
  gz_headless:=true use_joystick:=false use_point_cloud:=false \
  map:=$HOME/studica_ws/project_maps/sim_b_office.yaml
```

Each new world respawns its robot at `(0, 0)` facing positive X. In each RViz,
use **2D Pose Estimate** at that position and heading. Wait for the scan to align
with the map. Do not set the initial pose to where the previous SLAM run ended.

In **terminal 3**, check localization and then arm Sim A:

```bash
ros2 run tf2_ros tf2_echo map base_link
# After a valid transform appears, Ctrl+C this inspection command.
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
ros2 action list -t
```

Run the same three commands in **terminal 4** for Sim B. Expect the
`/navigate_to_pose` action in both domains. Do not run the manual `/cmd_vel`
publishers from the mapping step while Nav2 controls a robot.

### F. Send a different goal to each robot

In **terminal 3**, send Sim A a short forward goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 0.45, y: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

While it runs, in **terminal 4**, send Sim B a short backward-position goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -0.45, y: 0.0}, orientation: {w: 1.0}}}}' \
  --feedback
```

The planner may turn the robot to reach a goal behind it. Both should report
`SUCCEEDED` within the configured goal tolerance. For other routes, use **Nav2
Goal** in the correct RViz and choose a known free area of that robot's map.

### G. Stop both sessions

After the goals finish, run this in **terminal 3**, then **terminal 4**:

```bash
ros2 service call /robot/disarm std_srvs/srv/Trigger '{}'
```

Then press **Ctrl+C in terminals 1 and 2** to close navigation and Gazebo.
If a goal is still running and you need to end the lesson, stop both launches;
Ctrl+C on an action-client terminal alone is not a reliable goal cancellation.
Do not restart or re-arm a stopped session with an old goal still active.

This two-domain workflow passed the local [SLAM/navigation isolation test](TWO_SLAM_NAVIGATION_ACCEPTANCE.md):
different maps, isolated TF and distinct successful goal UUIDs. That test used
headless RViz-disabled sessions; this walkthrough additionally opens RViz for
learning. It does not establish physical robot networking or hardware acceptance.
