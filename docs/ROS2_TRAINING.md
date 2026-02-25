# ROS 2 Training Material (Using `vmxpi_ros2`)

This document is a practical ROS 2 training guide using this project as the reference robot stack.

Target audience:
- Developers new to ROS 2
- Developers moving from ROS 1
- Robotics developers who want one repo for simulation and Titan hardware

Platform:
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim (Harmonic / `gz sim`)

---

## 1. Learning Goals

By the end of this training, you should be able to:
- Explain ROS 2 graph concepts: nodes, topics, services, actions, parameters, TF.
- Build and run the `vmxpi_ros2` package.
- Operate the robot in Gazebo Sim using keyboard and joystick.
- Understand `ros2_control` and controller lifecycle in this project.
- Build a map with SLAM Toolbox and run Nav2 localization/navigation.
- Debug common runtime problems with ROS 2 CLI tools.

---

## 2. Project Architecture

High-level data flow in this repo:

1. Robot model:
- `description/urdf/diffbot.urdf.xacro`
- `description/ros2_control/diffbot.ros2_control.xacro`
- `description/gz/diffbot.gz.xacro`

2. Simulation runtime:
- Gazebo Sim publishes clock and sensor topics.
- `ros_gz_bridge` converts Gazebo messages to ROS 2 messages.
- `robot_state_publisher` publishes TF from URDF.
- `gz_ros2_control` + `controller_manager` run drive controllers.

3. Control:
- Teleop sends velocity commands to `/diffbot_base_controller/cmd_vel`.
- `diff_drive_controller` generates odom and wheel state.

4. Navigation stack:
- SLAM Toolbox for mapping (`nav2_mapping_gz_sim.launch.py`).
- Nav2 + AMCL for localization/navigation (`nav2_navigation_gz_sim.launch.py`).

---

## 3. ROS 2 Core Concepts (Using This Repo)

### 3.1 Nodes
Node = a process with ROS interfaces.

Examples in this project:
- `robot_state_publisher`
- `controller_manager`
- `joint_state_broadcaster`
- `diffbot_base_controller`
- `scan_frame_relay_node`
- `ros_gz_bridge` (`parameter_bridge`)
- `joy_node`, `gamepad_teleop`

CLI:

```bash
ros2 node list
ros2 node info /robot_state_publisher
```

### 3.2 Topics
Topics = streaming data channels.

Important topics:
- `/scan`, `/scan_raw` (`sensor_msgs/LaserScan`)
- `/imu` (`sensor_msgs/Imu`)
- `/tf`, `/tf_static`
- `/diffbot_base_controller/cmd_vel`
- `/diffbot_base_controller/odom`
- `/clock`

CLI:

```bash
ros2 topic list
ros2 topic info /imu -v
ros2 topic echo /imu --qos-profile sensor_data --once
ros2 topic hz /scan --qos-profile sensor_data
```

### 3.3 Services
Services = request/response calls.

Examples:
- Controller manager service APIs (`list_controllers`, load/switch controller)
- SLAM map save service

CLI:

```bash
ros2 service list
ros2 service type /slam_toolbox/save_map
```

### 3.4 Actions
Actions = long-running goals with feedback and result.

In Nav2:
- `NavigateToPose` action server handles goal execution.

CLI:

```bash
ros2 action list
ros2 action info /navigate_to_pose
```

### 3.5 Parameters
Parameters are runtime configuration values for nodes.

Example:
- `bringup/config/diffbot_controllers.yaml` configures `diffbot_base_controller`.

CLI:

```bash
ros2 param list /diffbot_base_controller
ros2 param get /diffbot_base_controller wheel_radius
```

### 3.6 TF (Transforms)
TF tracks robot frame relationships.

Key frames:
- `odom`
- `base_footprint`
- `base_link`
- `laser_scan_frame`

CLI:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

### 3.7 QoS
QoS controls delivery reliability/latency.

In this repo:
- Sensor topics use sensor-data style QoS (best effort, low latency).
- If echo shows no messages, try sensor QoS:

```bash
ros2 topic echo /scan --qos-profile sensor_data
ros2 topic echo /imu --qos-profile sensor_data
```

---

## 4. Workspace and Build

```bash
cd ~/ros2_ws
colcon build --packages-select vmxpi_ros2
source install/setup.bash
```

Check package visibility:

```bash
ros2 pkg list | grep vmxpi_ros2
ros2 pkg prefix vmxpi_ros2
```

---

## 5. Lab 1: Run Simulation and Inspect ROS Graph

Start simulation:

```bash
ros2 launch vmxpi_ros2 diffbot_gz_sim.launch.py gui:=true use_gz_sim:=true
```

Inspect:

```bash
ros2 node list
ros2 topic list
ros2 topic echo /clock --once
ros2 topic echo /diffbot_base_controller/odom --once
```

What to learn:
- How launch starts multiple nodes.
- How `/clock` drives simulated time.
- How odometry is produced by the controller.

---

## 6. Lab 2: Control the Robot

Keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True --remap cmd_vel:=/diffbot_base_controller/cmd_vel
```

Joystick (from launch):

```bash
ros2 launch vmxpi_ros2 diffbot_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true
```

Monitor command and output:

```bash
ros2 topic echo /diffbot_base_controller/cmd_vel
ros2 topic echo /diffbot_base_controller/cmd_vel_out
```

What to learn:
- Command topic interface and message type expectations.
- Why stamped twist is used in this stack.

---

## 7. Lab 3: Mapping (SLAM Toolbox)

Start mapping:

```bash
ros2 launch vmxpi_ros2 nav2_mapping_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true
```

Drive robot to cover the map area, then save:

```bash
mkdir -p ~/ros2_ws/src/vmxpi_ros2/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/ros2_ws/src/vmxpi_ros2/maps/my_map'}}"
```

What to learn:
- Online SLAM data flow: scan + odom + TF -> map.
- Difference between map generation and localization.

---

## 8. Lab 4: Navigation (Nav2 + AMCL)

Run navigation with saved map:

```bash
ros2 launch vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  gui:=true use_gz_sim:=true use_joystick:=true \
  map:=$HOME/ros2_ws/src/vmxpi_ros2/maps/my_map.yaml
```

In RViz:

1. Set fixed frame to `map`.
2. Click `2D Pose Estimate` once.
3. Send `Nav2 Goal`.

What to learn:
- Lifecycle-managed Nav2 nodes.
- Why AMCL needs initial pose.
- Separation of localization (`map->odom`) and control (`odom->base_link`).

---

## 9. ros2_control in This Project

Main components:
- `controller_manager`
- `joint_state_broadcaster`
- `diffbot_base_controller`
- `gz_ros2_control` backend in simulation
- Titan hardware backend for real robot mode

Useful commands:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Expected active controllers:
- `joint_state_broadcaster`
- `diffbot_base_controller`

---

## 10. Real Robot Mode (Titan/VMX)

Launch without simulation:

```bash
ros2 launch vmxpi_ros2 diffbot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false
```

Key point:
- Same repository and mostly same ROS interfaces for sim and hardware.
- This reduces integration drift between development and deployment.

---

## 11. Debugging Checklist

### 11.1 No movement
- Check controller status: `ros2 control list_controllers`
- Check command topic: `ros2 topic echo /diffbot_base_controller/cmd_vel`

### 11.2 TF issues
- Check TF stream: `ros2 topic echo /tf --once`
- Inspect frame tree: `ros2 run tf2_tools view_frames`

### 11.3 Sensor topic exists but no data
- Confirm publishers: `ros2 topic info /imu -v`
- Use sensor QoS in echo.
- Verify bridge source topic and remapping in launch.

### 11.4 RViz "jump back in time"
- Usually multiple `/clock` publishers or stale processes.
- Stop old launches and restart cleanly.

---

## 12. Suggested Training Schedule

Session 1 (2 hours):
- ROS 2 graph basics
- Run simulation
- Inspect nodes/topics/TF

Session 2 (2 hours):
- Teleop
- ros2_control internals
- QoS and sensor debugging

Session 3 (2 hours):
- SLAM mapping
- Save map
- Nav2 localization and goal navigation

Session 4 (1-2 hours):
- Real hardware bringup
- Failure analysis and recovery drills

---

## 13. Reference Commands (Fast Copy)

```bash
# Build
cd ~/ros2_ws && colcon build --packages-select vmxpi_ros2 && source install/setup.bash

# Sim
ros2 launch vmxpi_ros2 diffbot_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true

# Mapping
ros2 launch vmxpi_ros2 nav2_mapping_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true

# Navigation
ros2 launch vmxpi_ros2 nav2_navigation_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true map:=$HOME/ros2_ws/src/vmxpi_ros2/maps/my_map.yaml

# Debug
ros2 control list_controllers
ros2 topic info /imu -v
ros2 topic echo /imu --qos-profile sensor_data --once
ros2 run tf2_ros tf2_echo odom base_link
```
