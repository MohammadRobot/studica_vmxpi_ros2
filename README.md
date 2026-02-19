# vmxpi_ros2
## Overview

This package provides ROS 2 support for the VMX-pi hardware and a DiffBot simulation.

## Installation

### Dependencies

Install the necessary ROS 2 packages:

```bash
sudo apt install ros-humble-backward-ros
sudo apt install ros-humble-hardware-interface
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-diff-drive-controller
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-xacro
sudo apt install ros-humble-teleop-twist-keyboard
```

### Build

Build the package and source the setup file:

```bash
cd ~/ros2_ws && colcon build --packages-select vmxpi_ros2 && source install/setup.bash
```

If you use Conda, make sure `colcon` uses system Python (or install `catkin_pkg` in Conda), otherwise the build will fail.

Note: On machines without the VMXPi library, `studica_drivers` builds as a stub and `studica_control` is skipped. Simulation still works.

## Quick Start

```bash
cd ~/ros2_ws
colcon build --packages-select vmxpi_ros2
source install/setup.bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py gui:=true use_gazebo_classic:=true
```

## Usage

### Simulation

To launch the simulation with Gazebo:

```bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py gui:=true use_gazebo_classic:=true
```

If RViz time is out of sync, add `use_sim_time:=true` to the launch arguments.

### Combined Bringup (Drive + Studica Sensors)

To start ros2_control and the Studica sensor stack together:

```bash
ros2 launch vmxpi_ros2 robot_bringup.launch.py use_studica_sensors:=true
```

`use_studica_sensors` defaults to `use_hardware`, so sensors auto-start on real hardware.

### Real Robot

To launch the real robot:

```bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

### VMX Setup

For VMX setup, source the workspace and add the Studica driver path:

```bash
cd  /home/vmx/ros2_ws
source /home/vmx/.bashrc 
source install/setup.bash 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

If you must run as root, add the following lines to `/root/.bashrc`:

```bash
sudo su
source /home/vmx/.bashrc 
source /opt/ros/humble/setup.bash
source /home/vmx/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
```

### Studica Sensors (Optional)

If you want Studica sensor drivers (IMU, ultrasonic, etc.) without taking over motor control:

```bash
ros2 launch studica_control sensors_only.launch.py
```

## Controlling the Robot

### Publish Commands

To control the robot using ROS 2 topics:

```bash
ros2 topic pub --rate 10 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "
twist:
  linear:
    x: 0.5
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5"
```

Or:

```bash
ros2 topic pub /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "base_link"}, twist: {linear: {x: 0.01, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}' -r 10
```

### Teleoperation

To control the robot using the keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True --remap cmd_vel:=/diffbot_base_controller/cmd_vel
```

## Debugging

### Topics

To echo the command velocity topic:

```bash
ros2 topic echo /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped
```

To echo the joint states topic:

```bash
ros2 topic echo /joint_states
```

To get information about the command velocity topic:

```bash
ros2 topic info -v /diffbot_base_controller/cmd_vel
```

### TF Frames

To view the TF frames:

```bash
ros2 run tf2_tools view_frames
```

- If the robots do not spawn in Gazebo, run:

```bash
gazebo --verbose

gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so

```
