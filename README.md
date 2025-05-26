# vmxpi_ros2
## Overview

This package provides ROS 2 support for the VMX-pi hardware. It includes launch files and configurations for both simulated and real robots.

## Installation

### Dependencies

Install the necessary ROS 2 packages:

```bash
sudo apt install ros-humble-backward-ros
sudo apt install ros-humble-hardware-interface
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-gazebo-ros2-control
sudo apt install ros-humble-gazebo-plugins
sudo apt install ros-humble-xacro
```

### Build

Build the package and source the setup file:

```bash
cd ~/ros2_ws && colcon build --packages-select vmxpi_ros2 && source install/setup.bash
```

## Usage

### Simulation

To launch the simulation with Gazebo:

```bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py gui:=true use_gazebo_classic:=true
```
meshes
### Real Robot

To launch the real robot:

```bash
sudo su 
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

### VMX Setup

For VMX setup, switch to the root user and source the necessary files:

```bash
sudo su 
cd  /home/vmx/ros2_ws
source /home/vmx/.bashrc 
source install/setup.bash 
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/studica_drivers
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

Update the source file for the root user:

```bash
cp /etc/skel/.bash* ~
```

Add the following lines to `/root/.bashrc `:

```bash
sudo su
source /home/vmx/.bashrc 
source /opt/ros/humble/setup.bash
source /home/vmx/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/studica_drivers
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
ros2 daemon stop
ros2 daemon start
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
Add to .bashrc 

chmod 0700 /run/user/1000 
