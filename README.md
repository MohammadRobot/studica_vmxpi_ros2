# vmxpi_ros2

ROS 2 Humble package for Titan/VMX hardware and Gazebo Classic simulation of the DiffBot platform.

## Dependencies

```bash
sudo apt install -y \
  ros-humble-backward-ros \
  ros-humble-hardware-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-plugins \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joy \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select vmxpi_ros2
source install/setup.bash
```

If you use Conda, make sure `colcon` resolves to system Python, or install missing Python ROS deps in the active Conda env (for example `catkin_pkg`).

## Quick Start

Simulation:

```bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py gui:=true use_gazebo_classic:=true
```

Simulation with joystick:

```bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py gui:=true use_gazebo_classic:=true use_joystick:=true
```

Real hardware:

```bash
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

## VMX Real Robot Setup

On the VMX/Titan robot, source the workspace and VMX runtime path before launch:

```bash
cd /home/vmx/ros2_ws
source /home/vmx/.bashrc
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch vmxpi_ros2 diffbot_gazebo_classic.launch.py use_hardware:=true
```

If you must run as `root`, add equivalent exports/sourcing to `/root/.bashrc`:

```bash
source /home/vmx/.bashrc
source /opt/ros/humble/setup.bash
source /home/vmx/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
```

## Optional Bringup

Start drive stack plus Studica sensors:

```bash
ros2 launch vmxpi_ros2 robot_bringup.launch.py use_studica_sensors:=true
```

`use_studica_sensors` defaults to `use_hardware`, so sensors auto-enable on hardware launches.

## Control

Keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=True --remap cmd_vel:=/diffbot_base_controller/cmd_vel
```

Joystick teleop publishes to:

```text
/diffbot_base_controller/cmd_vel
```

## Mapping (SLAM Toolbox)

Launch mapping:

```bash
ros2 launch vmxpi_ros2 nav2_mapping.launch.py gui:=true use_gazebo_classic:=true use_joystick:=true
```

Launch mapping with a specific world:

```bash
ros2 launch vmxpi_ros2 nav2_mapping.launch.py gui:=true use_gazebo_classic:=true use_joystick:=true world:=/home/mohammadrobot/ros2_ws/install/vmxpi_ros2/share/vmxpi_ros2/description/gazebo/worlds/office_map.world
```

Save map:

```bash
mkdir -p /home/mohammadrobot/ros2_ws/src/vmxpi_ros2/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/mohammadrobot/ros2_ws/src/vmxpi_ros2/maps/my_map'}}"
```

## Navigation (Nav2 + AMCL)

Launch navigation with a saved map:

```bash
ros2 launch vmxpi_ros2 nav2_navigation.launch.py gui:=true use_gazebo_classic:=true use_joystick:=true world:=/home/mohammadrobot/ros2_ws/install/vmxpi_ros2/share/vmxpi_ros2/description/gazebo/worlds/office_map.world map:=/home/mohammadrobot/ros2_ws/src/vmxpi_ros2/maps/my_map.yaml
```

After launch in RViz:

1. Set fixed frame to `map`.
2. Click `2D Pose Estimate` once to initialize AMCL.
3. Use `Nav2 Goal` to send goals.

`nav2_topic_bridge_node` is started automatically for Nav2 launches:

- `/cmd_vel` (`Twist`) -> `/diffbot_base_controller/cmd_vel` (`TwistStamped`)
- `/diffbot_base_controller/odom` -> `/odom`

## Troubleshooting

`diffbot_base_controller` fails with `expected [double] got [integer]`:

Set these values as floats in `bringup/config/diffbot_controllers.yaml`:

- `wheel_separation_multiplier: 1.0`
- `left_wheel_radius_multiplier: 1.0`
- `right_wheel_radius_multiplier: 1.0`

Map is not visible in RViz:

- Confirm `/map` is published.
- Add a `Map` display on topic `/map` with `Reliable` + `Transient Local`.
- Confirm lifecycle nodes are active: `/map_server`, `/amcl`.

Map save fails with "Unable to open file":

- Create target folder first with `mkdir -p`.
