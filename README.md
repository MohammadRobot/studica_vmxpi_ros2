# studica_vmxpi_ros2

ROS 2 Humble package for Titan/VMX hardware and Gazebo Sim (`gz sim`) simulation of the DiffBot platform.

## Training Material

Detailed ROS 2 training guide for this project:

- `docs/ROS2_TRAINING.md`

## Dependencies

```bash
sudo apt install -y \
  ros-humble-backward-ros \
  ros-humble-hardware-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros-gzharmonic-bridge \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joy \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

You can also install the Gazebo Harmonic meta package:

```bash
sudo apt install -y ros-humble-ros-gzharmonic
```

Legacy Gazebo Classic support (optional):

```bash
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-plugins
```

Note: Gazebo Classic packages conflict with the Harmonic `gz-*` stack on the same host. Keep Classic in a separate container/VM if needed.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select studica_vmxpi_ros2
source install/setup.bash
```

If you use Conda, make sure `colcon` resolves to system Python, or install missing Python ROS deps in the active Conda env (for example `catkin_pkg`).

## Humble + Harmonic

On ROS 2 Humble, `gz_ros2_control` for Harmonic must be built from source overlay.

```bash
cd ~/ros2_ws/src
git clone --branch humble --depth 1 https://github.com/ros-controls/gz_ros2_control.git

cd ~/ros2_ws
export GZ_VERSION=harmonic
colcon build --packages-select gz_ros2_control --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Verify overlay is active:

```bash
ros2 pkg prefix gz_ros2_control
```

Expected output should be inside your workspace, for example:
`/home/<user>/ros2_ws/install/gz_ros2_control`

## Quick Start

Simulation:

```bash
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py gui:=true use_gz_sim:=true
```

If the robot appears partially below ground, spawn it slightly above zero:

```bash
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py gui:=true use_gz_sim:=true spawn_z:=0.10
```

Simulation with joystick:

```bash
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true
```

Simulation with office world:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py \
  gui:=true use_gz_sim:=true use_joystick:=true \
  world:="${WORLD_SDF}"
```

Real hardware:

```bash
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false
```

## VMX Real Robot Setup

On the VMX/Titan robot, source the workspace and VMX runtime path before launch:

```bash
cd /home/vmx/ros2_ws
source /home/vmx/.bashrc
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false
```

Known-good hardware bringup (single command, runs as root):

```bash
sudo -E bash -lc '
cd /home/vmx/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 diffbot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false
'
```

Why `sudo` is required for hardware mode:

- `use_hardware:=true` loads VMX/Titan hardware drivers (pigpio + SPI/CAN access).
- These low-level interfaces require root privileges on the current VMX HAL setup.
- Simulation mode (`use_gz_sim:=true`) does not require `sudo`.

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
ros2 launch studica_vmxpi_ros2 robot_bringup.launch.py use_studica_sensors:=true
```

`use_studica_sensors` defaults to `use_hardware`, so sensors auto-enable on hardware launches.

## Gazebo Sim Notes

- Gazebo Sim launch: `diffbot_gz_sim.launch.py`
- Gazebo Sim Nav2 launches:
  - `nav2_mapping_gz_sim.launch.py`
  - `nav2_navigation_gz_sim.launch.py`
- Gazebo Sim sensor topics:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/imu` (`sensor_msgs/Imu`)
- Legacy Gazebo Classic launches are still available (`diffbot_gazebo_classic.launch.py`, `nav2_mapping.launch.py`, `nav2_navigation.launch.py`).

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
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py gui:=true use_gz_sim:=true use_joystick:=true
```

Launch mapping with a specific world:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  gui:=true use_gz_sim:=true use_joystick:=true \
  world:="${WORLD_SDF}"
```

Save map:

```bash
mkdir -p "$HOME/ros2_ws/src/studica_vmxpi_ros2/maps"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map'}}"
```

## Navigation (Nav2 + AMCL)

Launch navigation with a saved map:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  gui:=true use_gz_sim:=true use_joystick:=true \
  world:="${WORLD_SDF}" \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

After launch in RViz:

1. Set fixed frame to `map`.
2. Click `2D Pose Estimate` once to initialize AMCL.
3. Use `Nav2 Goal` to send goals.

`nav2_topic_bridge_node` is started automatically for Nav2 launches:

- `/cmd_vel` (`Twist`) -> `/diffbot_base_controller/cmd_vel` (`TwistStamped`)
- `/diffbot_base_controller/odom` -> `/odom`

## Raspberry Pi 4 Performance Tuning (ROS 2 Robot)

For lower control-loop jitter and better runtime stability on real hardware:

1. Switch to headless boot target:

```bash
sudo systemctl set-default multi-user.target
```

2. Disable desktop/auxiliary services not needed on a robot:

```bash
sudo systemctl disable --now gdm cups cups-browsed avahi-daemon bluetooth hciuart ModemManager
```

3. Set CPU governor to `performance`:

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

4. Reduce swap aggressiveness:

```bash
echo 'vm.swappiness=10' | sudo tee /etc/sysctl.d/99-robot-performance.conf
sudo sysctl -p /etc/sysctl.d/99-robot-performance.conf
```

5. Use Cyclone DDS for ROS 2 middleware:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

6. Optional: disable Snap background services on dedicated robot images:

```bash
sudo systemctl stop snapd.service snapd.socket snapd.seeded.service
sudo systemctl disable snapd.service snapd.socket snapd.seeded.service snapd.refresh.timer snapd.snap-repair.timer
sudo systemctl mask snapd.service snapd.socket snapd.seeded.service
```

Verify after reboot:

```bash
systemctl get-default
for c in /sys/devices/system/cpu/cpu[0-9]*; do echo "$(basename "$c"): $(cat "$c/cpufreq/scaling_governor")"; done
uptime
free -h
journalctl -k --no-pager | rg -i "under-?voltage|throttl|oom|thermal"
vcgencmd get_throttled   # expected healthy value: throttled=0x0
```

## Motor Smoke Test (Drivers + ROS 2 Control + ros2_control)

Use the repeatable smoke test script to validate motor control across:

- `studica_drivers` (direct Titan API)
- `studica_ros2_control` (`manual_composition` + `titan_cmd`)
- `studica_vmxpi_ros2` (`ros2_control` + `diffbot_base_controller`)

Safety: put the robot on blocks so wheels can spin freely.

Run:

```bash
sudo /home/vmx/ros2_ws/src/studica_vmxpi_ros2/scripts/motor_smoke_test.sh
```

Optional: rebuild the 3 packages before testing:

```bash
sudo /home/vmx/ros2_ws/src/studica_vmxpi_ros2/scripts/motor_smoke_test.sh --build
```

Logs are written to `/tmp/studica_motor_smoke_YYYYMMDD_HHMMSS/`.

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

VMX crashes or reboots during `colcon build`:

- Use a low-load build command:

```bash
colcon build --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
```

- Use a stable power source (official Raspberry Pi 5.1V/3A PSU recommended) and a good USB-C cable.
- Disconnect high-current USB peripherals during build (for example depth cameras), or use a powered USB hub.
- Increase swap to 4G on low-memory systems:

```bash
sudo swapoff /swapfile
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

- Add active cooling (fan/heatsink) to prevent thermal issues during long builds.

Gazebo Sim launch fails with missing package errors:

- Install Gazebo Sim dependencies:
- `ros-humble-gz-ros2-control`
- `ros-humble-ros-gzharmonic-sim`
- `ros-humble-ros-gzharmonic-bridge`

Gazebo Sim launch fails with `libgazebo_ros2_control.so` / `libgazebo_ros_*` plugin errors:

- You launched a Gazebo Classic `.world` file in `gz sim`.
- Use a Gazebo Sim `.sdf` world, for example:
- `$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf`

Robot drives too slowly in simulation:

- Check wheel joint velocity limits in `description/diffbot/urdf/diffbot_description.urdf.xacro`.
- Low values (for example `velocity="1.0"`) strongly cap top speed.

`/imu` topic exists but no visible data in `ros2 topic echo /imu`:

- Use sensor QoS for echo:
- `ros2 topic echo /imu --qos-profile sensor_data`

Odometry is noisy / not smooth enough:

- Increase controller update/publish rates and velocity rolling window in `bringup/config/diffbot_controllers.yaml`.
- Current defaults are tuned for smoother sim odometry:
  - `controller_manager.update_rate: 100`
  - `diffbot_base_controller.publish_rate: 100.0`
  - `diffbot_base_controller.velocity_rolling_window_size: 30`
