# studica_vmxpi_ros2

ROS 2 Humble package for Titan/VMX hardware and Gazebo Sim simulation of the robot platform.  
Supports 2WD, 4WD, Mecanum, and Omni robot configurations with a unified launch entry point.

**Maintainer:** Mohammad Alshamsi (`alshamsi.mohammad@gmail.com`)

---

## Table of Contents

1. [Training Material](#training-material)
2. [Dependencies](#dependencies)
3. [Build](#build)
4. [Quick Start — Simulation](#quick-start--simulation)
5. [Available Worlds](#available-worlds)
6. [Robot Profiles](#robot-profiles)
7. [Mapping (SLAM)](#mapping-slam)
8. [Navigation (Nav2)](#navigation-nav2)
9. [Hardware — Real Robot](#hardware--real-robot)
   - [VMXPi hardware and remote PC RViz (recommended)](#vmxpi-hardware-and-remote-pc-rviz-recommended)
   - [Recommended workflow: configure, calibrate, and visualize](#recommended-workflow-configure-calibrate-and-visualize)
   - [Raspberry Pi USB buffer](#raspberry-pi-usb-buffer-persistent)
   - [Raspberry Pi performance tuning](#raspberry-pi-4-performance-tuning)
10. [Remote Control](#remote-control)
11. [Package Structure](#package-structure)
12. [How It Works](#how-it-works)
13. [Launch Migration](#launch-migration)
14. [Profile Automation](#profile-automation)
15. [Gazebo Sim Notes](#gazebo-sim-notes)
16. [System Architecture](#system-architecture-best-performance)
17. [Motor Smoke Test](#motor-smoke-test)
18. [Troubleshooting](#troubleshooting)
19. [License](#license)

---

## Training Material

Use these docs in order for classroom/lab onboarding:

1. `docs/ROS2_TRAINING.md` — hands-on labs: bringup, teleop, SLAM, Nav2, debugging
2. `docs/ARCHITECTURE.md` — runtime architecture and data flow
3. `docs/PROFILE_AUTHORING.md` — create and validate new robot profiles
4. `docs/LAUNCH_MIGRATION.md` — legacy launch command migration

---

## Dependencies

### Core runtime + Gazebo Sim

```bash
sudo apt install -y \
  ros-humble-backward-ros \
  ros-humble-hardware-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-mecanum-drive-controller \
  ros-humble-imu-sensor-broadcaster \
  ros-humble-joint-state-broadcaster \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros-gzharmonic-bridge \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-tf2-ros \
  ros-humble-rviz2 \
  ros-humble-ros2controlcli \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joy \
  ros-humble-rmw-cyclonedds-cpp \
  python3-yaml
```

### Mapping and navigation

Required for `nav2_mapping_gz_sim.launch.py` and `nav2_navigation_gz_sim.launch.py`:

```bash
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

Or install the full Gazebo Harmonic meta-package:

```bash
sudo apt install -y ros-humble-ros-gzharmonic
```

### Source dependencies

Clone into the same workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/MohammadRobot/studica_drivers.git
git clone https://github.com/MohammadRobot/studica_ros2_control.git
git clone https://github.com/MohammadRobot/ydlidar_ros2_driver.git
git clone https://github.com/MohammadRobot/OrbbecSDK_ROS2.git
```

---

## Build

### Gazebo Harmonic overlay (required for ROS 2 Humble)

`gz_ros2_control` for Harmonic must be built from source:

```bash
cd ~/ros2_ws/src
git clone --branch humble --depth 1 https://github.com/ros-controls/gz_ros2_control.git

cd ~/ros2_ws
export GZ_VERSION=harmonic
colcon build --packages-select gz_ros2_control --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Verify overlay is active (expected path should be inside your workspace):

```bash
ros2 pkg prefix gz_ros2_control
# Expected: /home/<user>/ros2_ws/install/gz_ros2_control
```

### Main build

```bash
cd ~/ros2_ws
colcon build --packages-select studica_drivers studica_ros2_control studica_vmxpi_ros2
source install/setup.bash
```

### Optional: LiDAR driver (for hardware stability)

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver \
  --allow-overriding ydlidar_ros2_driver --symlink-install
source install/setup.bash
```

### Optional: Camera packages (Release build required)

```bash
cd ~/ros2_ws
colcon build \
  --packages-select orbbec_camera orbbec_camera_msgs orbbec_description \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> **Conda users:** make sure `colcon` resolves to system Python, or install missing ROS Python deps (e.g. `catkin_pkg`) in the active Conda env.

---

## Quick Start — Simulation

**Step 1 — Build and source:**

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver studica_drivers studica_ros2_control studica_vmxpi_ros2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

**Step 2 — Launch Gazebo Sim:**

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_4wd \
  gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false
```

On a slow machine, use low-load settings:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_4wd \
  gui:=true gz_headless:=true \
  sim_camera_width:=320 sim_camera_height:=240 sim_camera_update_rate:=10.0 \
  sim_lidar_samples:=120 sim_lidar_update_rate:=10.0 sim_lidar_visualize:=false \
  sim_imu_update_rate:=50.0 \
  use_joystick:=true use_ground_truth_odom_tf:=false
```

**Step 3 — Build a map with SLAM:**

Install Nav2 + SLAM if not already installed:

```bash
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
```

Launch mapping:

```bash
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  robot_profile:=class_4wd world:=office_map \
  gui:=true use_joystick:=true use_ground_truth_odom_tf:=false
```

**Step 4 — Save the map:**

```bash
mkdir -p "$HOME/ros2_ws/src/studica_vmxpi_ros2/maps"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map'}}"
```

**Step 5 — Launch navigation:**

```bash
ros2 launch studica_vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  robot_profile:=class_4wd world:=office_map \
  gui:=true use_joystick:=false use_ground_truth_odom_tf:=false \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

**In RViz for navigation:**
- `nav2_navigation_gz_sim.launch.py` loads `nav2_navigation.rviz` by default (Fixed Frame: `map`).
- Click `2D Pose Estimate` once to initialize AMCL, then use `Nav2 Goal`.
- If the map looks empty right after startup, toggle the `Map` display once.
- Keep `use_joystick:=false` while testing autonomous goals.
- If goals show `Goal succeeded` but the robot does not move, verify RViz is on sim time:

```bash
ros2 param set /rviz2 use_sim_time true
```

---

## Available Worlds

Pass any of these short names as `world:=<name>` — no full path needed.

| Short name | File | World name | Description |
|---|---|---|---|
| `diff_drive` | `diff_drive_world.sdf` | `default` | Simple open world for basic drive testing |
| `office_map` | `office_map.sdf` | `default` | Office-style environment for SLAM and Nav2 |
| `maze` | `maze_world.sdf` | `maze` | 4 m × 4 m three-chamber Z-path maze with goal marker |

**Maze layout** (`world:=maze`):

```
  y=+3 ┌─────────────────────────────────────┐  (solid north wall)
       │         north chamber               │
       │              ★ goal (0, 2.5)        │
  y=+1 ├──────────┤                ├─────────┤  H2 — gap on west side
       │       middle chamber      │         │
  y=-1 ├──────────────────┤        ├─────────┤  H1 — gap on east side
       │       south chamber                 │
  y=-3 ├────────┤       ├─────────────────────┤  south wall — 0.8 m entry gap
                 enter here
```

Spawn the robot at the entry point:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_4wd \
  spawn_y:=-2.5 gui:=true use_joystick:=true
```

> When `world_name` is not specified, it is set automatically (`maze` world → `world_name:=maze`, others → `world_name:=default`).

---

## Robot Profiles

Select a profile with `robot_profile:=<name>`:

| Profile | Drive type | Use case |
|---|---|---|
| `class_2wd` | Differential (2-wheel) | Classroom, simple navigation |
| `class_4wd` | Differential (4-wheel) | Classroom, general purpose |
| `class_mecanum` | Mecanum (holonomic) | Strafing demos |
| `class_omni` | Omni / X-drive (holonomic) | Full omnidirectional motion |
| `training_2wd` | Differential (2-wheel) | Training exercises |
| `training_4wd` | Differential (4-wheel) | Training exercises (default) |

Profile files live under `bringup/config/profiles/<name>/`:
- `robot_profile.yaml` — URDF geometry and hardware mapping
- `robot_controllers.yaml` — controller types and tuning

**Quick profile examples:**

```bash
# 2WD diff drive
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_2wd gui:=true use_joystick:=true

# 4WD diff drive (default classroom profile)
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_4wd gui:=true use_joystick:=true

# Mecanum (holonomic)
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_mecanum gui:=true use_joystick:=true

# Omni / X-drive (holonomic)
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim world:=maze robot_profile:=class_omni gui:=true use_joystick:=true
```

Wheel layout options in `robot_profile.yaml`:
- `diff` — 2-wheel differential drive
- `diff_4wd` — 4-wheel differential drive
- `mecanum` — Mecanum drive
- `omni` — X-drive (45° wheel yaw at each corner)

> In ROS 2 Humble, `mecanum_drive_controller` is used for both `mecanum` and `omni` layouts.
> For `diff_4wd`, `mecanum`, and `omni`, all four motor indices must be active (≥ 0).

---

## Mapping (SLAM)

Launch mapping in the office world:

```bash
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  robot_profile:=class_4wd world:=office_map \
  gui:=true use_joystick:=true use_ground_truth_odom_tf:=false
```

Launch mapping in the maze:

```bash
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  robot_profile:=class_4wd world:=maze spawn_y:=-2.5 \
  gui:=true use_joystick:=true use_ground_truth_odom_tf:=false
```

Save the map when done:

```bash
mkdir -p "$HOME/ros2_ws/src/studica_vmxpi_ros2/maps"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map'}}"
```

---

## Navigation (Nav2)

Launch navigation in simulation with a saved map:

```bash
ros2 launch studica_vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  robot_profile:=class_4wd world:=office_map \
  gui:=true use_joystick:=false use_ground_truth_odom_tf:=false \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

Launch navigation on the real robot with a saved map:

```bash
ros2 launch studica_vmxpi_ros2 nav2_navigation_hw.launch.py \
  gui:=true \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

**After launch in RViz:**

1. Set fixed frame to `map` (if not already set).
2. Click `2D Pose Estimate` once to initialize AMCL.
3. Use `Nav2 Goal` to send goals.

`topic_adapter_node` bridges Nav2 topics automatically (`enable_nav2_bridge:=true`):
- `/cmd_vel` (Twist) → selected drive command topic
- Selected drive odom topic → `/odom`

Override the RViz config if needed:

```bash
ros2 launch studica_vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  ... \
  rviz_config_file:=/absolute/path/to/file.rviz
```

---

## Hardware — Real Robot

### Environment setup (`~/.bashrc`)

Add to `~/.bashrc` on both robot host and remote PC:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Add these extra lines on the robot host:

```bash
source /home/vmx/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
```

After editing, run `source ~/.bashrc` once (or open a new terminal).

### Launch hardware mode

VMX hardware requires root (`sudo`) for SPI/CAN/GPIO access:

```bash
sudo -E bash -lc '
cd /home/vmx/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware robot_profile:=class_4wd \
  use_lidar:=true lidar_type:=x2 \
  use_camera:=true \
  orbbec_launch_file:=gemini_e.launch.py \
  orbbec_enable_point_cloud:=true \
  orbbec_enable_ir:=true \
  orbbec_color_width:=640 orbbec_color_height:=480 orbbec_color_fps:=5 \
  orbbec_depth_width:=640 orbbec_depth_height:=480 orbbec_depth_fps:=5
'
```

> `mode:=gz_sim` does **not** require `sudo`.

If you run as `root` persistently, add the same environment lines to `/root/.bashrc`.

### VMXPi hardware and remote PC RViz (recommended)

If the VMXPi cannot open RViz reliably, use a split setup:

1. Run hardware bringup on VMXPi (root, no GUI):

```bash
sudo -E bash -lc '
cd /home/vmx/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware robot_profile:=class_4wd gui:=false
'
```

2. Run RViz and operator tools on remote PC (non-root):

```bash
source ~/.bashrc
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -E "^/tf$|^/odom$|^/scan$|^/imu$"
rviz2 -d ~/ros2_ws/src/studica_vmxpi_ros2/description/robot/rviz/robot.rviz
```

For navigation visualization, use:

```bash
rviz2 -d ~/ros2_ws/src/studica_vmxpi_ros2/description/robot/rviz/nav2_navigation.rviz
```

Keep `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, and `RMW_IMPLEMENTATION` the same on both hosts.

### Disable auto-start for LiDAR or camera

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware use_lidar:=false use_camera:=false
```

### YDLIDAR options

Default behavior in hardware mode:

- If launch arg `lidar_type` is set, that value is used.
- If launch arg `lidar_type` is empty, launch uses `hardware.lidar_type` from `robot_profile.yaml`.
- If `hardware.lidar_type` is not set, fallback is `tmini`.
- `ydlidar_params_file` overrides `lidar_type`.

Use a model preset:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware lidar_type:=tmini
```

Use a custom params file (overrides `lidar_type`):

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware ydlidar_params_file:=/path/to/ydlidar.yaml
```

### Orbbec camera options

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  orbbec_launch_file:=gemini_e.launch.py \
  orbbec_camera_name:=camera \
  orbbec_serial_number:=<serial_number> \
  orbbec_enable_point_cloud:=true
```

> `bringup.launch.py` auto-aligns the Orbbec `base_link → camera_link` TF from the selected `robot_profile`. No need to set `base_to_camera_*` manually.

**USB2 low-bandwidth profile:**

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  orbbec_launch_file:=gemini_e.launch.py \
  orbbec_enable_point_cloud:=false \
  orbbec_enable_ir:=false \
  orbbec_color_width:=320 orbbec_color_height:=240 orbbec_color_fps:=5 \
  orbbec_depth_width:=320 orbbec_depth_height:=240 orbbec_depth_fps:=5
```

**USB2 depth-only fallback (if still disconnecting):**

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  orbbec_launch_file:=gemini_e.launch.py \
  orbbec_enable_point_cloud:=false \
  orbbec_enable_color:=false \
  orbbec_enable_ir:=false \
  orbbec_depth_width:=320 orbbec_depth_height:=240 orbbec_depth_fps:=5
```

### Known-good hardware checklist

> Hardware sensor testing may require root on default VMXPi setups. Prefer `sudo -E` so ROS environment variables are preserved.

1. Build and source YDLIDAR driver:

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver \
  --allow-overriding ydlidar_ros2_driver --symlink-install
source ~/ros2_ws/install/setup.bash
```

2. Build and source Orbbec packages in Release:

```bash
cd ~/ros2_ws
colcon build \
  --packages-select orbbec_camera orbbec_camera_msgs orbbec_description \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```

3. Optional standalone sensor checks:

```bash
sudo -E bash -lc '
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_tmini.launch.py
'

sudo -E bash -lc '
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch orbbec_camera gemini_e.launch.py
'
```

4. Launch integrated hardware stack:

```bash
sudo -E bash -lc '
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
'
```

5. Verify topics and controllers:

```bash
ros2 topic list | grep -E "^/scan$|^/imu$|^/camera/color/image_raw$|^/camera/depth/image_raw$"
ros2 control list_controllers
```

### Production health check

```bash
# Core topics
ros2 topic list | grep -E "^/cmd_vel$|^/odom$|^/imu$|^/scan$"

# Camera topics (when use_camera:=true)
ros2 topic list | grep -E "^/camera/color/image_raw$|^/camera/depth/image_raw$"

# Controllers (expect joint_state_broadcaster, imu_sensor_broadcaster, and drive controller = active)
ros2 control list_controllers

# IMU rate and data
ros2 topic hz /imu
ros2 topic echo /imu --qos-profile sensor_data --once

# Odometry
ros2 topic echo /odom --once
```

### Recommended workflow: configure, calibrate, and visualize

Use this sequence for reliable bringup and repeatable tuning on real hardware.

1. Configure one source of truth for drive + IMU:
   - Start from profile files:
     - `bringup/config/profiles/<profile>/robot_profile.yaml`
     - `bringup/config/profiles/<profile>/robot_controllers.yaml`
   - For this repo's classroom profile:
     - `bringup/config/profiles/class_4wd/robot_profile.yaml`
     - `bringup/config/profiles/class_4wd/robot_controllers.yaml`
   - Launch hardware stack:
     ```bash
     sudo -E bash -lc '
     source /opt/ros/humble/setup.bash
     source ~/ros2_ws/install/setup.bash
     ros2 launch studica_vmxpi_ros2 bringup.launch.py \
       mode:=hardware robot_profile:=class_4wd gui:=false
     '
     ```
   - If you also run `studica_ros2_control` sensor nodes, use `config/params_sensors.yaml` or
     `config/params_imu.yaml` carefully to avoid duplicate motor/IMU publishers.

2. Calibrate in fixed order (safest and fastest):
   - Put robot on blocks, then run motor smoke test:
     ```bash
     sudo /home/vmx/ros2_ws/src/studica_vmxpi_ros2/scripts/motor_smoke_test.sh
     ```
   - Tune in this order:
     - Motor and encoder polarity (`invert_*_motor`, `invert_*_encoder`)
     - Geometry (`wheel_radius`, `wheel_separation`)
     - Motion limits and smoothing (`publish_rate`, `velocity_rolling_window_size`)
     - IMU covariance values in `imu_sensor_broadcaster`
   - Validate each iteration:
     ```bash
     ros2 control list_controllers
     ros2 topic hz /imu
     ros2 topic echo /imu --qos-profile sensor_data --once
     ros2 topic echo /odom --once
     ```

3. Visualize in RViz (and use plotting tools for non-RViz data):
   - For bringup/sensors:
     - Use `description/robot/rviz/robot.rviz` (`Fixed Frame: odom`)
   - For navigation:
     - Use `description/robot/rviz/nav2_navigation.rviz` (`Fixed Frame: map`)
   - Recommended RViz displays:
     - `RobotModel`, `TF`, `LaserScan (/scan)`, `Odometry (/odom)`
     - `Map` on `/map` for Nav2/SLAM (Reliable + Transient Local)
     - `Range` displays for ultrasonic/IR topics (for example `/ultrasonic_range1`, `/ir_range1`)
   - For motor raw arrays like `/titan_encoders` (`std_msgs/Float32MultiArray`), use `rqt_plot` or
     PlotJuggler instead of RViz.

4. Keep wheel radius values consistent:
   - Odometry quality depends on matching wheel-radius values across profile/controller/URDF paths.
   - In `class_4wd`, `xacro.wheel_radius` and controller `wheel_radius` are `0.0625` while
     `hardware.wheel_radius` is `0.05`. Align these to your measured physical wheel radius.

### Raspberry Pi USB buffer (persistent)

On Raspberry Pi, increase USBFS memory for stable high-bandwidth USB sensors (cameras, LiDAR):

```bash
FILE=/boot/firmware/cmdline.txt
[ -f /boot/cmdline.txt ] && FILE=/boot/cmdline.txt

grep -q 'usbcore.usbfs_memory_mb=' "$FILE" \
  && sudo sed -i 's/usbcore\.usbfs_memory_mb=[^ ]*/usbcore.usbfs_memory_mb=128/' "$FILE" \
  || sudo sed -i '1 s|$| usbcore.usbfs_memory_mb=128|' "$FILE"

sudo reboot
```

Verify after reboot (`128` expected):

```bash
cat /sys/module/usbcore/parameters/usbfs_memory_mb
```

Disable USB autosuspend (recommended for unstable cameras/sensors):

```bash
FILE=/boot/firmware/cmdline.txt
[ -f /boot/cmdline.txt ] && FILE=/boot/cmdline.txt

grep -q 'usbcore.autosuspend=' "$FILE" \
  && sudo sed -i 's/usbcore\.autosuspend=[^ ]*/usbcore.autosuspend=-1/' "$FILE" \
  || sudo sed -i '1 s|$| usbcore.autosuspend=-1|' "$FILE"

sudo reboot
```

Verify after reboot (`-1` expected):

```bash
cat /sys/module/usbcore/parameters/autosuspend
```

### Raspberry Pi 4 performance tuning

For lower control-loop jitter on real hardware:

**1. Switch to headless boot:**

```bash
sudo systemctl set-default multi-user.target
```

**2. Disable unused services:**

```bash
sudo systemctl disable --now gdm cups cups-browsed avahi-daemon bluetooth hciuart ModemManager
```

**3. Set CPU governor to performance:**

```bash
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
```

**4. Reduce swap aggressiveness:**

```bash
echo 'vm.swappiness=10' | sudo tee /etc/sysctl.d/99-robot-performance.conf
sudo sysctl -p /etc/sysctl.d/99-robot-performance.conf
```

**5. Use Cyclone DDS in every shell:**

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

**6. Optional: disable Snap background services:**

```bash
sudo systemctl stop snapd.service snapd.socket snapd.seeded.service
sudo systemctl disable snapd.service snapd.socket snapd.seeded.service \
  snapd.refresh.timer snapd.snap-repair.timer
sudo systemctl mask snapd.service snapd.socket snapd.seeded.service
```

Verify after reboot:

```bash
systemctl get-default
for c in /sys/devices/system/cpu/cpu[0-9]*; do
  echo "$(basename "$c"): $(cat "$c/cpufreq/scaling_governor")"
done
uptime && free -h
journalctl -k --no-pager | grep -Ei "under-?voltage|throttl|oom|thermal"
vcgencmd get_throttled   # healthy: throttled=0x0
```

---

## Remote Control

Use the command topic that matches the selected robot profile:

| Profile | Command topic |
|---|---|
| `training_2wd`, `training_4wd`, `class_2wd`, `class_4wd` | `/robot_base_controller/cmd_vel` |
| `class_mecanum` | `/mecanum_base_controller/reference` |
| `class_omni` | `/omni_base_controller/reference` |

On the remote PC, source your environment:

```bash
source ~/.bashrc
```

Check connectivity (optional):

```bash
ros2 topic list | grep -E "cmd_vel|reference|odom|base_controller"
```

**Keyboard teleop:**

```bash
CMD_TOPIC=/robot_base_controller/cmd_vel
# CMD_TOPIC=/mecanum_base_controller/reference
# CMD_TOPIC=/omni_base_controller/reference
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=true -p frame_id:=base_link \
  --remap cmd_vel:=${CMD_TOPIC}
```

**Joystick teleop:**

```bash
source ~/ros2_ws/install/setup.bash
CMD_TOPIC=/robot_base_controller/cmd_vel
ros2 launch studica_ros2_control gamepad_launch.py \
  cmd_vel_topic:=${CMD_TOPIC} publish_stamped:=true
```

Lower speeds if the real robot is too fast:

```bash
ros2 launch studica_ros2_control gamepad_launch.py \
  cmd_vel_topic:=${CMD_TOPIC} publish_stamped:=true \
  linear_scale:=0.20 angular_scale:=0.60 \
  turbo_multiplier:=1.0 button_turbo:=-1
```

Joystick tuning parameters: `linear_scale`, `angular_scale`, `deadzone`, `turbo_multiplier`, `button_turbo` (`-1` disables turbo).

> When launching `bringup.launch.py mode:=gz_sim` from another PC, keep `use_joystick:=false` (default) to avoid two joystick publishers.

---

## Package Structure

```text
studica_vmxpi_ros2/
├── CMakeLists.txt
├── package.xml
├── README.md
├── bringup/
│   ├── launch/
│   │   ├── bringup.launch.py              ← unified entry point (use this)
│   │   ├── robot_gz_sim.launch.py         ← compatibility wrapper
│   │   ├── robot_bringup.launch.py        ← compatibility wrapper
│   │   ├── _launch_helpers.py             ← shared launch utilities
│   │   ├── _launch_gz.py                  ← Gazebo-specific handlers
│   │   ├── _launch_sensors.py             ← sensor launch handlers
│   │   ├── nav2_mapping_gz_sim.launch.py
│   │   ├── nav2_navigation_gz_sim.launch.py
│   │   ├── nav2_navigation_hw.launch.py
│   │   ├── camera_hw.launch.py
│   │   └── lidar_hw.launch.py
│   └── config/
│       ├── profiles/
│       │   ├── class_2wd/
│       │   ├── class_4wd/
│       │   ├── class_mecanum/
│       │   ├── class_omni/
│       │   ├── training_2wd/
│       │   └── training_4wd/
│       ├── profile_template/
│       ├── slam_toolbox_mapper_params.yaml
│       └── ydlidar_x2_hw.yaml
├── description/
│   ├── urdf/
│   ├── robot/urdf/
│   ├── ros2_control/
│   ├── gz/worlds/                         ← SDF worlds (diff_drive, office_map, maze)
│   └── meshes/
├── hardware/
│   ├── vmx_system.cpp                     ← Titan/VMX hardware interface
│   ├── sim_system.cpp                     ← mock/sim hardware interface
│   └── include/studica_vmxpi_ros2/
├── src/
│   ├── topic_adapter_node.cpp             ← scan/IMU/Nav2 topic adapters
│   └── patrol.cpp
├── scripts/
│   ├── create_profile.sh
│   ├── validate_profiles.py
│   ├── check_project.sh
│   ├── install_git_hooks.sh
│   └── motor_smoke_test.sh
└── docs/
    ├── ROS2_TRAINING.md
    ├── PROFILE_AUTHORING.md
    └── LAUNCH_MIGRATION.md
```

---

## How It Works

Runtime flow:

1. Start from `bringup.launch.py` with `mode:=<gz_sim|hardware|mock>` and `robot_profile:=<name>`.
2. The launch validates the selected profile (`robot_profile.yaml` + `robot_controllers.yaml`) before starting nodes.
3. URDF is generated from Xacro (`description/urdf/robot.urdf.xacro`) using profile values and controller config.
4. Mode-specific startup:
   - `gz_sim` — starts Gazebo Sim, spawns the robot, bridges `/clock`, `/scan`, and simulated camera topics, relays `/scan_raw → /scan` with normalized frame ID.
   - `hardware` — starts `ros2_control_node` with VMX/Titan hardware plugin (`vmx_system.cpp`), then controllers.
   - `mock` — starts `ros2_control_node` without real hardware for software-only testing.
5. Common control layer runs `joint_state_broadcaster`, `imu_sensor_broadcaster`, and the selected drive controller from profile.
6. `topic_adapter_node` provides API compatibility:
   - IMU alias: `/imu_sensor_broadcaster/imu → /imu` (with odom fallback in sim when needed).
   - Nav2 bridge: `/cmd_vel (Twist) → /<drive_controller>/cmd_vel|reference (TwistStamped)` and odom aliasing.
7. Optional features:
   - LiDAR launch in hardware mode (`use_lidar:=true`).
   - Camera launch in hardware mode (`use_camera:=true`).
   - Joystick teleop (`use_joystick:=true`).
   - Mapping and navigation launches include bringup then add SLAM/Nav2.

Robot differences are encoded in profiles under `bringup/config/profiles/<profile_name>/`. Create a new profile, run `scripts/check_project.sh`, then launch with `robot_profile:=<new_profile>`.

---

## Launch Migration

Use `bringup.launch.py` as the single entry point for new labs and robots.

| Legacy command | Unified command |
|---|---|
| `robot_gz_sim.launch.py gui:=true use_gz_sim:=true` | `bringup.launch.py mode:=gz_sim gui:=true` |
| `robot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false` | `bringup.launch.py mode:=hardware` |
| `robot_bringup.launch.py ...` | `bringup.launch.py mode:=<gz_sim\|hardware\|mock> ...` |

Deprecation policy:
- Since March 3, 2026, compatibility wrappers are read-only.
- New features are added only to `bringup.launch.py`.
- Wrapper removal not planned before January 1, 2027.
- Migration examples: `docs/LAUNCH_MIGRATION.md`

---

## Profile Automation

Create a new robot profile from template:

```bash
cd ~/ros2_ws/src/studica_vmxpi_ros2
scripts/create_profile.sh my_robot_4wd
```

Validate all profiles locally:

```bash
python3 scripts/validate_profiles.py --profiles-dir bringup/config/profiles
```

Run the full local maintenance check (syntax + profile validation):

```bash
scripts/check_project.sh
```

Enable local pre-commit validation:

```bash
scripts/install_git_hooks.sh
```

When relevant launch or profile files are staged, pre-commit runs `scripts/check_project.sh` automatically. CI also validates profiles in `.github/workflows/profile-validation.yml`.

---

## Gazebo Sim Notes

- Unified Gazebo Sim launch: `bringup.launch.py mode:=gz_sim`
- Gazebo Sim Nav2 launches: `nav2_mapping_gz_sim.launch.py`, `nav2_navigation_gz_sim.launch.py`
- Real robot Nav2 launch: `nav2_navigation_hw.launch.py`

**Sensor topics in simulation:**

| Topic | Message type |
|---|---|
| `/scan` | `sensor_msgs/LaserScan` |
| `/imu` | `sensor_msgs/Imu` |
| `/camera/color/image_raw` | `sensor_msgs/Image` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/image_raw` | `sensor_msgs/Image` |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |

---

## System Architecture (Best Performance)

Split workload across two machines for best stability:

| Component | Run on | Why |
|---|---|---|
| `studica_drivers` + VMX hardware interfaces | Robot (VMX) | Direct hardware access, lowest latency |
| `controller_manager` + controllers | Robot (VMX) | Keeps motor/IMU control loop local |
| `ydlidar_ros2_driver` | Robot (VMX) | Serial capture stays local to `/dev/ttyUSB*` |
| `orbbec_camera` | Robot (VMX) | USB capture stays local to robot USB bus |
| TF publishers tied to sensors | Robot (VMX) | Robot frame tree stays synced with real sensors |
| Teleop nodes, `rviz2` | Remote PC | Operator UI is not control-critical |
| Nav2 planning / BT tools | Remote PC | CPU-heavy; offloading improves VMX performance |
| SLAM (`slam_toolbox`) | Remote PC | Mapping load stays off the control path |
| Debug tools (`rqt`, `ros2 bag`) | Remote PC | Prevents debug from stealing robot compute |

Use the same ROS 2 networking settings on both hosts (`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`).

---

## Motor Smoke Test

Validates motor control across `studica_drivers`, `studica_ros2_control`, and `studica_vmxpi_ros2`.

> **Safety:** put the robot on blocks so wheels spin freely.

```bash
sudo /home/vmx/ros2_ws/src/studica_vmxpi_ros2/scripts/motor_smoke_test.sh
```

Rebuild all three packages before testing:

```bash
sudo /home/vmx/ros2_ws/src/studica_vmxpi_ros2/scripts/motor_smoke_test.sh --build
```

Logs are written to `/tmp/studica_motor_smoke_YYYYMMDD_HHMMSS/`.

---

## Troubleshooting

### PC is very slow when running `mode:=gz_sim`

- Ensure only one Gazebo session is running:
  ```bash
  pkill -f "ros2 launch studica_vmxpi_ros2 bringup.launch.py"; pkill -f "gz sim"
  ```
- Clear stale FastDDS shared-memory locks if you see `RTPS_TRANSPORT_SHM` errors:
  ```bash
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
  ```
- Use low-load sim settings:
  ```bash
  ros2 launch studica_vmxpi_ros2 bringup.launch.py \
    mode:=gz_sim world:=maze robot_profile:=class_4wd gui:=true \
    gz_headless:=true \
    sim_camera_width:=320 sim_camera_height:=240 sim_camera_update_rate:=10.0 \
    sim_lidar_samples:=120 sim_lidar_update_rate:=10.0 sim_lidar_visualize:=false \
    sim_imu_update_rate:=50.0
  ```

### Teleop runs but robot does not move

- If launch runs as `root` and teleop as a normal user, DDS discovery may succeed but topic data can fail.
- Avoid `sudo su`; prefer `sudo -E` so the ROS environment is preserved.
- Short-term: run teleop as `sudo -E`. Long-term: grant VMX/SPI/CAN permissions to the `vmx` user.

### `No transform from [front_left_wheel] to [odom]` in RViz

- Usually a startup timing issue — wait a few seconds, or increase `rviz_start_delay`:
  ```bash
  ros2 launch studica_vmxpi_ros2 bringup.launch.py \
    mode:=gz_sim robot_profile:=class_4wd rviz_start_delay:=14.0 gui:=true
  ```
- Verify controller spawner logs: `Configured and activated robot_base_controller`
- Stop stale processes and relaunch:
  ```bash
  pkill -f "gz sim"; pkill -f "ros2 launch studica_vmxpi_ros2 bringup.launch.py"
  ```
- Isolate DDS domain if `TF_OLD_DATA` warnings repeat:
  ```bash
  ROS_DOMAIN_ID=66 ros2 launch studica_vmxpi_ros2 bringup.launch.py \
    mode:=gz_sim robot_profile:=class_4wd gui:=true
  ```

### Robot appears in Gazebo but does not move with joystick

```bash
# Check joystick topic has non-zero values
ros2 topic echo /joy --once

# Check controller input topic has both publisher and subscriber
ros2 topic info /robot_base_controller/cmd_vel

# Check command is accepted
ros2 topic echo /robot_base_controller/cmd_vel_out \
  --qos-durability transient_local --once
```

If using a custom joystick launcher in sim, ensure stamped commands use a valid timestamp (`use_sim_time:=false` in the gamepad node).

### `librmw_cyclonedds_cpp.so` not found

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo $RMW_IMPLEMENTATION
sudo -E bash -lc 'echo $RMW_IMPLEMENTATION'
```

### `robot_base_controller` fails with `expected [double] got [integer]`

Set these as floats in `bringup/config/profiles/<profile>/robot_controllers.yaml`:

```yaml
wheel_separation_multiplier: 1.0
left_wheel_radius_multiplier: 1.0
right_wheel_radius_multiplier: 1.0
```

### Map not visible in RViz

- Add a `Map` display on topic `/map` with `Reliable` + `Transient Local`.
- Confirm lifecycle nodes are active: `/map_server`, `/amcl`.

### Map save fails with "Unable to open file"

Run `mkdir -p <target_folder>` first.

### VMX crashes or reboots during `colcon build`

```bash
# Use a low-parallel build
colcon build --executor sequential --parallel-workers 1 \
  --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
```

Also:
- Use a stable 5.1V/3A power supply and good USB-C cable.
- Disconnect high-current USB peripherals during build.
- Add active cooling.
- Increase swap to 4 GB:
  ```bash
  sudo swapoff /swapfile
  sudo fallocate -l 4G /swapfile
  sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
  ```

### Gazebo Sim launch fails with missing package errors

Install:
```bash
sudo apt install -y \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros-gzharmonic-bridge
```

### `nav2_mapping_gz_sim.launch.py` fails with `package 'slam_toolbox' not found`

```bash
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

### `/scan` is empty — bridge logs `Unknown message type [8]` or `[9]`

Mixed Gazebo bridge packages. Fix:
- Use only Harmonic bridge packages (`ros-humble-ros-gzharmonic-*`).
- Remove conflicting `ros-humble-ros-gz-*` bridge packages.

Verify (should show `libgz-transport13`, not `libignition-transport11`):

```bash
ldd /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge | grep transport
```

### Gazebo Sim launch fails with `libgazebo_ros2_control.so` / `libgazebo_ros_*` errors

Stale legacy Gazebo artifacts. Fix:

```bash
rm -rf ~/ros2_ws/build/studica_vmxpi_ros2 ~/ros2_ws/install/studica_vmxpi_ros2
cd ~/ros2_ws && colcon build --packages-select studica_vmxpi_ros2
```

### Robot drives too slowly in simulation

Check wheel joint velocity limits in `description/robot/urdf/robot_description.urdf.xacro`. Low values (e.g. `velocity="1.0"`) cap top speed.

### IMU topic exists but no visible data

Use sensor QoS:

```bash
ros2 topic echo /imu --qos-profile sensor_data
```

### Odometry is noisy or not smooth enough

Increase rates and rolling window in `bringup/config/profiles/<profile>/robot_controllers.yaml`:

```yaml
controller_manager:
  update_rate: 100
robot_base_controller:
  publish_rate: 100.0
  velocity_rolling_window_size: 30
```

---

## License

Licensed under the Apache License, Version 2.0.

- Full license text: `LICENSE`
- Project notices/attribution: `NOTICE`
- ROS package metadata: `package.xml` (`<license>Apache-2.0</license>`)
