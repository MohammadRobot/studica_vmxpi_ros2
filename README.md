# studica_vmxpi_ros2

ROS 2 Humble package for Titan/VMX hardware and Gazebo Sim (`gz sim`) simulation of the robot platform.

## Training Material

Use these docs in order for classroom/lab onboarding:

1. `docs/ROS2_TRAINING.md` (hands-on labs: bringup, teleop, SLAM, Nav2, debugging)
2. `docs/ARCHITECTURE.md` (runtime architecture and data flow)
3. `docs/PROFILE_AUTHORING.md` (create and validate new robot profiles)
4. `docs/LAUNCH_MIGRATION.md` (legacy launch command migration)

## Dependencies

```bash
sudo apt install -y \
  ros-humble-backward-ros \
  ros-humble-hardware-interface \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-mecanum-drive-controller \
  ros-humble-gz-ros2-control \
  ros-humble-ros-gzharmonic-sim \
  ros-humble-ros-gzharmonic-bridge \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  ros-humble-joy \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rmw-cyclonedds-cpp
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

Source dependencies (clone into the same workspace):

```bash
cd ~/ros2_ws/src
git clone https://github.com/MohammadRobot/studica_drivers.git
git clone https://github.com/MohammadRobot/studica_ros2_control.git
git clone https://github.com/MohammadRobot/ydlidar_ros2_driver.git
git clone https://github.com/MohammadRobot/OrbbecSDK_ROS2.git
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver studica_drivers studica_ros2_control studica_vmxpi_ros2
source install/setup.bash
```

For LIDAR stability, build the LiDAR package exactly as:

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver --symlink-install
source ~/ros2_ws/install/setup.bash
```

For camera stability, build Orbbec packages in `Release` exactly as:

```bash
cd ~/ros2_ws/
colcon build --packages-select orbbec_camera orbbec_camera_msgs orbbec_description --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If you use Conda, make sure `colcon` resolves to system Python, or install missing Python ROS deps in the active Conda env (for example `catkin_pkg`).

## Student Quick Path (Simulation First)

If you are new to ROS 2, follow this section first.
The rest of this README has deeper details for customization and hardware.

1. Build and source:

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver studica_drivers studica_ros2_control studica_vmxpi_ros2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

2. Start simulation bringup:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim robot_profile:=class_4wd gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false
```
  - Low-Load Mode 
```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim robot_profile:=class_4wd gui:=true \
  gz_headless:=false \
  sim_camera_width:=320 sim_camera_height:=240 sim_camera_update_rate:=10.0 \
  sim_lidar_samples:=120 sim_lidar_update_rate:=10.0 sim_lidar_visualize:=false \
  sim_imu_update_rate:=50.0 \
  use_joystick:=true use_ground_truth_odom_tf:=false
```


3. Build a map (SLAM):

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  robot_profile:=class_4wd gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false \
  world:="${WORLD_SDF}"
```

4. Save the map:

```bash
mkdir -p "$HOME/ros2_ws/src/studica_vmxpi_ros2/maps"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map'}}"
```

5. Start navigation:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  robot_profile:=class_4wd gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false \
  world:="${WORLD_SDF}" \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

6. In RViz for navigation:
- `nav2_navigation_gz_sim.launch.py` uses `nav2_navigation.rviz` by default (`Fixed Frame: map`).
- Click `2D Pose Estimate` once to initialize AMCL.
- Then use `Nav2 Goal`.
- If `Map` looks empty right after startup, toggle the `Map` display once in RViz.

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

Unified bringup entry point (recommended):

```bash
# Gazebo Sim with default profile
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_2wd gui:=true use_joystick:=true

# Switch robot profile (example: 4WD profile)
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_4wd gui:=true use_joystick:=true

# Mecanum drive profile (holonomic)
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_mecanum gui:=true use_joystick:=true

# Omni-wheel profile (holonomic)
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_omni gui:=true use_joystick:=true
```

Profiles live under:

- `bringup/config/profiles/class_2wd/`
- `bringup/config/profiles/class_4wd/`
- `bringup/config/profiles/class_mecanum/`
- `bringup/config/profiles/class_omni/`
- `bringup/config/profiles/training_4wd/`
- `bringup/config/profiles/training_2wd/`

Each profile contains:

- `robot_profile.yaml` (URDF geometry + hardware mapping)
- `drive.*` section in `robot_profile.yaml` (`wheel_layout`, `controller_name`, `controller_type`)
- `robot_controllers.yaml` (controller tuning)

Wheel layout options:

- `diff` (2-wheel differential drive)
- `diff_4wd` (4-wheel differential drive)
- `mecanum`
- `omni` (X-drive wheel yaw in URDF)

Holonomic profile note:

- In ROS 2 Humble this package uses `mecanum_drive_controller` for both `wheel_layout: mecanum` and `wheel_layout: omni`.
- `wheel_layout: omni` uses an X-drive wheel mounting in URDF (45 deg wheel yaw at each corner).
- `class_4wd` uses `wheel_layout: diff_4wd` with `diff_drive_controller`.
- For `diff_4wd`, `mecanum`, and `omni` layouts, all four motor indices must be active (`>= 0`).

Profile template source files are provided in:

- `bringup/config/profile_template/`

Compatibility launch wrappers (`robot_gz_sim.launch.py`, `robot_gazebo_classic.launch.py`, `robot_bringup.launch.py`) are still available.

## Package Structure

The package is split so students can edit robot configs without changing launch internals or C++ code.

```text
studica_vmxpi_ros2/
|-- CMakeLists.txt
|-- package.xml
|-- README.md
|-- bringup/
|   |-- launch/
|   |   |-- bringup.launch.py
|   |   |-- robot_gz_sim.launch.py
|   |   |-- robot_gazebo_classic.launch.py
|   |   |-- robot_bringup.launch.py
|   |   |-- nav2_mapping*.launch.py
|   |   |-- nav2_navigation*.launch.py
|   |   |-- camera_hw.launch.py
|   |   `-- lidar_hw.launch.py
|   `-- config/
|       |-- profiles/
|       |   |-- class_2wd/
|       |   |-- class_4wd/
|       |   |-- class_mecanum/
|       |   |-- class_omni/
|       |   |-- training_2wd/
|       |   `-- training_4wd/
|       |-- profile_template/
|       |-- slam_toolbox_mapper_params.yaml
|       `-- ydlidar_x2_hw.yaml        # legacy optional config (default is ydlidar_ros2_driver/params/Tmini.yaml)
|-- description/
|   |-- urdf/
|   |-- robot/urdf/
|   |-- ros2_control/
|   |-- gz/ + gz/worlds/
|   |-- gazebo/ + gazebo/worlds/
|   `-- meshes/
|-- hardware/
|   |-- vmx_system.cpp                # Titan/VMX hardware interface
|   |-- sim_system.cpp                # Mock/sim hardware interface
|   `-- include/studica_vmxpi_ros2/
|-- src/
|   |-- topic_adapter_node.cpp        # Scan/IMU/Nav2 topic adapters
|   `-- patrol.cpp
|-- scripts/
|   |-- create_profile.sh
|   |-- validate_profiles.py
|   |-- check_project.sh
|   |-- install_git_hooks.sh
|   `-- motor_smoke_test.sh
`-- docs/
    |-- ROS2_TRAINING.md
    |-- PROFILE_AUTHORING.md
    `-- LAUNCH_MIGRATION.md
```

## How It Works

Runtime flow:

1. Start from `bringup.launch.py` with `mode:=<gz_sim|hardware|mock|gazebo_classic>` and `robot_profile:=<name>`.
2. The launch validates the selected profile (`robot_profile.yaml` + `robot_controllers.yaml`) before starting nodes.
3. URDF is generated from Xacro (`description/urdf/robot.urdf.xacro`) using profile values and controller config.
4. Mode-specific startup:
   - `gz_sim`: starts Gazebo Sim, spawns the robot, bridges `/clock`, `/scan`, and simulated camera topics (`/camera/color/*`, `/camera/depth/*`), and relays `/scan_raw -> /scan` with normalized frame id.
   - `hardware`: starts `ros2_control_node` with VMX/Titan hardware plugin (`vmx_system.cpp`), then controllers.
   - `mock`: starts `ros2_control_node` without real hardware for software-only testing.
   - `gazebo_classic`: routes through the Classic compatibility launch.
5. Common control layer runs `joint_state_broadcaster`, `imu_sensor_broadcaster`, and the selected drive controller from profile (`diff_drive_controller` or `mecanum_drive_controller`).
6. `topic_adapter_node` provides API compatibility:
   - IMU alias: `/imu_sensor_broadcaster/imu -> /imu` (with odom fallback in sim when needed).
   - Nav2 bridge (in Nav2 launches and hardware path): `/cmd_vel (Twist) -> /<drive_controller>/cmd_vel|reference (TwistStamped)` and odom aliasing.
7. Optional features are composed on top:
   - LiDAR launch in hardware mode (`use_lidar:=true`).
   - Camera launch in hardware mode (`use_camera:=true`).
   - Joystick teleop (`use_joystick:=true`).
   - Mapping and navigation launches include bringup and then add SLAM/Nav2.

How this supports different robot configurations:

- Robot differences are encoded in profiles under `bringup/config/profiles/<profile_name>/`.
- `robot_profile.yaml` controls geometry and hardware mapping (motors, inversions, scales, CAN, wheel params).
- `robot_controllers.yaml` controls controller types and tuning.
- Create a new profile, run `scripts/check_project.sh`, then launch with `robot_profile:=<new_profile>`.

## Launch Migration

Use `bringup.launch.py` as the single launch entry point for new labs and new robots.

| Legacy command | Unified command |
|---|---|
| `ros2 launch studica_vmxpi_ros2 robot_gz_sim.launch.py gui:=true use_gz_sim:=true` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim gui:=true` |
| `ros2 launch studica_vmxpi_ros2 robot_gz_sim.launch.py use_hardware:=true use_gz_sim:=false` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware` |
| `ros2 launch studica_vmxpi_ros2 robot_gazebo_classic.launch.py use_gazebo_classic:=true` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gazebo_classic` |
| `ros2 launch studica_vmxpi_ros2 robot_bringup.launch.py ...` | `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=<gz_sim|hardware|mock|gazebo_classic> ...` |

Deprecation policy:

- Since March 3, 2026, wrappers are compatibility paths only.
- New features are added only to `bringup.launch.py`.
- Wrapper removals are not planned before January 1, 2027.
- Detailed migration examples: `docs/LAUNCH_MIGRATION.md`.
- New profile checklist and schema guide: `docs/PROFILE_AUTHORING.md`.

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

Run the full local maintenance checks (syntax + profile validation):

```bash
scripts/check_project.sh
```

Enable local pre-commit validation:

```bash
scripts/install_git_hooks.sh
```

When relevant launch/profile files are staged, pre-commit runs `scripts/check_project.sh` automatically.
CI also validates profiles in `.github/workflows/profile-validation.yml`.

Simulation:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim gui:=true
```

If the robot appears partially below ground, spawn it slightly above zero:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim gui:=true spawn_z:=0.10
```

Simulation with joystick:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim gui:=true use_joystick:=true
```

This launch publishes joystick commands as `TwistStamped` on the active drive command topic:

- diff profiles: `/<drive_controller>/cmd_vel`
- mecanum/omni profiles: `/<drive_controller>/reference`

Simulation with office world:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim gui:=true use_joystick:=true \
  world:="${WORLD_SDF}"
```

Real hardware (current VMX setup requires root; see full setup in `VMX Real Robot Setup` below):

```bash
sudo -E bash -lc '
cd /home/vmx/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
'
```

In this real-hardware mode, LiDAR and Orbbec camera start automatically by default
(`use_lidar:=true` and `use_camera:=true` when `mode:=hardware`).
IMU is published by `imu_sensor_broadcaster` in the same `ros2_control` stack (single VMX hardware owner), and relayed to `/imu` for compatibility.

Disable LiDAR and camera auto-start if needed:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware use_lidar:=false use_camera:=false
```

Check IMU data (published from `ros2_control`):

```bash
ros2 topic echo /imu --qos-profile sensor_data --once
```

Use a custom YDLIDAR params file from the main bringup:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  ydlidar_params_file:=/path/to/ydlidar.yaml
```

Use custom Orbbec camera options from the main bringup:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  orbbec_launch_file:=gemini_e.launch.py \
  orbbec_camera_name:=camera \
  orbbec_serial_number:=<serial_number> \
  orbbec_enable_point_cloud:=true
```

## Known-Good Hardware Checklist

Use this sequence when setting up hardware to match the known-good LiDAR + camera workflow.

1. Build and source YDLIDAR driver:

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver --symlink-install
source ~/ros2_ws/install/setup.bash
```

2. Build and source Orbbec packages in Release:

```bash
cd ~/ros2_ws
colcon build --packages-select orbbec_camera orbbec_camera_msgs orbbec_description --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/ros2_ws/install/setup.bash
```

3. Optional standalone sensor checks:

```bash
ros2 launch ydlidar_ros2_driver ydlidar_tmini.launch.py
```

```bash
ros2 launch orbbec_camera gemini_e.launch.py
```

4. Launch integrated hardware stack:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
```

5. Quick verification:

```bash
ros2 topic list | grep -E "^/scan$|^/imu$|^/camera/color/image_raw$|^/camera/depth/image_raw$"
ros2 control list_controllers
```

IMU broadcaster settings are configured in the selected profile controller file
(`bringup/config/profiles/<profile>/robot_controllers.yaml`, `imu_sensor_broadcaster` block).

## Production Health Check

After launching hardware mode, run these checks:

```bash
ros2 topic list | grep -E "^/cmd_vel$|^/odom$|^/imu$|^/scan$"
```

Expected: public API topics are present (`/cmd_vel`, `/odom`, `/imu`, `/scan`).

If camera is enabled (`use_camera:=true`), verify camera topics:

```bash
ros2 topic list | grep -E "^/camera/color/image_raw$|^/camera/depth/image_raw$"
```

```bash
ros2 control list_controllers
```

Expected: `joint_state_broadcaster`, `imu_sensor_broadcaster`, and the selected drive controller are `active`
(for example: `robot_base_controller`, `mecanum_base_controller`, or `omni_base_controller`).

```bash
ros2 topic hz /imu
```

Expected: stable IMU publish rate (non-zero, continuous output).

```bash
ros2 topic echo /imu --qos-profile sensor_data --once
```

Expected: one `sensor_msgs/msg/Imu` message with orientation, angular velocity, and linear acceleration fields.

```bash
ros2 topic echo /odom --once
```

Expected: one `nav_msgs/msg/Odometry` message on `/odom` (aliased from the selected drive controller odom topic).

## VMX Real Robot Setup

With ROS environment values in `~/.bashrc`, source your shell config plus workspace/runtime before launch:

```bash
cd /home/vmx/ros2_ws
source /home/vmx/.bashrc
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
```

Recommended `~/.bashrc` setup (both robot host and PC host):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Optional robot-host additions in `~/.bashrc`:

```bash
source /home/vmx/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
```

After updating `.bashrc`, run `source ~/.bashrc` once (or open a new terminal).
From then on, you do not need to re-run the ROS export lines in every terminal.

Known-good hardware bringup (single command, runs as root):

```bash
sudo -E bash -lc '
cd /home/vmx/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
'
```

Why `sudo` is required for hardware mode:

- `mode:=hardware` loads VMX/Titan hardware drivers (pigpio + SPI/CAN access).
- These low-level interfaces require root privileges on the current VMX HAL setup.
- Simulation modes (`mode:=gz_sim` or `mode:=gazebo_classic`) do not require `sudo`.

If you must run as `root`, add equivalent ROS environment to `/root/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Optional robot-host additions for `/root/.bashrc`:

```bash
source /home/vmx/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/vmxpi
```

## Optional Bringup

Legacy combined bringup (not recommended for production single-runtime mode):

```bash
ros2 launch studica_vmxpi_ros2 robot_bringup.launch.py use_studica_sensors:=true
```

Use this only for compatibility testing.
Production hardware mode should use `bringup.launch.py mode:=hardware`, where IMU is already inside `ros2_control`.

## LiDAR (YDLIDAR)

Recommended approach: keep `ydlidar_ros2_driver` as the LiDAR hardware driver, and keep `studica_ros2_control` for integration/bringup.

Main hardware launch auto-starts LiDAR:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
```

Standalone driver test (known-good command):

```bash
ros2 launch ydlidar_ros2_driver ydlidar_tmini.launch.py
```

Known-good standalone LiDAR flow:

```bash
cd ~/ros2_ws
colcon build --packages-select ydlidar_ros2_driver --symlink-install
source ~/ros2_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_tmini.launch.py
```

Integrated launch from Studica stack:

```bash
ros2 launch studica_ros2_control lidar_launch.py
```

Use a custom YDLIDAR parameter file when needed:

```bash
ros2 launch studica_ros2_control lidar_launch.py ydlidar_params_file:=/path/to/ydlidar.yaml
```

Show LiDAR in RViz2:

```bash
# Terminal 1 (VMX)
ros2 launch studica_ros2_control lidar_launch.py
```

```bash
# Terminal 2 (Remote PC)
rviz2
```

In RViz2:

- Set `Fixed Frame` to `laser_frame` (or `base_link`).
- Add display type `LaserScan`.
- Set topic to `/scan`.

One-command RViz launch from driver package (optional):

```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch_view.py
```

## Camera (OrbbecSDK_ROS2)

Recommended approach: keep `orbbec_camera` as the camera hardware driver, and let
`studica_vmxpi_ros2` own system-level bringup and TF integration.

Main hardware launch auto-starts camera:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=hardware
```

Standalone driver test (known-good command):

```bash
ros2 launch orbbec_camera gemini_e.launch.py
```

Use custom camera launch selection and device serial when needed:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  orbbec_launch_file:=gemini_e.launch.py \
  orbbec_camera_name:=camera \
  orbbec_serial_number:=<serial_number> \
  orbbec_enable_point_cloud:=true
```

Known-good standalone camera bringup flow:

```bash
# Build (Release)
cd ~/ros2_ws/
colcon build --packages-select orbbec_camera orbbec_camera_msgs orbbec_description --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
# Terminal 1
source install/setup.bash
ros2 launch orbbec_camera gemini_e.launch.py
```

```bash
# Terminal 2
. ./install/setup.bash
rviz2
```

If you switch away from `gemini_e.launch.py` and need an extra camera TF from this package, enable:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware \
  publish_camera_tf:=true
```

## Gazebo Sim Notes

- Unified Gazebo Sim launch: `bringup.launch.py mode:=gz_sim`
- Gazebo Sim Nav2 launches:
  - `nav2_mapping_gz_sim.launch.py`
  - `nav2_navigation_gz_sim.launch.py` (simulation only)
- Real robot Nav2 launch: `nav2_navigation_hw.launch.py`
- Gazebo Sim sensor topics:
  - `/scan` (`sensor_msgs/LaserScan`)
  - `/imu` (`sensor_msgs/Imu`)
  - `/camera/color/image_raw` (`sensor_msgs/Image`)
  - `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)
  - `/camera/depth/image_raw` (`sensor_msgs/Image`)
  - `/camera/depth/camera_info` (`sensor_msgs/CameraInfo`)
- Gazebo Classic compatibility launches are still available (`robot_gazebo_classic.launch.py`, `nav2_mapping.launch.py`, `nav2_navigation.launch.py`).

## System Architecture (Best Performance)

Recommended deployment is split across two machines:

- VMX robot host: run hardware-critical nodes only (VMX/Titan drivers, ros2_control, motor controllers, hardware sensor drivers like LiDAR and depth cameras).
- Remote PC host: run operator and high-CPU nodes (RViz2, teleop, Nav2 tools, SLAM tools, debugging/visualization tools).

Why this split works better:

- Motor/sensor timing stays local to hardware on the robot.
- VMX CPU load is lower, so control loops are more stable.
- UI and planning workloads run on a stronger PC without impacting robot control.

Recommended runtime pattern:

- On robot host: launch hardware stack and sensor drivers.
- On PC host: launch remote control, RViz2, and any heavy analysis nodes.
- Use the same ROS 2 networking settings on both hosts (`ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY`, `RMW_IMPLEMENTATION`).

Node placement reference:

| Component / Node Group | Run Host | Why |
|---|---|---|
| `studica_drivers` + VMX/Titan hardware interfaces | Robot (VMX) | Direct hardware access (SPI/CAN/GPIO), lowest control latency. |
| `controller_manager` + controllers (`diff_drive_controller`, `imu_sensor_broadcaster`) | Robot (VMX) | Keeps motor + IMU control loop local and stable. |
| `studica_ros2_control` teleop/utilities (optional) | Robot or Remote PC | Use for optional tools (for example joystick helper), not primary hardware runtime. |
| `ydlidar_ros2_driver` | Robot (VMX) | Serial LiDAR data capture should stay local to `/dev/ttyUSB*`. |
| `orbbec_camera` | Robot (VMX) | USB camera capture should stay local to robot USB bus and sensor power domain. |
| TF publishers tied to physical sensors | Robot (VMX) | Keeps robot frame tree synchronized with real sensors. |
| Teleop nodes (`teleop_twist_keyboard`, gamepad client) | Remote PC | Operator input/UI runs offboard to reduce VMX load. |
| `rviz2` | Remote PC | Visualization is GPU/CPU heavy and not control-critical. |
| Nav2 planning/BT tools (recommended) | Remote PC | Planning is CPU-heavy; offloading improves VMX performance. |
| SLAM (`slam_toolbox`) (recommended) | Remote PC | Mapping can be heavy; offloading preserves control responsiveness. |
| Debug/record tools (`rqt`, `ros2 bag`, diagnostics UI) | Remote PC | Prevents debug workloads from stealing robot compute. |


## Control (From Another PC)

This section is for teleop from a second PC on the same network, while the robot stack runs on the robot host.
Use the command topic that matches the selected robot profile:

- `training_2wd`, `training_4wd`, `class_2wd`, `class_4wd`: `/robot_base_controller/cmd_vel`
- `class_mecanum`: `/mecanum_base_controller/reference`
- `class_omni` (X-drive): `/omni_base_controller/reference`

On the remote PC, open a terminal and load your ROS networking setup from `~/.bashrc`:

```bash
source ~/.bashrc
```

Optional connectivity check:

```bash
ros2 topic list | grep -E "cmd_vel|reference|odom|odometry|base_controller"
```

Keyboard teleop (remote PC -> robot):

```bash
CMD_TOPIC=/robot_base_controller/cmd_vel
# CMD_TOPIC=/mecanum_base_controller/reference
# CMD_TOPIC=/omni_base_controller/reference
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p frame_id:=base_link --remap cmd_vel:=${CMD_TOPIC}
```

Joystick teleop (remote PC -> robot):

```bash
# Required for gamepad_launch.py on the remote PC:
source ~/ros2_ws/install/setup.bash
CMD_TOPIC=/robot_base_controller/cmd_vel
# CMD_TOPIC=/mecanum_base_controller/reference
# CMD_TOPIC=/omni_base_controller/reference
ros2 launch studica_ros2_control gamepad_launch.py \
  cmd_vel_topic:=${CMD_TOPIC} \
  publish_stamped:=true
```

If the real robot is too fast, use lower joystick scales:

```bash
# Required for gamepad_launch.py on the remote PC:
source ~/ros2_ws/install/setup.bash
CMD_TOPIC=/robot_base_controller/cmd_vel
# CMD_TOPIC=/mecanum_base_controller/reference
# CMD_TOPIC=/omni_base_controller/reference
ros2 launch studica_ros2_control gamepad_launch.py \
  cmd_vel_topic:=${CMD_TOPIC} \
  publish_stamped:=true \
  linear_scale:=0.20 \
  angular_scale:=0.60 \
  turbo_multiplier:=1.0 \
  button_turbo:=-1
```

Joystick tuning parameters:

- `linear_scale` (m/s at full stick)
- `angular_scale` (rad/s at full stick)
- `deadzone`
- `turbo_multiplier`
- `button_turbo` (`-1` disables turbo)

Notes:

- When launching `bringup.launch.py mode:=gz_sim` or `nav2_*_gz_sim.launch.py`, keep `use_joystick:=false` (default) if you drive from another PC, or you may have two joystick publishers.
- `robot_bringup.launch.py` does not include a `use_joystick` argument.

## Mapping (SLAM Toolbox)

Launch mapping:

```bash
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false
```

Launch mapping with a specific world:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 nav2_mapping_gz_sim.launch.py \
  robot_profile:=class_4wd gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false \
  world:="${WORLD_SDF}"
```

Save map:

```bash
mkdir -p "$HOME/ros2_ws/src/studica_vmxpi_ros2/maps"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map'}}"
```

## Navigation (Nav2 + AMCL)

Launch navigation in simulation with a saved map:

```bash
WORLD_SDF="$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf"
ros2 launch studica_vmxpi_ros2 nav2_navigation_gz_sim.launch.py \
  robot_profile:=class_4wd gui:=true use_joystick:=true \
  use_ground_truth_odom_tf:=false \
  world:="${WORLD_SDF}" \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

`nav2_navigation_gz_sim.launch.py` defaults to
`description/robot/rviz/nav2_navigation.rviz` (Fixed Frame: `map`).
Use `rviz_config_file:=/absolute/path/to/file.rviz` to override.

Launch navigation on the real robot with a saved map:

```bash
ros2 launch studica_vmxpi_ros2 nav2_navigation_hw.launch.py \
  gui:=true \
  map:="$HOME/ros2_ws/src/studica_vmxpi_ros2/maps/my_map.yaml"
```

After launch in RViz:

1. Set fixed frame to `map` (if it is not already `map`).
2. Click `2D Pose Estimate` once to initialize AMCL.
3. Use `Nav2 Goal` to send goals.

`topic_adapter_node` is started automatically for Nav2 launches with `enable_nav2_bridge:=true`:

- `/cmd_vel` (`Twist`) -> selected drive command topic (`/<drive_controller>/cmd_vel` or `/<drive_controller>/reference`)
- selected drive odom topic (`/<drive_controller>/odom` or `/<drive_controller>/odometry`) -> `/odom`

## Raspberry Pi 4 Performance Tuning (VMX)

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

5. Use Cyclone DDS consistently in every shell (normal user and `sudo`):

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
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
journalctl -k --no-pager | grep -Ei "under-?voltage|throttl|oom|thermal"
vcgencmd get_throttled   # expected healthy value: throttled=0x0
```

## Motor Smoke Test (Drivers + ROS 2 Control + ros2_control)

Use the repeatable smoke test script to validate motor control across:

- `studica_drivers` (direct Titan API)
- `studica_ros2_control` (`manual_composition` + `titan_cmd`)
- `studica_vmxpi_ros2` (`ros2_control` + `robot_base_controller`)

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

PC becomes very slow when running `mode:=gz_sim`:

- Make sure only one Gazebo/bringup session is running:
  `pkill -f "ros2 launch studica_vmxpi_ros2 bringup.launch.py"; pkill -f "gz sim"`
- If logs show `RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed`, clear stale FastDDS locks:
  `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`
- Use low-load sim settings (headless Gazebo + reduced sensor rates):
  `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_4wd gui:=true gz_headless:=true sim_camera_width:=320 sim_camera_height:=240 sim_camera_update_rate:=10.0 sim_lidar_samples:=120 sim_lidar_update_rate:=10.0 sim_lidar_visualize:=false sim_imu_update_rate:=50.0`
- If you still need Gazebo GUI, keep `gz_headless:=false` but keep reduced sensor settings.

Teleop command runs but robot does not move:

- If launch is running as `root` and teleop runs as normal user, local DDS discovery may work but topic/service data can fail between users.
- Avoid `sudo su` for launch. Prefer `sudo -E` so ROS environment is preserved.
- Short-term workaround: run teleop as `sudo -E` (same user context as launch command).
- Long-term fix: run bringup without `sudo` by granting required VMX/SPI/CAN permissions to the `vmx` user.

`No transform from [front_left_wheel] to [odom]` in RViz:

- This is usually a startup timing issue while controllers/TF are still activating.
- Wait a few seconds after launch, or increase `rviz_start_delay`:
  `ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_4wd rviz_start_delay:=14.0 gui:=true`
- Verify controller spawner logs include:
  `Configured and activated robot_base_controller` or
  `spawner-fallback controllers active on /controller_manager`.
- If needed, stop stale sim processes and relaunch cleanly:
  `pkill -f "gz sim"; pkill -f "ros2 launch studica_vmxpi_ros2 bringup.launch.py"`
- If RViz shows repeated `TF_OLD_DATA` warnings, ensure only one active ROS/Gazebo session per DDS domain.
  Quick isolation test: launch in a different domain:
  `ROS_DOMAIN_ID=66 ros2 launch studica_vmxpi_ros2 bringup.launch.py mode:=gz_sim robot_profile:=class_4wd gui:=true`

Robot appears in Gazebo Sim, joystick is detected, but robot does not move:

- Confirm joystick topic has non-zero values: `ros2 topic echo /joy --once`
- Confirm controller input topic has both publisher and subscriber:
  `ros2 topic info /robot_base_controller/cmd_vel`
- Confirm command is accepted by controller:
  `ros2 topic echo /robot_base_controller/cmd_vel_out --qos-durability transient_local --once`
- If using a custom joystick launcher in sim, ensure stamped commands use a valid timestamp
  (for example gamepad node `use_sim_time:=false` or non-zero header stamp).

`librmw_cyclonedds_cpp.so` not found:

- Install Cyclone DDS middleware: `sudo apt install ros-humble-rmw-cyclonedds-cpp`
- Ensure both shells use the same RMW: `echo $RMW_IMPLEMENTATION` and `sudo -E bash -lc 'echo $RMW_IMPLEMENTATION'`

`robot_base_controller` fails with `expected [double] got [integer]`:

Set these values as floats in the active profile controller file
(`bringup/config/profiles/<profile>/robot_controllers.yaml`):

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

`/scan` is empty and bridge logs `Unknown message type [8]` / `[9]`:

- Cause: mixed Gazebo bridge packages (legacy `ros-humble-ros-gz-*` using ignition transport with Harmonic `gz sim` runtime).
- Fix: use only Harmonic bridge packages (`ros-humble-ros-gzharmonic-*`) and remove conflicting `ros-humble-ros-gz-*` bridge packages.
- Verify: `ldd /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge | grep transport` should show `libgz-transport13`, not `libignition-transport11`.

Gazebo Sim launch fails with `libgazebo_ros2_control.so` / `libgazebo_ros_*` plugin errors:

- You launched a Gazebo Classic `.world` file in `gz sim`.
- Use a Gazebo Sim `.sdf` world, for example:
- `$(ros2 pkg prefix studica_vmxpi_ros2)/share/studica_vmxpi_ros2/description/gz/worlds/office_map.sdf`

Robot drives too slowly in simulation:

- Check wheel joint velocity limits in `description/robot/urdf/robot_description.urdf.xacro`.
- Low values (for example `velocity="1.0"`) strongly cap top speed.

IMU topic exists but no visible data in echo:

- Use sensor QoS for echo:
- Hardware mode (aliased from `imu_sensor_broadcaster`): `ros2 topic echo /imu --qos-profile sensor_data`
- Gazebo Sim mode (relayed from `imu_sensor_broadcaster`): `ros2 topic echo /imu --qos-profile sensor_data`

Odometry is noisy / not smooth enough:

- Increase controller update/publish rates and velocity rolling window in the active profile controller file
  (`bringup/config/profiles/<profile>/robot_controllers.yaml`).
- Current defaults are tuned for smoother sim odometry:
  - `controller_manager.update_rate: 100`
  - `robot_base_controller.publish_rate: 100.0`
  - `robot_base_controller.velocity_rolling_window_size: 30`

## License

This project is licensed under the Apache License, Version 2.0.

- Full license text: `LICENSE`
- Project notices/attribution: `NOTICE`
- ROS package metadata: `package.xml` (`<license>Apache-2.0</license>`)
