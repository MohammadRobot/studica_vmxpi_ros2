# Launch Arguments

ROS 2 launch arguments use `name:=value` after the launch-file name:

```bash
ros2 launch <package> <file.launch.py> name:=value
```

Arguments affect only that launch. They are not shell environment variables or
ROS parameters unless the launch file explicitly forwards them.

Display the installed arguments at any time:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py --show-args
```

Source the selected simulation or robot network environment, ROS 2, and the
workspace before using any command on this page.

## Simulation

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py [arguments]
```

| Argument | Default | Purpose |
|---|---|---|
| `gui` | `true` | Start RViz |
| `gz_headless` | `false` | Run Gazebo without its graphical client |
| `use_camera` | `false` | Enable simulated RGB and depth streams |
| `use_point_cloud` | `false` | Enable the camera and publish raw plus floor-filtered `PointCloud2` data |
| `use_joystick` | `true` | Start deadman-protected DualShock teleoperation |
| `world` | `maze` | Select a bundled world or an absolute SDF path |

Camera and RViz DepthCloud test:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py \
  use_camera:=true use_joystick:=false
```

The simulated DepthCloud display combines `/camera/color/image_raw`,
`/camera/depth/image_raw`, and `/camera/depth/camera_info` inside RViz.

Create an actual point-cloud topic for application code and RViz:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py \
  use_point_cloud:=true use_joystick:=false
```

This automatically enables the depth camera and publishes raw optical-frame
points on `/camera/depth/points` plus `base_link` obstacle points on
`/camera/depth/points_filtered`, both as `sensor_msgs/msg/PointCloud2`. See
[Camera and point cloud](CAMERA_POINT_CLOUD.md).

Headless automated run:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py \
  gui:=false gz_headless:=true use_joystick:=false
```

Supported short world names are `maze`, `diff_drive`, and `office_map`.

## Mapping

```bash
ros2 launch studica_vmxpi_ros2 mapping.launch.py [arguments]
```

| Argument | Default | Purpose |
|---|---|---|
| `mode` | `gz_sim` | Use local `gz_sim` or the PC client for separate `hardware` bringup |
| `gui` | `true` | Start RViz |
| `gz_headless` | `false` | Run Gazebo without its graphical client |
| `use_joystick` | `true` | Start deadman-protected DualShock teleoperation |
| `joystick_config_file` | project DualShock YAML | Override joystick parameters |
| `rviz_config_file` | project `robot.rviz` | Override the RViz configuration |
| `slam_params_file` | selected by `mode` | Override the SLAM Toolbox parameter file |

With the default `mode:=gz_sim`, the mapping launch fixes the office world and
keeps the camera disabled to preserve mapping performance. With
`mode:=hardware`, run it on the PC after `robot.launch.py` is running on the
VMXPi. It starts only PC-side SLAM, RViz, and deadman joystick nodes; it never
starts local controllers, Gazebo, or physical sensor drivers.

```bash
ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=hardware
```

Hardware mode automatically uses the X2-tuned
`slam_toolbox_hardware_mapper_params.yaml` and limits normal joystick motion to
0.08 m/s and 0.25 rad/s. L1 is still required; the optional R1 turbo is limited
to 0.12 m/s and 0.40 rad/s. The physical profile places scans with calibrated
wheel/IMU odometry, disables continuous scan matching, and permits only strict
high-confidence loop closure to correct return-to-start drift. Simulation retains
`slam_toolbox_mapper_params.yaml`.

## Navigation

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py [arguments]
```

| Argument | Default | Purpose |
|---|---|---|
| `mode` | `gz_sim` | Use the local office simulation or the PC-side `hardware` client |
| `robot_profile` | `class_4wd` in simulation; `stack_4wd` in hardware | Geometry used by simulation and the Nav2 footprint |
| `gui` | `true` | Start RViz |
| `gz_headless` | `false` | Run Gazebo without its graphical client |
| `map` | office map in simulation; `~/studica_ws/project_maps/real_robot_map.yaml` in hardware | Map YAML loaded by the map server and AMCL |
| `nav2_params_file` | Nav2 Humble default YAML | Override the complete Nav2 parameter file |
| `autostart` | `true` | Automatically configure and activate lifecycle nodes |
| `hardware_max_linear_speed` | `0.20` | Physical Nav2 limit in m/s; supported range is greater than 0 through 0.30 |
| `hardware_max_angular_speed` | `0.35` | Physical Nav2 limit in rad/s; supported range is greater than 0 through 0.60 |
| `use_point_cloud` | `true` in simulation; `false` in hardware | Mark local obstacles with filtered depth points and clear stale costs with raw depth rays |
| `use_joystick` | `false` | Start simulation joystick teleop; ignored in hardware mode and keep false while Nav2 owns motion |
| `rviz_config_file` | project Nav2 RViz file | RViz configuration used by hardware mode |

Use the generated project map:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  map:="$STUDICA_WS/project_maps/office_project.yaml"
```

After launch, set **2D Pose Estimate** in RViz so AMCL can publish
`map -> odom`. Do this before sending a Nav2 goal.

For the physical robot, keep `robot.launch.py` running on the VMXPi and run on
the PC:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=hardware
```

Hardware mode starts no simulator, hardware driver, or joystick. It applies
the measured footprint, AMCL's X2 LiDAR range, and bounded configurable Nav2
motion limits. Stop the mapping launch first so SLAM and teleop do not compete
with AMCL and Nav2.

The camera overlay is applied to the local obstacle/voxel layer of the selected
complete `nav2_params_file`; all other custom parameters remain intact. Use
`use_point_cloud:=false` on a resource-constrained computer or for a LiDAR-only
comparison.

## Physical robot

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py [arguments]
```

| Argument | Default | Purpose |
|---|---|---|
| `robot_profile` | `stack_4wd` | Physical profile below `bringup/config/profiles` |
| `use_lidar` | `true` | Start the YDLidar driver |
| `use_camera` | `true` | Start the Orbbec low-load depth stream |
| `use_camera_color` | `false` | Also start 640×480 color at 15 Hz |
| `use_colored_depth_cloud` | `false` | Start registered 320×240 color and depth at 15 Hz for RViz DepthCloud |
| `use_point_cloud` | `false` | Publish raw 320×240 depth points at 5 Hz |
| `use_point_cloud_filter` | `false` | Also publish the floor/body-filtered cloud when raw points are enabled |
| `use_foxglove` | `true` | Start the read-only Foxglove bridge |
| `use_joystick` | `false` | Start joystick teleop on the VMXPi |
| `hardware_control_rate_hz` | `25` | Set the hardware control and odometry rate |
| `use_imu_odometry` | `true` | Fuse encoder forward velocity with IMU yaw and own odometry TF |
| `foxglove_address` | `127.0.0.1` | Bind address for Foxglove Bridge |
| `foxglove_port` | `8765` | Foxglove WebSocket port |

Hardware launch requires the supervised procedure, root HAL permissions, a
clear work area, and an emergency stop. Follow [Supervised hardware](HARDWARE.md)
instead of running this table as an unsupervised checklist.

The physical point-cloud profile is deliberately depth-only and low-rate. Run
this launch through the preserved-env root wrapper in
[Supervised hardware](HARDWARE.md#4-start-the-vmx-hal-correctly):

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  use_point_cloud:=true use_camera_color:=false \
  use_foxglove:=false use_joystick:=false
```

This publishes raw Gemini E points only. Add
`use_point_cloud_filter:=true` when the filtered obstacle cloud is also
required.

Do not combine color, point-cloud processing, and remote visualization until a
sustained thermal test passes. The point-cloud filter samples every second raw
point on hardware; this is independent of the camera's 320×240 output mode.

## Application observer

```bash
ros2 launch studica_robot_apps robot_observer.launch.py [arguments]
```

| Argument | Default | Purpose |
|---|---|---|
| `use_sim_time` | `false` | Use Gazebo `/clock` when true |

Use `use_sim_time:=true` with simulation and `false` on the VMXPi.

The point-cloud observer uses the same launch argument:

```bash
ros2 launch studica_robot_apps point_cloud_observer.launch.py \
  use_sim_time:=true
```

Its `point_cloud_topic` launch argument defaults to
`/camera/depth/points_filtered`.

| Argument | Default | Purpose |
|---|---|---|
| `use_sim_time` | `false` | Use Gazebo `/clock` when true |
| `point_cloud_topic` | `/camera/depth/points_filtered` | Select the `PointCloud2` input |

## Advanced bringup

`bringup.launch.py` exposes platform-level configuration. Prefer the smaller
launch files above unless an application needs a setting they do not expose.

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py [arguments]
```

### Runtime and Gazebo

| Argument | Default | Purpose |
|---|---|---|
| `mode` | `gz_sim` | Select `gz_sim`, `hardware`, or `mock` |
| `gui` | `false` | Start RViz |
| `rviz_config_file` | project `robot.rviz` | Absolute RViz configuration path |
| `robot_profile` | `class_4wd` | Profile below `bringup/config/profiles` |
| `use_sim_time` | empty/automatic | Override the clock selection |
| `use_ground_truth_odom_tf` | `false` | Use Gazebo ground-truth `/odom` and `/tf` |
| `hardware_control_rate_hz` | `25` | Hardware-only `ros2_control` and odometry rate |
| `use_imu_odometry` | automatic | Enable encoder-`vx` + IMU-yaw EKF only in hardware mode |
| `world` | mode default | Short world name or absolute SDF path |
| `world_name` | `default` | Gazebo world name used by service calls |
| `gz_headless` | `false` | Run only the Gazebo server |
| `spawn_x`, `spawn_y`, `spawn_z` | world default | Robot spawn position in metres |
| `spawn_yaw` | world default | Robot spawn heading in radians |
| `spawn_entity_name` | `robot_system_position` | Gazebo entity name |

### Simulated sensors

| Argument | Default | Purpose |
|---|---|---|
| `sim_enable_camera` | `true` | Enable RGB and depth sensors |
| `sim_camera_width` | `640` | Image width in pixels |
| `sim_camera_height` | `480` | Image height in pixels |
| `sim_camera_update_rate` | `30.0` | Requested RGB/depth rate in Hz |
| `sim_lidar_samples` | `200` | Horizontal scan sample count |
| `sim_lidar_update_rate` | `20.0` | Requested scan rate in Hz |
| `sim_lidar_visualize` | `true` | Show LiDAR rays in Gazebo |
| `sim_imu_update_rate` | `100.0` | Requested IMU rate in Hz |

Actual rates depend on GPU and CPU load. A lower-resolution camera example is:

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=gz_sim gui:=true world:=maze \
  sim_enable_camera:=true sim_camera_width:=320 \
  sim_camera_height:=240 sim_camera_update_rate:=15.0 \
  use_joystick:=false
```

### Optional components and remote visualization

| Argument | Default | Purpose |
|---|---|---|
| `use_lidar` | automatic | Enable hardware LiDAR |
| `lidar_type` | profile value | Select the YDLidar hardware preset |
| `ydlidar_params_file` | empty | Override the YDLidar YAML |
| `use_camera` | automatic | Enable the hardware camera |
| `use_monitoring` | automatic | Enable robot health monitoring |
| `use_foxglove` | automatic | Enable the read-only Foxglove bridge |
| `use_joystick` | `false` | Enable deadman-protected joystick teleop |
| `joystick_config_file` | project DualShock YAML | Override joystick parameters |
| `foxglove_address` | `127.0.0.1` | Foxglove bind address |
| `foxglove_port` | `8765` | Foxglove WebSocket port |

An empty automatic value enables hardware-only components in hardware mode and
keeps them off in simulation or mock mode.

`hardware_control_rate_hz` is applied only when `mode:=hardware`. The launch
creates a temporary controller parameter file with that rate; simulation and
mock mode continue to use the profile's checked-in 100 Hz values. Keep the
25 Hz hardware default was validated on the VMXPi with all eight guarded motor
directions passing, controller recovery confirmed, and no active throttling.
Override it only after repeating CPU, thermal, and motor-control validation.

`use_imu_odometry` is hardware-only. When enabled, controller odometry TF is
disabled, raw encoder odometry is published as `/wheel/odom`, and the EKF is
the sole `/odom` plus `odom -> base_footprint` owner. Setting it to `false`
restores the direct controller odometry path for comparison or diagnosis.

### Orbbec hardware camera

These arguments affect hardware mode only.

| Argument | Default | Purpose |
|---|---|---|
| `orbbec_launch_file` | `gemini_e.launch.py` | Orbbec launch file |
| `orbbec_camera_name` | `camera` | Camera namespace/name |
| `orbbec_serial_number` | empty | Select one physical camera |
| `orbbec_enable_point_cloud` | `false` | Publish the Orbbec point cloud |
| `orbbec_enable_color` | `false` | Enable color streaming |
| `orbbec_depth_registration` | `false` | Align depth pixels to the color image |
| `orbbec_enable_depth` | `true` | Enable depth streaming |
| `orbbec_enable_ir` | `false` | Enable infrared streaming |
| `orbbec_color_width`, `orbbec_color_height` | `640`, `480` | Color resolution when enabled |
| `orbbec_color_fps` | `15` | Color frame rate when enabled |
| `orbbec_depth_width`, `orbbec_depth_height` | `320`, `240` | Low-load depth resolution |
| `orbbec_depth_fps` | `5` | Low-load depth frame rate |

Point-cloud output increases VMXPi compute, memory, USB, and network load. Enable
it only after the base hardware health check passes. Explicit launch arguments
can override this profile, but `640×480@15` point-cloud testing reached 82.3 °C
and activated the platform's soft temperature limit on the measured VMXPi.

### Optional camera TF override

The robot description already owns the normal camera transform. Use this group
only for a measured hardware installation that lacks the required driver TF.
Publishing a duplicate transform is an error.

| Argument | Default | Purpose |
|---|---|---|
| `publish_camera_tf` | `false` | Publish an additional static camera transform |
| `camera_parent_frame` | `base_link` | Parent frame |
| `camera_child_frame` | `<camera_name>_link` | Child frame when left empty |
| `camera_tf_x`, `camera_tf_y`, `camera_tf_z` | `0.0` | Translation in metres |
| `camera_tf_qx`, `camera_tf_qy`, `camera_tf_qz` | `0.0` | Quaternion vector components |
| `camera_tf_qw` | `1.0` | Quaternion scalar component |

## Argument troubleshooting

- `unrecognized argument` means that launch file does not expose the name; run
  `--show-args` on the exact file.
- Boolean values should be `true` or `false`.
- Quote paths containing spaces and use absolute paths for external YAML/SDF.
- Source the workspace again after rebuilding a copied install.
- Do not enable joystick and Nav2 or another `/cmd_vel` publisher together
  without an explicit command multiplexer.
