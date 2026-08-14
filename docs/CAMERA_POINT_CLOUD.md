# Camera and Point Cloud

The simulated RGB-D camera can publish color images, depth images, calibration,
and a sampled XYZ point cloud. Camera processing is optional in the general
simulation launch and enabled by default for simulation navigation. Physical
navigation defaults to LiDAR-only operation to reduce VMXPi and network load.

## Build

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
cd "$STUDICA_WS"
colcon build --symlink-install --packages-select \
  studica_vmxpi_ros2 studica_robot_apps
source "$STUDICA_WS/install/setup.bash"
```

## Start the simulated point cloud

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py \
  use_point_cloud:=true use_joystick:=false
```

`use_point_cloud:=true` automatically enables the depth camera. The converter
samples every fourth pixel, then a second node transforms the cloud into
`base_link` and removes floor, robot-body, and out-of-range returns:

```text
depth image -> raw optical-frame cloud -> base_link filter -> obstacle cloud
```

| Topic | Type | Meaning |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB image |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth in metres (`32FC1`) |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Projection calibration |
| `/camera/depth/points` | `sensor_msgs/msg/PointCloud2` | Raw sampled XYZ points in the optical frame |
| `/camera/depth/points_filtered` | `sensor_msgs/msg/PointCloud2` | Filtered obstacle points in `base_link` |

RViz enables `Raw PointCloud` and disables `Filtered Obstacles` and
`DepthCloud` by default. This keeps the standard camera view on
`/camera/depth/points` only; the other displays remain available as explicit
opt-ins.

## Verify the data

In a second terminal with the same environment and workspace sourced:

```bash
ros2 topic info /camera/depth/points --verbose
ros2 topic hz /camera/depth/points
ros2 topic echo /camera/depth/points --once --field width
```

The cloud uses the camera optical frame: positive `x` points right in the
image, positive `y` points down, and positive `z` points forward from the
camera. The filtered cloud uses `base_link`, where positive `x` is forward,
positive `y` is left, and positive `z` is upward from the ground plane.

## Colored RViz DepthCloud on hardware

RViz `DepthCloud` combines color and depth images on the PC; it is separate from
the raw and filtered `PointCloud2` topics. Start the physical camera with matched
registered streams:

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  use_colored_depth_cloud:=true use_point_cloud:=false \
  use_foxglove:=false use_joystick:=false
```

This selects 320x240 at 15 Hz for both streams and enables Orbbec depth
registration. In RViz, configure `DepthCloud` with
`/camera/depth/image_raw`, `/camera/depth/camera_info`, and
`/camera/color/image_raw`. Do not enable this visualization merely to obtain
ground-filtered obstacles; use `/camera/depth/points_filtered` for that purpose.

The default filter keeps heights from 0.04 m to 1.50 m, forward distances from
0.15 m to 3.00 m, and lateral distances within 2.00 m. It also removes a box
around the `stack_4wd` body. These ROS parameters are intentionally configurable
on `point_cloud_filter`:

| Parameter | Default | Purpose |
|---|---|---|
| `target_frame` | `base_link` | Ground-referenced output frame |
| `stride` | `1` | Keep every Nth input point before transform/filter work |
| `min_height_m`, `max_height_m` | `0.04`, `1.50` | Remove floor and returns above the useful obstacle height |
| `min_forward_m`, `max_forward_m` | `0.15`, `3.00` | Forward crop |
| `max_lateral_m` | `2.00` | Symmetric lateral crop |
| `robot_min_x_m`, `robot_max_x_m` | `-0.20`, `0.25` | Robot-body exclusion in x |
| `robot_half_width_m` | `0.21` | Robot-body exclusion half-width |
| `robot_max_height_m` | `0.35` | Robot-body exclusion height |

## Run the point-cloud observer

The application observer parses the standard message without commanding
motion. Start it in Terminal 2 and leave it running:

```bash
ros2 launch studica_robot_apps point_cloud_observer.launch.py \
  use_sim_time:=true
```

Read its output from Terminal 3:

```bash
ros2 topic echo /apps/point_cloud_summary --once
```

The observer defaults to `/camera/depth/points_filtered`. A clear scene may
report zero obstacle points, while an object above the floor should produce a
decreasing nearest distance as the robot approaches. The distance is measured
from `base_link`.

## Use the cloud for navigation

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py
```

Simulation navigation starts this point-cloud pipeline by default. Nav2's local
ObstacleLayer or VoxelLayer uses the filtered `base_link` cloud for marking and
the raw optical-frame cloud only for ray-clearing. The saved global map and
global `/scan` obstacle source are unchanged. Nav2 then applies the robot
footprint and inflation layer to turn points into collision clearance.

Verify the resolved local layer and its subscriptions:

```bash
ros2 param get /local_costmap/local_costmap \
  voxel_layer.observation_sources
ros2 topic info /camera/depth/points_filtered --verbose
ros2 topic info /camera/depth/points --verbose
```

The default source list is `scan depth_mark depth_clear`. In RViz, compare the
green **Filtered Obstacles** display with the colored **Local Costmap**. Disable
the camera integration with `use_point_cloud:=false` when measuring a
LiDAR-only baseline or reducing simulation load. In `mode:=hardware`, the
default is already `use_point_cloud:=false`; enable it only when the VMXPi
hardware launch is publishing both cloud topics and cooling has been validated.

Inspect the raw cloud when debugging projection or TF:

```bash
ros2 launch studica_robot_apps point_cloud_observer.launch.py \
  use_sim_time:=true point_cloud_topic:=/camera/depth/points
```

## Physical camera

The Orbbec driver can publish its own point cloud, so the simulation converter
must not run on the VMXPi. Run the following launch through the preserved-env
root wrapper in [Supervised hardware](HARDWARE.md#4-start-the-vmx-hal-correctly).
The physical launch uses depth-only 320×240 at 5 Hz and enables the driver
cloud. Raw `/camera/depth/points` is the default point-cloud output:

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  use_point_cloud:=true use_camera_color:=false \
  use_foxglove:=false use_joystick:=false
ros2 topic list -t | grep -E 'points|PointCloud2'
```

Enable the optional ground/body filter explicitly when
`/camera/depth/points_filtered` is also needed. It samples every second raw
point:

```bash
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  use_point_cloud:=true use_point_cloud_filter:=true \
  use_camera_color:=false use_foxglove:=false use_joystick:=false
```

The normal Gemini E topic is `/camera/depth/points`; the filtered result is
`/camera/depth/points_filtered`. Start only the read-only observer separately:

```bash
ros2 launch studica_robot_apps point_cloud_observer.launch.py \
  use_sim_time:=false point_cloud_topic:=/camera/depth/points
```

During physical navigation, keep the PC launch at `use_point_cloud:=false` so
LiDAR alone marks and clears the Nav2 costmaps. The RViz `Raw PointCloud`
display can still subscribe to `/camera/depth/points`; this launch argument and
the RViz display are independent. Disable the display while driving if the
VMXPi EKF starts missing update-rate deadlines. For the lowest navigation load,
restart hardware with `use_camera:=false use_point_cloud:=false`.

The measured hardware camera TF and robot dimensions must be verified before
using these bounds. Point-cloud processing increases CPU, USB, memory, and
network use; enable it only when the application needs it.

The measured 640×480×15 raw cloud contained about 208,000 points and 3.18 MiB
per message, fell to 3–4.4 Hz, and was rejected for sustained VMXPi use. At
320×240×15 the raw message fell to about 0.77 MiB and 8–9.5 Hz, but the complete
camera/filter workload still reached 82.3 °C and set `get_throttled=0x80008`.
The checked-in 5 Hz depth-only profile is therefore the starting point, not a
guarantee: verify cooling and throttling on the installed robot.

After five minutes of continuous operation, require the active low-order
throttling bits to be clear and record the temperature:

```bash
sudo vcgencmd measure_temp
sudo vcgencmd get_throttled
```

`0x8` means the soft temperature limit is currently active. Stop camera
processing and cool the computer if it appears. A historical high-order bit
may remain set until reboot; do not confuse that sticky history with an active
low-order bit.

Hardware monitoring observes `CameraInfo` only for each deliberately enabled
color or depth stream. These small messages prove that the selected streams are
active and provide their timestamp, optical frame, width, and height without
forcing the image-transport plugins to encode JPEG or compressed depth. A
dashboard or recording that explicitly subscribes to compressed images still
activates that processing and should be included in CPU and thermal testing.

## Troubleshooting

- No raw or filtered point cloud: confirm the simulation was launched with
  `use_point_cloud:=true` and that the workspace was rebuilt and re-sourced.
- Raw points exist but filtered points do not: check
  `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` and the filter log.
- The floor remains: inspect the filtered cloud frame and raise `min_height_m`
  slightly only after confirming `base_link` is on the ground plane.
- RViz reports no transform: set the fixed frame to `odom` and check
  `ros2 run tf2_ros tf2_echo odom camera_optical_frame`.
- The topic exists but a subscriber receives nothing: use best-effort sensor
  QoS and confirm both nodes use the same DDS environment.
- Simulation becomes slow: close duplicate RViz windows, disable the camera
  with `use_point_cloud:=false`, or increase the converter's `stride` parameter.
- VMXPi temperature approaches 80 °C or `get_throttled` has low-order bits set:
  stop the cloud/filter, keep navigation LiDAR-only, and improve active cooling
  before repeating a sustained test.
