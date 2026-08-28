# Mapping and Navigation

Complete Labs 1–6 before this guide. Mapping starts deadman-protected joystick
teleoperation by default. Navigation keeps joystick teleoperation disabled so
Nav2 remains the sole motion owner after you send a goal.

Simulation starts in `READY_DISARMED`. After each mapping or navigation launch,
use a second sourced terminal to run:

```bash
ros2 topic echo /robot/state --once
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
```

Hardware software-arming is disabled in the current production phase; the
physical workflows below are read-only until the local hardware gate is added.

For simulation, use the local simulation DDS profile. This prevents a
previously sourced robot Wi-Fi or Ethernet profile from leaking into the local
Gazebo session:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
```

## Mapping in simulation

**Terminal 1 — office world and SLAM:**

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 mapping.launch.py
```

Expected: Gazebo opens the office world, RViz displays `/scan`, and SLAM Toolbox
publishes `/map` plus the `map -> odom` transform.

Hold L1 and drive slowly around the perimeter with the left stick, then through
the center; use the right stick to turn. Avoid spinning in place for long
periods because overlapping LiDAR views improve the map. For keyboard control,
launch `mapping.launch.py use_joystick:=false` before starting keyboard teleop.

Check progress:

```bash
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

## Mapping with the physical robot: two launches

This workflow has one launch on each computer. The VMXPi owns the Titan,
controllers, robot model, LiDAR, fused odometry, TF, and monitoring. The PC owns the
joystick, SLAM Toolbox, and RViz. Do not start another joystick or SLAM process.

On the VMXPi, source the generated robot Wi-Fi profile and start the low-load
LiDAR hardware configuration. Keep the work area clear and the emergency stop
reachable:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_vmxpi_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

sudo --preserve-env=ROS_DOMAIN_ID,RMW_IMPLEMENTATION,ROS_LOCALHOST_ONLY,CYCLONEDDS_URI \
  env STUDICA_WS="$STUDICA_WS" bash -lc '
    cd "$STUDICA_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec ros2 launch studica_vmxpi_ros2 robot.launch.py \
      use_lidar:=true use_camera:=false use_point_cloud:=false \
      use_camera_color:=false use_foxglove:=false use_joystick:=false
  '
```

On the PC, connect the joystick, source the matching peer profile, and start the
single mapping client launch:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

ros2 launch studica_vmxpi_ros2 mapping.launch.py mode:=hardware
```

The PC launch uses wall-clock time, starts RViz in the `map` frame, and applies
conservative hardware teleop limits: 0.08 m/s and 0.25 rad/s normally, or
0.12 m/s and 0.40 rad/s while R1 turbo is held. Motion still requires L1.
Release L1 before inspecting RViz or saving the map.

`mode:=hardware` automatically selects
`slam_toolbox_hardware_mapper_params.yaml`. This X2-specific profile processes
every scan, accepts poses at 2 cm/0.02 rad intervals, uses 5 cm map cells, and
places scans from the calibrated wheel/IMU odometry rather than allowing noisy
X2 matches to continuously rotate the pose graph. A conservative loop closure
can correct return-to-start drift only after a long, high-confidence match.
Simulation continues to use
`slam_toolbox_mapper_params.yaml`. An explicit `slam_params_file:=...` still
overrides either default.

The hardware launch enables IMU-assisted odometry by default. On the VMXPi,
raw encoder feedback appears on `/wheel/odom`; `robot_localization` combines
its forward velocity with `/imu` yaw and yaw rate, then becomes the only
publisher of `/odom` and `odom -> base_footprint`. This is important for the
high-grip 4WD chassis: the map no longer inherits encoder yaw caused by tire
scrub during a turn. Straight distance still comes from the calibrated wheels.

Before driving, confirm `/cmd_vel` has one external publisher and the remote
sensor/TF graph is available:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once --field header
ros2 topic echo /wheel/odom --once --field header
ros2 node info /hardware_odometry_filter
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

If `/hardware_odometry_filter` is absent, install the declared dependency on
the VMXPi with `sudo apt install ros-humble-robot-localization`, rebuild, and
restart `robot.launch.py`. Use `use_imu_odometry:=false` only for a deliberate
encoder-only comparison; it restores the controller as the `/odom` and odom-TF
owner.

In RViz, the occupancy map is grayscale. Begin a new session after any failed
map; restarting the PC launch clears the in-memory map. Drive straight sections
with gentle stick input, release L1 briefly before and after each turn, cover the
perimeter and center, then approach the starting area slowly from approximately
the original direction and remain stopped for 2--3 seconds. A single small map
adjustment at that point is expected when loop closure succeeds; progressive
wall duplication is not. Do not use turbo during the first validation map. Stop
both launches before switching back to simulation or starting navigation.

Confirm that the hardware profile was selected:

```bash
ros2 param get /slam_toolbox throttle_scans
ros2 param get /slam_toolbox resolution
ros2 param get /slam_toolbox minimum_travel_distance
ros2 param get /slam_toolbox use_scan_matching
ros2 param get /slam_toolbox do_loop_closing
```

Expected values are `1`, `0.05`, `0.02`, `False`, and `True`. With this
odometry-first hardware profile, `map -> odom` should stay close to its initial
value during ordinary motion. It may adjust once after a valid loop closure.
Stationary walls must remain fixed in `map`; duplicated or continuously sliding
walls are not normal.

## Save a project map

Choose a directory outside the source repository so generated maps are not
accidentally committed:

```bash
mkdir -p "$STUDICA_WS/project_maps"
ros2 run nav2_map_server map_saver_cli \
  -f "$STUDICA_WS/project_maps/office_project"
```

Expected files:

```text
office_project.pgm
office_project.yaml
```

Open the image and identify walls, free space, unknown space, and one mapping
artifact. The bundled `office_map` remains the course baseline.

Occupancy images are intentionally grayscale: black is occupied, white is free,
and gray is unknown. During navigation, RViz overlays **Global Costmap** and
**Local Costmap** using a colored cost scheme. Their cyan, red, and purple bands
show increasing obstacle/inflation cost; they are not stored in the PGM image.

## Navigation in simulation

Stop mapping first. Start a fresh navigation session:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 navigation.launch.py
```

This uses the bundled office map unless `map:=<MAP_YAML>` is supplied. It starts
Nav2, the depth-camera obstacle pipeline, and RViz. Joystick teleoperation stays
off by default. Do not enable it while Nav2 is controlling `/cmd_vel` unless a
command mux is added.

Perception ownership is deliberate:

- the saved map and `/scan` remain the global planning inputs;
- `/scan` continues to mark and clear the local costmap;
- `/camera/depth/points_filtered` marks camera obstacles in the local costmap;
- `/camera/depth/points` supplies complete optical rays that clear stale camera
  costs but are never allowed to mark the floor as an obstacle.

In RViz, green **Filtered Obstacles** are sensor points. Colored **Local
Costmap** cells are the collision costs Nav2 actually uses. They should overlap
near an observed object, with extra colored inflation around its footprint.

The hardware launch overlay reads `xacro.overall_length` and
`xacro.overall_width` from the `stack_4wd` profile and writes the same
rectangular footprint to the local and global costmaps. The current measured
footprint is 0.350 x 0.385 m. Keep
clearance policy in Nav2's inflation layer; do not enlarge the physical profile
to imitate a safety margin.

In RViz:

1. choose **2D Pose Estimate**;
2. click the robot's map position and drag in its heading direction;
3. wait for the laser scan to align with the map;
4. choose **Nav2 Goal** and set a nearby, obstacle-free goal.

Inspect Nav2 while it runs:

```bash
ros2 lifecycle nodes
ros2 topic echo /cmd_vel
ros2 topic hz /odom
ros2 topic hz /camera/depth/points_filtered
ros2 param get /local_costmap/local_costmap \
  voxel_layer.observation_sources
```

The final command should include `scan depth_mark depth_clear` when the default
Nav2 VoxelLayer is in use. A custom parameter file may name its supported local
ObstacleLayer differently; the launch overlay locates that configured layer.
For a LiDAR-only baseline, relaunch with `use_point_cloud:=false`.

The controller publishes repeated `/cmd_vel` messages while following a plan.
Cancel the goal in RViz before stopping the launch.

## Navigation with the physical robot: two launches

Use the accepted map saved during the physical mapping session. Stop the PC
mapping launch so SLAM Toolbox and joystick teleop are gone; Nav2/AMCL must not
compete with them. The VMXPi remains the owner of hardware, fused odometry,
LiDAR, robot transforms, and monitoring.

On the VMXPi:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_vmxpi_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

sudo --preserve-env=ROS_DOMAIN_ID,RMW_IMPLEMENTATION,ROS_LOCALHOST_ONLY,CYCLONEDDS_URI \
  env STUDICA_WS="$STUDICA_WS" bash -lc '
    cd "$STUDICA_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    exec ros2 launch studica_vmxpi_ros2 robot.launch.py \
      use_lidar:=true use_camera:=false use_point_cloud:=false \
      use_camera_color:=false use_foxglove:=false use_joystick:=false
  '
```

On the PC, verify the saved pair and start one navigation client launch:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_pc_wifi.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"

test -s "$STUDICA_WS/project_maps/real_robot_map.yaml"
test -s "$STUDICA_WS/project_maps/real_robot_map.pgm"

ros2 launch studica_vmxpi_ros2 navigation.launch.py mode:=hardware
```

The confirmed `office_nav` setup uses the reviewed YAML/PNG pair and keeps the
raw camera cloud visualization-only:

```bash
test -s "$STUDICA_WS/project_maps/office_nav.yaml"
test -s "$STUDICA_WS/project_maps/office_nav.png"

ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  mode:=hardware \
  map:="$STUDICA_WS/project_maps/office_nav.yaml" \
  robot_profile:=stack_4wd \
  use_point_cloud:=false \
  hardware_max_linear_speed:=0.20 \
  hardware_max_angular_speed:=0.35
```

If raw depth visualization is required, the VMXPi hardware launch may use
`use_camera:=true use_point_cloud:=true use_point_cloud_filter:=false`. The PC
navigation launch must still use `use_point_cloud:=false`; this prevents raw
optical-frame points, including floor and self returns, from entering the Nav2
costmap. The saved RViz configuration can display `/camera/depth/points`
independently.

The hardware map default is
`$HOME/studica_ws/project_maps/real_robot_map.yaml`. Override it only when a
different reviewed YAML/PGM pair is intended:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  mode:=hardware map:="$STUDICA_WS/project_maps/another_map.yaml"
```

Hardware mode runs map server, AMCL, Nav2, and RViz on the PC with wall-clock
time. It does not start Gazebo, joystick teleop, controllers, or sensor drivers.
The parameter overlay preserves the measured 0.350 x 0.385 m footprint and
applies these physical limits:

- path speed: at most 0.20 m/s and 0.35 rad/s by default;
- recovery turns: at most 0.35 rad/s by default;
- smoothed acceleration: 0.18 m/s2 and 0.45 rad/s2;
- X2 localization range: 0.12 to 5.0 m with 5 cm AMCL updates.

The physical limits can be changed without editing YAML. For a slower test:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  mode:=hardware \
  hardware_max_linear_speed:=0.16 \
  hardware_max_angular_speed:=0.35
```

The launch accepts at most 0.30 m/s and 0.60 rad/s. Increase one step at a time
and repeat a nearby-goal test after each change; higher limits are intentionally
rejected by this supported physical profile.

Camera costmap integration is disabled by default in hardware navigation.
First prove LiDAR-only navigation. Enable `use_point_cloud:=true` on the PC
only when the VMXPi deliberately publishes the filtered obstacle cloud with
`use_point_cloud_filter:=true` and its temperature/throttling state remains
acceptable. Keep it false for the confirmed raw-visualization-only setup.

Raw point-cloud transport can increase VMXPi and Wi-Fi load even though Nav2
does not consume it. If `hardware_odometry_filter` reports missed update-rate
deadlines, disable the RViz `Raw PointCloud` display first. If warnings persist,
restart VMXPi bringup with `use_camera:=false use_point_cloud:=false` and keep
the proven LiDAR-only navigation path. Do not reduce the 10 Hz LiDAR rate to
compensate for camera load.

Before setting a goal, keep the physical emergency stop reachable and run these
checks on the PC:

```bash
ros2 lifecycle nodes
ros2 action list | grep -E 'navigate|follow'
ros2 topic echo /scan --once --field header.frame_id
ros2 topic echo /odom --once --field header.frame_id
timeout 5 ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /cmd_vel --verbose
```

All Nav2 lifecycle nodes should be active, `/scan` should use
`laser_scan_frame`, and `odom -> base_link` should be available. The velocity
smoother is the only Nav2 publisher on `/cmd_vel`; no teleop publisher should
be present.

In RViz, select **2D Pose Estimate**, click the robot's measured map position,
and drag in its real heading. Do not send a goal until the live red scan aligns
with the mapped walls and stays aligned while the robot is stationary. Use a
nearby clear goal for the first floor test. Cancel it immediately if the robot
localization jumps, the scan leaves the walls, or the local/global costmaps do
not show the expected clearance.

The bundled office waypoint route is simulation-specific. Never start it on the
physical map. Create and preview a separate route whose coordinates have been
checked against `real_robot_map.yaml` before using `--start`.

## Follow the bundled office waypoint route

Keep navigation running. Preview the route first; preview mode cannot command
motion:

```bash
ros2 run studica_vmxpi_ros2 run_waypoint_route.py
```

The five targets stay inside the office walls and away from the known obstacle
centers. Confirm the displayed coordinates, clear any active RViz goal, then
explicitly start the route:

```bash
ros2 run studica_vmxpi_ros2 run_waypoint_route.py --start \
  --ros-args -p use_sim_time:=true
```

The action reports each current waypoint and ends with `Waypoint route completed
successfully`. Press `Ctrl+C` to request cancellation.

Create a project-owned route without changing the bundled baseline:

```bash
mkdir -p "$STUDICA_WS/project_routes"
cp "$STUDICA_WS/src/studica_vmxpi_ros2/bringup/config/waypoints/office_loop.yaml" \
  "$STUDICA_WS/project_routes/office_custom.yaml"
ros2 run studica_vmxpi_ros2 run_waypoint_route.py \
  --route "$STUDICA_WS/project_routes/office_custom.yaml"
```

Edit the copy, preview again, and add `--start` only after every target is in
free space. Never assume the bundled coordinates are safe in another world.

## Use another map

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  map:="$STUDICA_WS/project_maps/office_project.yaml"
```

The map origin and scale are stored in YAML. Moving only the PGM file breaks the
pair, so keep both files together.

## Common checks

```bash
ros2 topic echo /map_metadata --once
ros2 action list | grep -E 'navigate|follow'
ros2 run tf2_ros tf2_echo map base_link
ros2 run studica_robot_monitor robot_check --mode simulation
```

If Nav2 remains inactive, check lifecycle nodes and the first launch error. If
the costmap is empty, check `/scan`, both point-cloud topics, sensor QoS, and TF.
If the robot pose jumps, repeat the initial pose carefully rather than adding a
new TF publisher.

## Cleanup

1. cancel any active Nav2 goal or waypoint route;
2. release L1 if mapping joystick teleop is active;
3. press `Ctrl+C` in the main launch terminal;
4. verify Gazebo and RViz close.

The detailed guided exercises are [Lab 7](labs/07_slam.md) and
[Lab 8](labs/08_navigation.md).
