# Mapping and Navigation

Complete Labs 1–6 before this guide. Mapping and navigation launch robot
components but never start keyboard, gamepad, patrol, or other motion publishers.
You choose when to drive or send a goal.

## Mapping in simulation

**Terminal 1 — office world and SLAM:**

```bash
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 mapping.launch.py
```

Expected: Gazebo opens the office world, RViz displays `/scan`, and SLAM Toolbox
publishes `/map` plus the `map -> odom` transform.

**Terminal 2 — external low-speed teleop:**

```bash
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

Drive slowly around the perimeter, then through the center. Avoid spinning in
place for long periods; overlapping LiDAR views improve the map.

Check progress:

```bash
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map odom
```

## Save a student map

Choose a directory outside the source repository so generated maps are not
accidentally committed:

```bash
mkdir -p "$STUDICA_WS/student_maps"
ros2 run nav2_map_server map_saver_cli \
  -f "$STUDICA_WS/student_maps/office_student"
```

Expected files:

```text
office_student.pgm
office_student.yaml
```

Open the image and identify walls, free space, unknown space, and one mapping
artifact. The bundled `office_map` remains the course baseline.

## Navigation in simulation

Stop mapping and teleop first. Start a fresh navigation session:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py
```

This uses the bundled office map unless `map:=<MAP_YAML>` is supplied. It starts
Nav2 and RViz but no command publisher outside Nav2.

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
```

The controller publishes repeated `/cmd_vel` messages while following a plan.
Cancel the goal in RViz before stopping the launch.

## Use another map

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  map:="$STUDICA_WS/student_maps/office_student.yaml"
```

The map origin and scale are stored in YAML. Moving only the PGM file breaks the
pair, so keep both files together.

## Common checks

```bash
ros2 topic echo /map_metadata --once
ros2 action list | grep navigate
ros2 run tf2_ros tf2_echo map base_link
ros2 run studica_robot_monitor robot_check --mode simulation
```

If Nav2 remains inactive, check lifecycle nodes and the first launch error. If
the costmap is empty, check `/scan` QoS and TF. If the robot pose jumps, repeat
the initial pose carefully rather than adding a new TF publisher.

## Cleanup

1. cancel any active Nav2 goal;
2. press `Ctrl+C` in external teleop, if used;
3. press `Ctrl+C` in the main launch terminal;
4. verify Gazebo and RViz close.

The detailed student exercises are [Lab 7](labs/07_slam.md) and
[Lab 8](labs/08_navigation.md).
