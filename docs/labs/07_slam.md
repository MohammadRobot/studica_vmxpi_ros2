# Lab 7 — SLAM and Map Saving

## Learning goals

- Explain how LiDAR, odometry, and TF support SLAM.
- Build a map by driving deliberately through the office simulation.
- Save and inspect a portable PGM/YAML map pair.

## Prerequisites

- Labs 1–6 complete.
- Previous simulation and teleop processes stopped.
- At least 500 MiB free beneath `$STUDICA_WS` for student artifacts.

## Terminals

### Terminal 1 — launch the office mapping session

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 mapping.launch.py
```

The launch starts SLAM and the robot but no motion publisher.

### Terminal 2 — verify mapping interfaces

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /map_metadata --once
ros2 topic hz /map
```

Stop the rate command with `Ctrl+C`, then inspect the transform:

```bash
ros2 run tf2_ros tf2_echo map odom
```

### Terminal 3 — drive the map slowly

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.10 -p turn:=0.25
```

Drive the perimeter first, then the center. Pause after turns so scans overlap.
Use `k` to stop before examining RViz.

### Terminal 4 — save outside the source repository

After the office has been observed from several directions:

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
mkdir -p "$STUDICA_WS/student_maps"
ros2 run nav2_map_server map_saver_cli \
  -f "$STUDICA_WS/student_maps/office_student"
ls -lh "$STUDICA_WS/student_maps/office_student."{pgm,yaml}
```

## Expected output

- RViz shows `/map` growing as new `/scan` data arrives.
- TF includes `map -> odom -> base_footprint/base_link -> laser_scan_frame`.
- The robot remains stationary until Terminal 3 starts teleop.
- Saving creates both `office_student.pgm` and `office_student.yaml`.
- The YAML names the image and records resolution, origin, and thresholds.

## Checkpoint

Open the PGM and identify occupied, free, and unknown areas plus one mapping
artifact. Show the matching YAML and explain why both files must stay together.
Demonstrate that no `office_student` files were written inside the repository.

## Cleanup

1. Press `k`, then `Ctrl+C`, in Terminal 3.
2. Confirm the robot is stationary and the map save command has finished.
3. Stop `tf2_echo` or other inspection commands.
4. Press `Ctrl+C` in Terminal 1 and wait for Gazebo/RViz to close.

## Challenge

Create a second map using smoother turns and more scan overlap. Compare file
size and visible artifacts without replacing the first map. Write one evidence-
based rule for improving future mapping runs.
