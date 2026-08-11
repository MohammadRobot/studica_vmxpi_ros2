# Lab 7 — SLAM and Map Saving

## Learning goals

- Explain how LiDAR, odometry, and TF support SLAM.
- Build a map by driving deliberately through the office simulation.
- Save and inspect a portable PGM/YAML map pair.

## Prerequisites

- Labs 1–6 complete.
- Previous simulation and teleop processes stopped.
- At least 500 MiB free beneath `$STUDICA_WS` for project artifacts.

## Terminals

### Terminal 1 — launch the office mapping session

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 mapping.launch.py
```

The launch starts SLAM and deadman-protected joystick teleoperation. The robot
remains stopped until L1 is held and a stick is moved.

### Terminal 2 — verify mapping interfaces

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /map_metadata --once
ros2 topic hz /map
```

Stop the rate command with `Ctrl+C`, then inspect the transform:

```bash
ros2 run tf2_ros tf2_echo map odom
```

### Drive the map slowly

Hold L1 and use the left stick vertically to drive; use the right stick to
turn. Drive the perimeter first, then the center. Release L1 before examining
RViz, and pause after turns so scans overlap.

### Terminal 4 — save outside the source repository

After the office has been observed from several directions:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
mkdir -p "$STUDICA_WS/project_maps"
ros2 run nav2_map_server map_saver_cli \
  -f "$STUDICA_WS/project_maps/office_project"
ls -lh "$STUDICA_WS/project_maps/office_project."{pgm,yaml}
```

## Expected output

- RViz shows `/map` growing as new `/scan` data arrives.
- TF includes `map -> odom -> base_footprint/base_link -> laser_scan_frame`.
- The robot remains stationary until L1 is held and a stick is moved.
- Saving creates both `office_project.pgm` and `office_project.yaml`.
- The YAML names the image and records resolution, origin, and thresholds.

## Checkpoint

Open the PGM and identify occupied, free, and unknown areas plus one mapping
artifact. Show the matching YAML and explain why both files must stay together.
Demonstrate that no `office_project` files were written inside the repository.
Remember that black is occupied, white is free, and gray is unknown; colored
inflation bands appear later as RViz costmap overlays, not in the PGM.

## Cleanup

1. Release L1 and center both sticks.
2. Confirm the robot is stationary and the map save command has finished.
3. Stop `tf2_echo` or other inspection commands.
4. Press `Ctrl+C` in Terminal 1 and wait for Gazebo/RViz to close.

## Challenge

Create a second map using smoother turns and more scan overlap. Compare file
size and visible artifacts without replacing the first map. Write one evidence-
based rule for improving future mapping runs.
