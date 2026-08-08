# Lab 8 — Nav2 Localization and Goals

## Learning goals

- Localize the simulated robot against a saved occupancy map.
- Send and cancel a Nav2 goal from RViz.
- Observe localization, planning, control, and `/cmd_vel` without competing teleop.

## Prerequisites

- Labs 1–7 complete.
- Mapping launch and keyboard teleop fully stopped.
- The bundled office map, or a reviewed Lab 7 PGM/YAML pair.

## Terminals

### Terminal 1 — launch navigation

Use the bundled baseline first:

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 navigation.launch.py
```

Do not start keyboard teleop during this lab. Nav2 must be the only intentional
motion source after a goal is sent.

### Terminal 2 — inspect Nav2 readiness

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 lifecycle nodes
ros2 action list | grep navigate
ros2 topic echo /map_metadata --once
ros2 run tf2_ros tf2_echo map base_link
```

Stop `tf2_echo` after valid transforms appear.

### Terminal 3 — observe commands and odometry

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /cmd_vel
```

Open another tab for:

```bash
ros2 topic hz /odom
```

Both are observers; they do not command motion.

### Terminal 1 — localize and send one goal in RViz

1. Choose **2D Pose Estimate**.
2. Click the robot's map position and drag in its heading direction.
3. Wait for the scan to align with the map.
4. Choose **Nav2 Goal** and select a nearby clear location.
5. Watch the plan and costmaps; be ready to cancel the goal in RViz.

## Expected output

- Nav2 lifecycle nodes become active and a navigation action is listed.
- After the pose estimate, the laser scan aligns with mapped walls.
- `/cmd_vel` remains quiet before a goal and carries `Twist` commands during it.
- Odometry continues while AMCL supplies the `map -> odom` relationship.
- The robot reaches the clear goal or reports a visible, diagnosable failure.

## Checkpoint

Show a localized robot, one completed or deliberately cancelled goal, and the
observed `/cmd_vel` stream. Explain the roles of `map`, `odom`, and `base_link`,
and name which interface a Nav2 controller uses to request robot motion.

## Cleanup

1. Cancel any active goal in RViz and wait for `/cmd_vel` to return to zero.
2. Stop Terminal 3 observers.
3. Press `Ctrl+C` in Terminal 1.
4. Confirm Gazebo/RViz close and no Nav2 lifecycle nodes remain.

## Challenge

Relaunch with the reviewed Lab 7 map:

```bash
ros2 launch studica_vmxpi_ros2 navigation.launch.py \
  map:="$STUDICA_WS/student_maps/office_student.yaml"
```

Compare localization quality with the bundled map. If it is worse, identify a
specific map artifact or initial-pose error rather than changing TF publishers.
