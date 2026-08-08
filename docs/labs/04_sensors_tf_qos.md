# Lab 4 — Sensors, TF, RViz, and QoS

## Learning goals

- Inspect IMU, LiDAR, wheel-state, and odometry messages.
- Follow coordinate transforms from `odom` to sensor frames.
- Recognize why sensor topics commonly use best-effort QoS.

## Prerequisites

- Labs 1–3 complete.
- Simulation stopped after Lab 3 cleanup.
- Camera remains disabled to keep the lesson responsive.

## Terminals

### Terminal 1 — launch simulation and RViz

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

### Terminal 2 — sample each sensor interface

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /imu --once
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
```

Then measure two rates, stopping each command with `Ctrl+C`:

```bash
ros2 topic hz /imu
ros2 topic hz /scan
```

### Terminal 3 — inspect TF

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run tf2_ros tf2_echo odom base_link
```

After a few transforms, stop it and run:

```bash
ros2 run tf2_tools view_frames
```

Open the generated `frames.pdf` and find `odom`, `base_link`, `imu_link`, and
`laser_scan_frame`.

### Terminal 4 — compare QoS

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --qos-reliability best_effort
```

In RViz, inspect the RobotModel, LaserScan, and TF displays. Change only display
settings; do not add a new TF publisher.

## Expected output

- `/imu` contains orientation, angular velocity, and linear acceleration.
- `/scan` contains angles, valid range limits, and an array of ranges.
- `/joint_states` names all four wheel joints.
- `tf2_echo` prints a changing or steady transform without an extrapolation
  loop, and the PDF shows one connected robot tree.
- The verbose scan information identifies offered/requested QoS policies; the
  explicit best-effort subscription receives sensor data.

## Checkpoint

Complete a table with topic, message type, frame ID, measured rate, and one
useful field for `/imu`, `/scan`, `/joint_states`, and `/odom`. On the TF diagram,
trace the path from `odom` to `laser_scan_frame` and explain why LiDAR ranges
need a frame before RViz can place them.

## Cleanup

1. Stop every `echo`, `hz`, or `tf2_echo` process.
2. Close the generated PDF and RViz inspection dialogs.
3. Press `Ctrl+C` in Terminal 1.
4. Remove `frames.pdf` if it is not part of your submitted worksheet.

## Challenge

Inspect the covariance arrays in `/imu` and `/odom`. Find their definitions with
`ros2 interface show`, then explain why a covariance is different from a sensor
value. Use only observation commands; do not add another sensor or TF publisher.
