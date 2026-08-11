# Lab 6 — `ros2_control`, Odometry, and Diagnostics

## Learning goals

- Identify hardware interfaces, controllers, and broadcasters.
- Connect wheel feedback to `/joint_states` and `/odom`.
- Read standard diagnostics without changing controller state.

## Prerequisites

- Labs 1–5 complete.
- The Lab 5 application node stopped.
- `studica_robot_monitor` built in the classroom workspace.

## Terminals

### Terminal 1 — launch simulation

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py use_joystick:=false
```

### Terminal 2 — launch simulation-aware monitoring

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_robot_monitor monitoring.launch.py \
  use_sim_time:=true monitor_lidar:=true monitor_camera:=false
```

This observer does not publish `/cmd_vel` or switch controllers.

### Terminal 3 — inspect control and health

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
ros2 topic echo /odom --once
ros2 topic echo /diagnostics --once
ros2 run studica_robot_monitor robot_check --mode simulation
```

Raw simulation diagnostics may identify hardware-only Titan fields as
unavailable. `robot_check --mode simulation` evaluates the components required
for simulation and remains read-only.

### Terminal 4 — create feedback with low-speed teleop

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.10 -p turn:=0.25
```

Tap forward, turn, and stop while watching `/odom` or `/joint_states` in another
tab. Do not use controller-internal command topics.

## Expected output

- `joint_state_broadcaster`, `imu_sensor_broadcaster`, and
  `robot_base_controller` report `active`.
- Wheel state interfaces include position and velocity; command interfaces are
  claimed by the base controller.
- Wheel positions and odometry change while driving and settle after stopping.
- `/diagnostics` uses named status entries and levels.
- `robot_check` prints a PASS/WARN/FAIL table without causing motion.

## Checkpoint

Draw the data path from `/cmd_vel` through `robot_base_controller` and wheel
interfaces to `/joint_states` and `/odom`. Record controller states and one
diagnostic name, level, message, and key/value. Demonstrate that stopping teleop
stops motion while monitoring continues.

## Cleanup

1. Press `k`, then `Ctrl+C`, in Terminal 4.
2. Confirm the robot is stationary.
3. Stop monitoring in Terminal 2.
4. Stop simulation in Terminal 1 and wait for Gazebo/RViz to close.

## Challenge

Record `/cmd_vel`, `/joint_states`, and `/odom` to a short MCAP during a square
path, then replay only for analysis with no simulator running. Compare commanded
turns with odometry and explain one source of estimation error.
