# Lab 2 — Simulation and the ROS Graph

## Learning goals

- Launch the beginner simulation.
- Discover nodes, topics, types, publishers, and subscribers.
- Draw a small ROS graph from observed evidence.

## Prerequisites

- Lab 1 checkpoint complete.
- A graphical desktop capable of running Gazebo and RViz.
- Every terminal sourced as shown in Lab 1.

## Terminals

### Terminal 1 — launch the maze simulation

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

Wait until the robot is visible and controller spawners report success.

### Terminal 2 — inspect the graph

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 node list
ros2 topic list -t
ros2 topic info /odom --verbose
ros2 topic info /cmd_vel/joy --verbose
ros2 topic info /joy --verbose
```

### Terminal 3 — visualize and sample

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
rqt_graph
```

In another tab of Terminal 3 after closing `rqt_graph`:

```bash
ros2 topic echo /odom --once
```

## Expected output

- Gazebo shows the `class_4wd` robot in the maze and RViz displays the model.
- `/cmd_vel/joy`, `/joy`, `/odom`, `/imu`, `/scan`, and `/joint_states` are listed.
- `/odom` has type `nav_msgs/msg/Odometry` and a publisher.
- `/cmd_vel/joy` has type `geometry_msgs/msg/Twist`; the deadman teleop node is
  its publisher and the safety supervisor is its subscriber.
- `/joy` connects the joystick driver to both the converter and the supervisor's
  independent L1 check.
- One odometry message contains `pose` and `twist` sections.

## Checkpoint

Draw three boxes representing a command source, the robot controller, and
odometry output. Add `/joy`, `/cmd_vel/joy`, and `/odom` arrows with message
types. Mark which node publishes and which subscribes using the verbose topic
output—not guesses.

## Cleanup

1. Close `rqt_graph`.
2. Press `Ctrl+C` in Terminal 1.
3. Wait for Gazebo and RViz to close.
4. Confirm `ros2 node list` no longer shows the simulated robot nodes.

## Challenge

Relaunch with `gui:=false gz_headless:=true`. Explain which graphical program is
disabled by each argument and confirm the ROS topics still exist. Stop the
headless launch before continuing.
