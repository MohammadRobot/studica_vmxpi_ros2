# Lab 3 — Topics, Teleoperation, and `/cmd_vel`

## Learning goals

- Read and publish typed ROS topic data.
- Relate `Twist.linear.x` and `Twist.angular.z` to robot motion.
- Use an external low-speed teleop node and stop it cleanly.

## Prerequisites

- Labs 1–2 complete.
- The robot is in simulation, not connected to physical motor control.
- A clear area around the simulated robot.

## Terminals

### Terminal 1 — launch simulation

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 launch studica_vmxpi_ros2 sim.launch.py use_joystick:=false
```

### Terminal 2 — observe commands

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /cmd_vel
```

It is normal to see nothing until a command source starts.

### Terminal 3 — drive slowly

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.10 -p turn:=0.25
```

Keep Terminal 3 focused. Tap `i` to move forward, `j` or `l` to turn, and `k`
to stop. Use short taps and avoid the maze walls.

### Terminal 4 — inspect the interface

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 interface show geometry_msgs/msg/Twist
ros2 topic hz /cmd_vel
```

Press `Ctrl+C` after measuring the rate.

## Optional DualShock 4 path

After completing the keyboard checkpoint, stop the launch and Terminal 3, then
relaunch without the override to use the default joystick. Follow
[Joystick teleoperation](../JOYSTICK.md) to:

1. verify that Linux created `/dev/input/js0`;
2. prove the stick values change on `/joy`;
3. confirm the launch started `teleop_twist_joy` with the L1 deadman;
4. publish the same `geometry_msgs/msg/Twist` contract on `/cmd_vel`.

Keep Terminal 2 open. It should show the same message type whether commands come
from the keyboard or joystick. Do not run both teleop programs at once.

## Expected output

- Terminal 2 prints `linear` and `angular` vectors while keys are pressed.
- Positive `linear.x` moves forward; positive or negative `angular.z` turns.
- Unused components remain zero.
- When Terminal 3 stops publishing, the controller timeout brings the robot to
  rest; there is no embedded teleop publisher in the launch.

## Checkpoint

Save one `/cmd_vel` message and annotate its units: metres per second for
`linear.x` and radians per second for `angular.z`. Demonstrate forward, turn,
stop, and `Ctrl+C`, then confirm the robot remains stopped.

## Cleanup

1. Press `k` in Terminal 3.
2. Press `Ctrl+C` in Terminal 3, then Terminal 2.
3. Verify the robot is stationary.
4. Press `Ctrl+C` in Terminal 1 and wait for Gazebo to close.

## Challenge

In simulation only, publish one small `Twist` and observe the timeout:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

Explain why a stream of commands is normally needed for continuous driving.
