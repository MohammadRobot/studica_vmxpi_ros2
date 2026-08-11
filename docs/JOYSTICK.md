# Joystick Teleoperation

Use this guide to drive simulation or a remote physical robot with a DualShock
4 or a compatible Linux joystick. `sim.launch.py` and `mapping.launch.py` start
the configured joystick nodes by default and publish only the stable public
`/cmd_vel` interface. In `mapping.launch.py mode:=hardware`, the joystick is
attached to the PC while `robot.launch.py` runs separately on the VMXPi. Motion
still requires the L1 deadman button.

Test in simulation first. For hardware, follow the instructor-supervised process
in [Supervised hardware](HARDWARE.md) and keep the physical emergency stop
reachable.

## 1. Stop other motion publishers

Use only one teleop source at a time. Stop keyboard teleop, navigation, and any
older manual gamepad process before continuing. Inspect publishers when unsure:

```bash
ros2 topic info /cmd_vel --verbose
```

The bringup adapter may appear as an internal subscriber or publisher on
controller-facing topics. The concern is multiple external programs publishing
`/cmd_vel`.

## 2. Confirm Linux sees the controller

Connect the controller, then run:

```bash
ls -l /dev/input/js*
grep -A 8 -B 2 'Handlers=.*js' /proc/bus/input/devices
```

A single controller normally appears as `/dev/input/js0`. The DualShock 4 may
be named `Wireless Controller` when connected over Bluetooth. If no `js` device
exists, reconnect the controller before starting ROS nodes.

The simulation installer includes `joy` and `teleop_twist_joy`. Confirm them:

```bash
ros2 pkg prefix joy
ros2 pkg prefix teleop_twist_joy
```

If either is absent, rerun the simulation installer from
[Installation](INSTALL.md).

For a dependency-only repair, install the two ROS packages directly:

```bash
sudo apt update
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy
```

## 3. Default launch behavior

Start simulation or mapping normally:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py
# or
ros2 launch studica_vmxpi_ros2 mapping.launch.py
```

The launch starts both `joy_node` and `teleop_twist_joy_node` using
`config/dualshock4_teleop.yaml`. Do not start another copy of either node.

## 4. Manual diagnostic mode

When diagnosing one boundary at a time, first launch with
`use_joystick:=false`, then start raw input manually in another sourced terminal:

In a sourced terminal:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run joy joy_node --ros-args \
  -p device_id:=0 -p deadzone:=0.10 -p autorepeat_rate:=20.0
```

Expected output includes:

```text
Opened joystick: Wireless Controller
```

In another sourced terminal, verify that moving both sticks changes numbers:

```bash
ros2 topic echo /joy --field axes
```

Do not continue until the axes change. A steady stream of neutral samples only
proves that the node is alive, not that the current controller connection is
delivering events.

### Convert the DualShock input to `/cmd_vel`

Keep the `joy_node` terminal running. In a third sourced terminal:

```bash
ros2 run teleop_twist_joy teleop_node --ros-args \
  -p require_enable_button:=true \
  -p enable_button:=4 \
  -p enable_turbo_button:=5 \
  -p axis_linear.x:=1 \
  -p scale_linear.x:=0.30 \
  -p scale_linear_turbo.x:=0.60 \
  -p axis_angular.yaw:=3 \
  -p scale_angular.yaw:=0.80 \
  -p scale_angular_turbo.yaw:=1.20 \
  -p publish_stamped_twist:=false \
  -r cmd_vel:=/cmd_vel
```

This mapping uses:

- left stick vertical: forward and reverse;
- right stick horizontal: turn;
- L1, button index `4`: required deadman button;
- R1, button index `5`: turbo while L1 remains held.

Normal speed is limited to `0.30 m/s` and `0.80 rad/s`. Turbo is limited to
`0.60 m/s` and `1.20 rad/s`. Release L1 or both sticks to command zero.

Linux controller mappings vary. If the wrong control moves, inspect `/joy` while
moving one stick or pressing one button at a time, then change the corresponding
`axis_*` or `*_button` parameter. Never disable the enable button merely to hide
an incorrect mapping.

## 5. Verify the complete chain

With simulation running, check each boundary in order:

```bash
ros2 topic echo /joy --field axes
ros2 topic echo /cmd_vel
ros2 control list_controllers
ros2 topic echo /odom --once
```

Hold L1 and move one stick during the first two commands. Expected results:

1. `/joy` axes change;
2. `/cmd_vel` changes from zero;
3. the base controller is `active`;
4. the robot moves and `/odom` updates.

If a boundary works but the next one does not, troubleshoot that boundary rather
than changing controller-internal topics.

## Controller reconnects but stays neutral

If Bluetooth disconnects and reconnects after `joy_node` starts, Linux may create
a new joystick device while the existing process still holds the old device
handle. A common symptom is `/joy` publishing at 20 Hz while every axis remains
neutral.

Stop both joystick terminals with `Ctrl+C`, confirm the current device, and
restart them:

```bash
ls -l /dev/input/js*
```

Wait for `Opened joystick` again and repeat the raw `/joy` test. Do not start a
second `joy_node` alongside the stale one: duplicate neutral messages can
overwrite valid stick input.

## DDS and workspace checks

The simulator, `joy_node`, and teleop node must use the same `ROS_DOMAIN_ID`,
`RMW_IMPLEMENTATION`, and `CYCLONEDDS_URI`. Compare their terminal environments:

```bash
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
```

For local simulation, source `~/.ros/studica_sim.env` in every simulator or
manual joystick terminal.
For remote hardware, source the generated PC Wi-Fi or Ethernet environment in
both joystick terminals. Generate these files with the helper in
[Networking and Cyclone DDS](NETWORKING.md).

Use one workspace overlay per terminal. An automatically sourced older workspace
can expose a legacy gamepad executable with a different topic contract. The
supported path in this guide always publishes `geometry_msgs/msg/Twist` on
`/cmd_vel`.

See [Networking](NETWORKING.md) when nodes exist but cannot discover one another.

## Cleanup

1. release L1 and center both sticks;
2. verify the robot is stopped;
3. stop the main launch, which also stops its joystick nodes;
4. in manual diagnostic mode, stop teleop and then `joy_node` before simulation.

The controller timeout is a backup. Cleanly stopping the motion publisher is the
normal end of a teleoperation session.
