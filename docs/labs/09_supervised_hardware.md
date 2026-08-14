# Lab 9 — Supervised Hardware Readiness and Low-Speed Control

This lab is optional. It never starts motion automatically. A launch, health
check, topic echo, or controller listing must leave every target velocity at
zero. Any movement begins only after the safety operator and instructor approve
a separate, human-operated teleop action.

## Learning goals

- Apply a written readiness gate before enabling physical motion.
- Compare simulation interfaces with the real `stack_4wd` robot.
- Perform and stop one instructor-approved, low-speed teleop observation.

## Prerequisites

- Labs 1–8 completed and signed by the instructor.
- [Supervised hardware](../HARDWARE.md) reviewed by the whole team.
- Instructor-provided VMXPi image/SDK and a successful hardware build.
- Measured `drive.wheel_radius_m` with `wheel_radius_calibrated: true`.
- Supported Titan MCV2 firmware and a reviewed lifted-wheel validation PASS
  report prepared by the instructor before this supervised lab.
- Inspected wiring, battery, CAN, wheel fasteners, stable lift, and test area.
- Assigned safety operator beside a tested physical emergency stop.

Stop for unexpected/reversed motion, stale feedback, a fault latch, unusual
noise, hot electronics, loose wiring, or lost communication. Never increase a
timeout or bypass a diagnostic to continue.

## Terminals

### Terminal 1 — instructor starts hardware with motion sources absent

On the VMXPi, with motor power disabled initially:

```bash
export STUDICA_WS="$HOME/studica_ws"
sudo --preserve-env=ROS_DOMAIN_ID,RMW_IMPLEMENTATION,ROS_LOCALHOST_ONLY,CYCLONEDDS_URI \
  env STUDICA_WS="$STUDICA_WS" bash -lc '
    cd "$STUDICA_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch studica_vmxpi_ros2 robot.launch.py \
      use_camera:=false use_foxglove:=false
  '
```

The instructor must stop here if HAL, pigpio, firmware, controller, temperature,
or encoder initialization reports an error.

### Terminal 2 — operator runs read-only checks

On the VMXPi as the normal user:

```bash
export STUDICA_WS="$HOME/studica_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run studica_robot_monitor robot_check --mode hardware --timeout 5
ros2 control list_controllers
ros2 topic echo /robot_status/motors --once
ros2 topic echo /odom --once
ros2 topic echo /imu --once
ros2 topic echo /scan --once --qos-reliability best_effort
```

None of these commands publishes `/cmd_vel`.

### Safety Gate A — instructor and safety operator sign before motion

Do not continue until every item is true:

- all three required controllers are active;
- four motors and four fresh encoders are reported;
- target and measured wheel velocities are zero;
- `pid_supported: true`, `pid_type: 2`, and supported firmware are reported;
- `fault_latched: false` and temperature is below its limit;
- the previous guarded lifted-wheel validation report passed;
- the lift/test lane is stable and clear;
- the emergency stop is reachable and its operator says “ready.”

Record instructor and safety-operator initials plus the time. A WARN requires an
instructor decision; any FAIL ends the lab.

### Terminal 3 — human-operated low-speed control

Only after Safety Gate A, enable motor power as directed by the instructor. Keep
the wheels on the approved lift for the first direction check. The terminal
operator runs:

```bash
export STUDICA_WS="$HOME/studica_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p speed:=0.05 -p turn:=0.15
```

The safety operator says “clear,” then the terminal operator taps one agreed
key and immediately presses `k`. Confirm direction and zero speed. Floor motion
is a separate instructor decision and uses the same short, low-speed, manually
held commands in a marked clear lane.

### Terminal 4 — observe feedback during the approved action

```bash
export STUDICA_WS="$HOME/studica_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 topic echo /robot_status/motors
```

Observe target, measured velocity, error, feedback age, freshness, saturation,
and health. Terminal 4 never publishes a command.

## Expected output

- Startup holds zero targets and all required controllers become active.
- `robot_check` reports no required failure; PID type 2, fresh feedback, a
  plausible Celsius temperature, and an unlatched fault are visible.
- A human keypress produces only the agreed low-speed direction.
- `k`, loss of the command stream, or emergency stop brings motion to zero.
- No launch or check initiates a wheel test, autonomous path, or stored command.

## Checkpoint

Submit the signed Gate A sheet, firmware/PID/temperature facts, controller list,
and the instructor's existing lifted-validation report path. Demonstrate a
stopped robot with zero target/measured velocities. Explain why a healthy later
message must not automatically clear a latched hardware fault.

## Cleanup

1. Press `k`, then stop teleop with `Ctrl+C`.
2. Verify target and measured velocities are zero.
3. Disable motor power before leaving the robot.
4. Stop feedback observers.
5. The instructor stops Terminal 1 and waits for a complete HAL shutdown.
6. Save reports and record every WARN/FAIL; disconnect power per classroom rules.

## Challenge

Without enabling motor power, compare a saved simulation `/odom` sample with a
saved hardware `/odom` sample. Identify the shared fields and explain how wheel
radius error would affect distance. Do not run another motion test for the
challenge.
