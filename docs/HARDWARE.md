# Supervised Hardware

This guide is for an instructor and students working together. Complete the
simulation labs first. The person at the robot must be able to press a physical
emergency stop immediately.

Hardware motion is never part of installation, CI, `robot_check`, mapping
startup, navigation startup, or ordinary package tests.

## Roles and stop conditions

Assign these roles before power is applied:

- **safety operator:** stays beside the emergency stop and watches the robot;
- **terminal operator:** runs only the command agreed with the safety operator;
- **observer:** keeps people and objects outside the test area.

Stop immediately for unexpected motion, reversed motion, stale feedback, a
fault latch, unusual noise, hot electronics, loose wiring, or loss of network.
Do not continue by raising a timeout or disabling a diagnostic.

## 1. Mechanical and electrical inspection

With motor power disabled:

1. place the robot on a stable four-wheel lift;
2. confirm every wheel is clear and cannot contact the support;
3. inspect wheel fasteners, encoder wiring, motor wiring, battery, CAN, and fuse;
4. test that the physical emergency stop removes motor power;
5. rotate each wheel by hand and check for binding;
6. keep camera and LiDAR cables away from wheels.

The VMXPi remains powered for software checks only when the instructor considers
that safe. Motor power stays disabled until the health report is understood.

## 2. Install and verify the robot computer

Run as the normal VMXPi account:

```bash
export STUDICA_WS="$HOME/ros2_ws"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode hardware --check-only
./scripts/setup_ubuntu.sh --mode hardware
```

The check must identify Ubuntu 22.04, arm64, and the VMXPi vendor HAL. The setup
builds and tests without starting the HAL or motors.

## 3. Measure the wheel radius

`drive.wheel_radius_m` is the only radius used by URDF geometry, encoder
conversion, and controller validation.

1. Measure loaded tread diameter in several wheel orientations.
2. Average the measurements, convert to metres, and divide by two.
3. Update `bringup/config/profiles/class_4wd/robot_profile.yaml`:

```yaml
drive:
  wheel_radius_m: <MEASURED_RADIUS_METRES>
hardware:
  wheel_radius_calibrated: true
```

Set the flag only after entering and reviewing the measurement. Hardware
bringup refuses motion while it is false. Do not add a second radius to a
controller file.

Rebuild after changing the source profile:

```bash
cd "$STUDICA_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select studica_vmxpi_ros2
source install/setup.bash
```

## 4. Start the VMX HAL correctly

The VMX HAL uses protected memory and pigpio resources, so the hardware process
must run as root while retaining the selected ROS network environment. Do not
run the whole setup script as root.

First keep camera and remote access off to minimize load:

```bash
export STUDICA_WS="$HOME/ros2_ws"
sudo --preserve-env=ROS_DOMAIN_ID,RMW_IMPLEMENTATION,ROS_LOCALHOST_ONLY,CYCLONEDDS_URI \
  env STUDICA_WS="$STUDICA_WS" bash -lc '
    cd "$STUDICA_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch studica_vmxpi_ros2 robot.launch.py \
      use_camera:=false use_foxglove:=false
  '
```

This is one shell command even though it spans lines. Expected startup facts:

- Titan firmware probing succeeds;
- firmware supports Titan2 MCV2 velocity PID;
- PID type is `2` and sensitivity is `5`;
- all four target velocities remain zero;
- base, joint-state, and IMU controllers become active;
- monitoring begins without commanding motion.

If the HAL reports permission or pigpio errors, stop and correct the root launch
environment. Do not retry motion from the failed process.

## 5. Run read-only checks

Open a normal-user terminal on the robot:

```bash
export STUDICA_WS="$HOME/ros2_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run studica_robot_monitor robot_check --mode hardware
ros2 control list_controllers
ros2 topic echo /robot_status/motors --once
ros2 topic echo /diagnostics_agg --once
```

Required observations before motor power is enabled:

- every expected controller is active;
- all four motor channels are present and encoder feedback is fresh;
- target and measured velocities are zero;
- `pid_supported` is true and `pid_type` is `2`;
- `fault_latched` is false;
- Titan firmware matches the supported family;
- controller temperature is plausible and below the configured limit.

Titan firmware 2.0.5 reports MCU temperature in Fahrenheit. The low-level
driver converts that payload to Celsius before `/robot_status/motors`,
diagnostics, and the 80 °C safety comparison. Do not convert the published value
a second time.

## 6. Guarded lifted-wheel validation

Enable motor power only after the safety operator confirms the stable lift and
reachable emergency stop. No one touches the robot during this test.

```bash
ros2 run studica_robot_monitor validate_motors \
  --robot-lifted \
  --emergency-stop-ready
echo "Exit code: $?"
```

The utility atomically switches from the base controller to an inactive
four-joint forward-command controller. It tests each wheel independently at
`+2` and `-2 rad/s` with a one-second ramp, two-second hold, and stop interval.
It always commands zero and attempts to restore the base controller on success,
failure, timeout, cancellation, or process exit.

Passing requires:

- fresh feedback and correct direction;
- uncommanded wheels below `0.25 rad/s`;
- settled mean error within `0.3 rad/s` or 15%;
- overshoot below 25%;
- stop below `0.2 rad/s` within one second.

Expected success ends with eight `PASS` lines, a timestamped report path, and
exit code `0`. Review the YAML even after success:

```text
<RESULT_DIRECTORY>/motor_validation_<UTC>/report.yaml
```

The run also records selected telemetry and diagnostics as MCAP. Camera data is
excluded unless explicitly requested.

## 7. Fault-latch behavior

Non-finite commands, unsupported PID, failed CAN writes, overtemperature, stale
temperature while moving, or stale encoder feedback while moving zero all four
targets and latch a fault. A later healthy message does not clear it.

To recover:

1. remove motor power and identify the cause;
2. stop all command publishers;
3. correct wiring, temperature, CAN, firmware, or feedback issue;
4. deliberately deactivate/reactivate the hardware component or restart
   bringup;
5. rerun `robot_check` and the full lifted-wheel validation.

Never add an automatic fault reset to a student node.

## 8. Low-speed floor calibration

Proceed only after a saved lifted-wheel PASS report.

1. Mark a straight, level, obstruction-free test lane.
2. Place the safety operator alongside the robot, not in front of it.
3. Publish repeated low-speed commands with supervised teleop.
4. Compare odometry distance with a measured physical distance; refine wheel
   radius if needed.
5. Compare an in-place odometry rotation with a measured rotation; refine wheel
   separation.
6. Rebuild, then repeat the lifted test after geometry or inversion changes.
7. Store the final YAML report and MCAP/Foxglove baseline recording.

The command timeout must stop motion if the publisher disappears. Verify zero
targets after every run.

## 9. Camera and Foxglove

After core health passes, restart hardware bringup with the conservative camera
default. Foxglove stays on loopback unless an exact trusted-LAN robot address is
supplied. Follow [Networking](NETWORKING.md); never bind a read/write control
surface or forward the bridge to the internet.

## Shutdown

1. stop remote or local teleop;
2. verify zero target and measured wheel velocities;
3. disable motor power;
4. press `Ctrl+C` in hardware bringup and wait for shutdown;
5. disconnect the battery according to the classroom procedure;
6. save reports and record any WARN/FAIL observations.

Lab 9 turns this procedure into a student worksheet. The deeper safety and PID
implementation is documented in [Robot health and velocity PID](ROBOT_HEALTH_AND_PID.md).
