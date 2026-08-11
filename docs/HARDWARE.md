# Supervised Hardware

This guide is for an instructor and operators working together. Complete the
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
export STUDICA_WS="$HOME/studica_ws"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode hardware --check-only
./scripts/setup_ubuntu.sh --mode hardware
```

The check must identify Ubuntu 22.04, arm64, and the VMXPi vendor HAL. The setup
builds and tests without starting the HAL or motors.

## 3. Measure physical geometry and wheel radius

`drive.wheel_radius_m` is the only radius used by URDF geometry, encoder
conversion, and controller validation.

Measure the body envelope, ground clearance, wheelbase, wheel track, outer
footprint, and sensor poses before testing motion. Use `base_link` on the
ground at the centre of the four wheel contact points. Sensor poses in the
profile are relative to the body-centred `chassis_link`.

For the confirmed `class_4wd` platform, the profile records a 0.340 x 0.290 x
0.200 m body, 0.080 m clearance, 0.190 m wheelbase, 0.340 m physical track,
and 0.350 x 0.385 m outer envelope. The simulated collision body, wheels,
LiDAR, camera, IMU, and Nav2 footprint all consume those measurements. The
controller retains its separately validated effective 0.500 m wheel
separation.

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
colcon build --symlink-install \
  --packages-select studica_drivers studica_vmxpi_ros2 \
  --allow-overriding studica_drivers
source install/setup.bash
```

## 4. Start the VMX HAL correctly

The VMX HAL uses protected memory and pigpio resources, so the hardware process
must run as root while retaining the selected ROS network environment. Do not
run the whole setup script as root.

First generate and test the matching PC and VMXPi peer profiles from
[Networking and Cyclone DDS](NETWORKING.md). In the VMXPi launch terminal,
activate the profile for the selected link:

```bash
source "$HOME/.ros/studica_vmxpi_wifi.env"  # or studica_vmxpi_ethernet.env
printenv ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION CYCLONEDDS_URI
```

Do not continue if `CYCLONEDDS_URI` contains an address absent from
`ip -br address`.

First keep camera and remote access off to minimize load:

```bash
export STUDICA_WS="$HOME/studica_ws"
sudo --preserve-env=ROS_DOMAIN_ID,RMW_IMPLEMENTATION,ROS_LOCALHOST_ONLY,CYCLONEDDS_URI \
  env STUDICA_WS="$STUDICA_WS" bash -lc '
    cd "$STUDICA_WS"
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch studica_vmxpi_ros2 robot.launch.py \
      use_lidar:=true \
      use_camera:=false \
      use_point_cloud:=false \
      use_camera_color:=false \
      use_foxglove:=false \
      use_joystick:=false
  '
```

This is one shell command even though it spans lines. Expected startup facts:

- Titan firmware probing succeeds;
- firmware supports Titan2 MCV2 velocity PID;
- PID type is `2` and sensitivity is `5`;
- all four target velocities remain zero;
- the launch reports `Hardware ros2_control rate: 25 Hz`;
- base, joint-state, and IMU controllers become active;
- monitoring begins without commanding motion.

If the HAL reports permission or pigpio errors, stop and correct the root launch
environment. Do not retry motion from the failed process.

## 5. Run read-only checks

Open a normal-user terminal on the robot:

```bash
export STUDICA_WS="$HOME/studica_ws"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
ros2 run studica_robot_monitor robot_check \
  --mode hardware --timeout 5 --skip-lidar
ros2 param get /controller_manager update_rate
ros2 control list_controllers
ros2 topic echo /robot_status/motors --once
ros2 topic echo /diagnostics_agg --once
```

Use `--skip-lidar` only when the same launch explicitly used
`use_lidar:=false`. It omits `/scan`, the LiDAR TF requirement, and the LiDAR
diagnostic while retaining every controller, odometry, IMU, compute, Titan,
motor, and encoder check. With LiDAR enabled, omit `--skip-lidar`.

The hardware default is 25 Hz because measured 50 Hz and 100 Hz loops on the
VMXPi increased CPU and thermal load. The 25 Hz setting sustained controller,
odometry, IMU, and joint-state publication near 25 Hz and passed the guarded
motor test in all eight wheel directions, including controller recovery.
Simulation and mock mode remain at 100 Hz. Confirm the active hardware value is
`25` rather than editing the checked-in controller YAML.

Before guarded wheel validation, require `throttled=0x0` and a stable CPU
temperature below 70 °C:

```bash
sudo vcgencmd measure_temp
sudo vcgencmd get_throttled
```

Required observations before motor power is enabled:

- every expected controller is active;
- all four motor channels are present and encoder feedback is fresh;
- target and measured velocities are zero;
- `pid_supported` is true and `pid_type` is `2`;
- `fault_latched` is false;
- Titan firmware matches the supported family;
- controller temperature is plausible and below the configured limit.

### IMU data convention and physical checks

`/imu` follows REP-145: angular velocity and linear acceleration are expressed
in `imu_link`. Hardware acceleration uses the VMX raw sensor axes, converts g
to m/s^2, and retains gravity. The gravity-corrected VMX world-acceleration API
is deliberately not mixed into this sensor-frame message.

With the robot stationary and level, verify that Z is positive and close to
`+9.81 m/s^2` while X and Y remain small:

```bash
ros2 topic echo /imu --once --field linear_acceleration
```

Then rotate the robot about Z. Counterclockwise angular velocity and yaw change
must be positive; clockwise angular velocity must be negative. Because the IMU
is mounted at +90 degrees yaw, its X/Y sensor axes are not the same as the body
X/Y axes; TF performs that rotation for consumers that need `base_link` data.

The Titan `MCU_TEMP` payload contains whole and hundredths bytes but no unit
flag. Firmware 2.0.5 has been confirmed on this platform to publish Fahrenheit;
the low-level driver converts exactly that firmware to Celsius. Unknown
firmware is never guessed, so an unexpectedly high value still fails the
startup gate. `/robot_status/motors`, diagnostics, and the 80 °C comparison are
always Celsius after the supported normalization.

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

Never add an automatic fault reset to an application node.

## 8. Low-speed floor calibration

Proceed only after a saved lifted-wheel PASS report.

1. Mark a straight, level, obstruction-free test lane.
2. Place the safety operator alongside the robot, not in front of it.
3. Record `pose.pose.position` immediately before the run, publish repeated
   low-speed commands, stop explicitly, and record the position again. Use the
   change in odometry, not the final absolute `x`, as the odometry distance.
4. Compare the odometry distance with the measured physical distance. Keep the
   measured loaded radius in `drive.wheel_radius_m`. If repeated long runs show
   a consistent scale error, calculate a symmetric controller correction with:

   ```text
   new_multiplier = current_multiplier * physical_distance / odometry_distance
   effective_rolling_radius = measured_radius * new_multiplier
   ```

   Apply the result to both `left_wheel_radius_multiplier` and
   `right_wheel_radius_multiplier` in the profile's `robot_controllers.yaml`.
   Repeat the run in both directions and use the average correction. Only use
   different left and right values when repeated measurements demonstrate a
   side-specific error. Do not calibrate from one short run or from commanded
   speed multiplied by time.
5. Hardware defaults to `use_imu_odometry:=true`. It uses wheel forward
   velocity for distance and IMU yaw/yaw-rate for rotation, avoiding the large
   encoder-yaw error caused by sticky skid-steer tires. Confirm `/wheel/odom`
   and `/imu` are present, then compare a measured 90-degree turn with the
   change in fused `/odom` yaw. Do not tune effective wheel separation from
   that fused result because encoder yaw is intentionally excluded.
6. Rebuild and restart hardware bringup after changing geometry, then repeat
   the lifted test before another floor run.
7. Store the final profile value, YAML report, and MCAP/Foxglove baseline
   recording.

For a controlled encoder-only comparison, start hardware with
`use_imu_odometry:=false`. Never run an additional EKF beside that launch: the
normal launch enforces one `/odom` and `odom -> base_footprint` owner at a time.

The command timeout must stop motion if the publisher disappears. Verify zero
targets after every run.

## 9. Camera and Foxglove

After core health passes, restart hardware bringup with the conservative camera
default. Foxglove stays on loopback unless an exact trusted-LAN robot address is
supplied. Follow [Networking](NETWORKING.md); never bind a read/write control
surface or forward the bridge to the internet.

The default physical camera stream is depth-only 320×240 at 5 Hz. Color and
point clouds are separate opt-ins. Use each launch line below as the final
`ros2 launch` command inside the preserved-env root wrapper from step 4:

```bash
# Low-load depth health test.
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  use_camera:=true use_camera_color:=false use_point_cloud:=false \
  use_foxglove:=false use_joystick:=false

# Depth obstacle test; adds the raw and filtered point clouds.
ros2 launch studica_vmxpi_ros2 robot.launch.py \
  use_point_cloud:=true use_camera_color:=false \
  use_foxglove:=false use_joystick:=false
```

Run only one hardware launch at a time. After five minutes, check:

```bash
timeout 10 ros2 topic hz /camera/depth/points_filtered
sudo vcgencmd measure_temp
sudo vcgencmd get_throttled
```

Stop the camera workload if the temperature approaches 80 °C or any active
low-order throttling bit is set. `0x8` specifically means the soft temperature
limit is active. Use LiDAR-only navigation until active cooling provides a
repeatable sustained pass. See [Camera and point cloud](CAMERA_POINT_CLOUD.md)
for the measured baseline and filter bounds.

## Shutdown

1. stop remote or local teleop;
2. verify zero target and measured wheel velocities;
3. disable motor power;
4. press `Ctrl+C` in hardware bringup and wait for shutdown;
5. disconnect the battery according to the classroom procedure;
6. save reports and record any WARN/FAIL observations.

Lab 9 turns this procedure into a guided worksheet. The deeper safety and PID
implementation is documented in [Robot health and velocity PID](ROBOT_HEALTH_AND_PID.md).
