# Robot Health, Titan Velocity PID, and Foxglove

This workflow is for the `class_4wd` VMX robot. That profile requires Titan2 MCV2 encoder-velocity control and does not fall back to duty-cycle control.

## 1. Install runtime components

```bash
sudo apt install -y \
  ros-humble-diagnostic-aggregator \
  ros-humble-diagnostic-updater \
  ros-humble-forward-command-controller \
  ros-humble-foxglove-bridge \
  ros-humble-rosbag2-storage-mcap
```

Build the integrated stack:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to \
  studica_drivers studica_robot_monitor studica_vmxpi_ros2
source install/setup.bash
```

## 2. Measure the wheel before enabling motion

Hardware motion is intentionally disabled while `hardware.wheel_radius_calibrated` is `false`.

1. Measure the loaded wheel diameter through the tread in millimetres. Measure several orientations and use the mean.
2. Convert to metres and divide by two.
3. Put that number in `bringup/config/profiles/class_4wd/robot_profile.yaml` as `drive.wheel_radius_m`.
4. Set `hardware.wheel_radius_calibrated: true` only after checking the value.

`drive.wheel_radius_m` is injected into the URDF, hardware encoder conversion, and runtime drive-controller parameters. Do not add a second wheel-radius value to `robot_controllers.yaml`.

## 3. Hardware bringup and safety behavior

```bash
ros2 launch studica_vmxpi_ros2 bringup.launch.py \
  mode:=hardware robot_profile:=class_4wd gui:=false
```

On startup the hardware plugin probes Titan firmware, requires MCV2 support, configures PID type 2 with sensitivity 5, waits for a fresh safe controller-temperature sample while motor output remains disabled, and zeros all four targets before enabling. Unsupported firmware, non-finite commands, failed CAN writes, overtemperature, stale temperature telemetry while moving, or stale encoder feedback while moving cause all motors to receive zero and latch a hardware fault. A latch is cleared only by deliberately deactivating and reactivating the `Robot` hardware component or restarting bringup after fixing the cause.

Encoder/RPM and controller-temperature telemetry have separate freshness limits. Encoder/RPM feedback uses the 100/250 ms warning/error limits. Titan controller temperature is broadcast at approximately 1 Hz, so `controller_temp_error_timeout_ms` defaults to 3000 ms; do not reuse the encoder timeout for temperature telemetry.

Titan firmware 2.0.5 encodes `MCU_TEMP` in degrees Fahrenheit. The workspace driver detects firmware 2.0.5 (and later 2.0.x patch releases) and converts the payload to Celsius before publishing controller state or applying the 80 °C safety limit. Older firmware remains decoded as Celsius; unknown firmware families are deliberately not guessed and therefore fail safe if their reported value exceeds the temperature limit.

Useful checks:

```bash
ros2 topic echo /robot_status/motors
ros2 topic echo /diagnostics_agg
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 4. Read-only Foxglove dashboard

Hardware mode enables monitoring and Foxglove by default. Override with these launch arguments:

```text
use_monitoring:=true
use_foxglove:=true
foxglove_address:=ROBOT_PRIVATE_LAN_IP
foxglove_port:=8765
```

The bridge exposes only whitelisted telemetry, diagnostics, TF, ROS logs, and compressed camera topics. Its only client capability is `connectionGraph`; client publishing, service calls, and parameter access are excluded. Keep port 8765 restricted to the trusted robot LAN with the host firewall and never forward it to the Internet.

The address defaults to `127.0.0.1` as a safe local-only setting. For a remote laptop, pass the robot's private address on the trusted LAN; do not use `0.0.0.0` on a robot with any untrusted network interface.

On the laptop, connect Foxglove to `ws://ROBOT_LAN_IP:8765`, then import:

```text
share/studica_vmxpi_ros2/foxglove/class_4wd_robot_health.layout.json
```

The layout contains Overview, Motors / PID, IMU / Odometry, Camera / LiDAR, and Logs / Graph tabs. Foxglove layout import is documented at <https://docs.foxglove.dev/docs/visualization/layouts>.

## 5. Lifted four-wheel validation

Place the robot on a stable lift with every wheel clear. Keep a physical emergency stop within immediate reach. Then run locally on the robot:

```bash
ros2 run studica_robot_monitor validate_motors \
  --robot-lifted --emergency-stop-ready
```

The utility loads the inactive forward-command test controller, atomically switches away from the base controller, and tests each wheel independently at `+2` and `-2 rad/s` with a one-second ramp, two-second hold, and one-second stop observation. It passes only when encoders stay fresh, signs are correct, idle wheels stay below `0.25 rad/s`, settled error is within `0.3 rad/s` or 15%, overshoot is below 25%, and the wheel stops below `0.2 rad/s` within one second.

Each run produces `~/robot_test_results/motor_validation_<UTC>/report.yaml` and an MCAP telemetry directory. Camera topics are excluded unless `--include-camera` is supplied. Titan autotune is never called.

## 6. Floor calibration

After the lifted test passes, use a low command speed on a clear floor:

1. Compare straight odometry distance with a measured physical distance and refine `drive.wheel_radius_m`.
2. Compare an in-place odometry rotation with a measured rotation and refine `wheel_separation`.
3. Repeat the lifted test if wheel radius or motor/encoder inversion changed.
4. Store the final YAML report and Foxglove/MCAP recording as the calibration baseline.

ROS diagnostics follow the standard `diagnostic_updater` and `diagnostic_aggregator` flow. Foxglove diagnostics support is described at <https://docs.foxglove.dev/docs/visualization/panels/diagnostics>.
