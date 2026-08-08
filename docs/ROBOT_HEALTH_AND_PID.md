# Robot Health and Titan Velocity PID

This is the implementation reference for the supervised `class_4wd` robot. The
student-facing procedure is [Supervised hardware](HARDWARE.md).

## Safety guarantees

The hardware stack guarantees:

- wheel radius has one source: `drive.wheel_radius_m`;
- motion is refused until `hardware.wheel_radius_calibrated` is true;
- `class_4wd` requires Titan2 MCV2 encoder-velocity control;
- unsupported firmware never falls back to open-loop duty control;
- target and measured signs use the same robot-frame convention;
- unsafe input or feedback zeros every motor and latches a fault;
- a fault latch requires deliberate hardware reactivation;
- controller command timeout stops motion when `/cmd_vel` disappears;
- monitoring and remote visualization never publish motor commands.

These are runtime behavior, not dashboard conventions. A missing dashboard does
not remove a hardware safety action.

## Startup sequence

Hardware activation keeps output disabled while it:

1. validates the calibrated radius and finite profile values;
2. opens the VMXPi and Titan CAN interface;
3. probes firmware with bounded retries;
4. requires MCV2 capability when `pid.require_supported` is true;
5. configures PID type `2` and sensitivity on the driver's `0–10` scale;
6. reads a plausible, fresh controller temperature;
7. obtains fresh RPM/encoder feedback for all active channels;
8. sends zero target RPM to all motors;
9. exposes command interfaces only after the safe state is confirmed.

The tested Titan firmware is 2.0.5. A different firmware must pass probing and
capability checks; generation is not inferred from a product label.

## Command conversion and signs

`ros2_control` supplies wheel angular velocity in radians per second. Each
control cycle converts it to Titan target RPM:

```text
target_rpm = target_rad_s * 60 / (2 * pi)
```

The configured safety limit clamps the robot-frame target before the driver
applies its physical motor inversion. Encoder/RPM feedback applies the matching
inverse transform before conversion back to radians per second. Therefore a
positive target and positive measured velocity always mean the same robot-frame
wheel direction.

Non-finite commands are faults, not values to clamp. A CAN write failure while
sending any channel triggers a best-effort zero to every channel and latches the
component.

## Exported state

Each wheel exports standard position and velocity plus these hardware state
interfaces:

| State | Meaning |
|---|---|
| `commanded_velocity` | accepted robot-frame target in rad/s |
| `feedback_age` | seconds since fresh encoder/RPM feedback |
| `encoder_fresh` | `1` while feedback meets the error deadline |
| `command_saturated` | `1` when a finite command was clamped |
| `fault_latched` | `1` after a fail-safe event |

Controller-level state also exposes Titan temperature, temperature age, PID
capability/type, fault latch, and firmware major/minor/patch.

The monitor publishes the structured `/robot_status/motors` message. Each entry
contains joint name, Titan channel, target/measured/error velocity, position,
feedback age, freshness, saturation, and health level/message.

## Freshness policy

The `class_4wd` profile defaults to:

```yaml
hardware:
  feedback_warn_timeout_ms: 100
  feedback_error_timeout_ms: 250
  controller_temp_error_timeout_ms: 3000
```

Encoder/RPM feedback is expected much faster than Titan temperature, which is
broadcast at approximately 1 Hz. The two timeouts must remain separate.

Stale encoder data at rest produces health information without creating motion.
Stale data while a wheel is commanded is unsafe: all motors are zeroed and the
fault is latched. Stale temperature while moving receives the same fail-safe
response.

## Titan 2.0.5 temperature units

Titan firmware 2.0.5 encodes `MCU_TEMP` in degrees Fahrenheit. The driver
converts it once:

```text
celsius = (fahrenheit - 32) * 5 / 9
```

Published state and diagnostics are Celsius. The configured 80 °C limit is
compared to that converted value. Older known firmware that reports Celsius is
not converted; unknown firmware families are not guessed.

Unit tests cover the Fahrenheit conversion, Celsius compatibility path, and
temperature-limit decisions. A dashboard must label the final value as °C.

## Latched fault causes

| Cause | Immediate response | Recovery requirement |
|---|---|---|
| non-finite wheel command | zero all channels | fix publisher; reactivate |
| unsupported required PID | keep output disabled | supported Titan firmware |
| target CAN write failure | best-effort zero all | repair CAN; reactivate |
| overtemperature | zero all channels | cool/inspect; reactivate |
| stale temperature while moving | zero all channels | restore telemetry; reactivate |
| stale encoder/RPM while moving | zero all channels | restore feedback; reactivate |

Normal feedback recovery does not clear the latch. This prevents intermittent
wiring or CAN faults from silently resuming a previous command.

## Diagnostic flow

`studica_robot_monitor` observes:

- four wheel encoders and tracking behavior;
- `/imu`, `/scan`, `/odom`, camera streams, joint states, and TF;
- controller-manager state;
- Titan PID capability, firmware, fault latch, and temperature;
- Pi CPU, memory, disk, and temperature.

It detects missing/stale or malformed data, invalid IMU quaternion, missing TF,
inactive controllers, stuck/reversed encoders, excessive error, overspeed, and
left/right disagreement. It publishes `/diagnostics`.

`diagnostic_aggregator` groups `/diagnostics_agg` and
`/diagnostics_toplevel_state` into Motors, Sensors, Control, and Compute. The
standalone `robot_check` samples the graph read-only and presents a finite
PASS/WARN/FAIL table suitable for a lab checkpoint.

## Guarded validator

`validate_motors` is the only provided standalone wheel test. It requires both
explicit confirmations, atomically switches controllers, ramps one wheel at a
time, records telemetry, commands zero in all exit paths, and restores the base
controller.

Reports go to a timestamped directory and include YAML plus MCAP. Camera streams
are omitted unless explicitly requested. Acceptance thresholds are documented
in [Supervised hardware](HARDWARE.md).

Titan's automatic tuning API is deliberately not integrated: it lacks reliable
completion and result feedback. PID changes are instructor-owned profile
changes followed by the entire lifted and floor acceptance sequence.

## Read-only Foxglove bridge

The hardware launch defaults the bridge to `127.0.0.1`. Supplying an exact
trusted-LAN address enables remote observation. The allowlist contains robot
telemetry, diagnostics, TF, logs, compressed color, and compressed depth. The
only advertised client capability is `connectionGraph`; client publishing,
parameter mutation, and service calls are disabled.

Import `foxglove/class_4wd_robot_health.layout.json` for Overview, Motors/PID,
IMU/Odometry, Camera/LiDAR, and Logs/Graph tabs. Follow
[Networking](NETWORKING.md) for LAN and firewall policy.

## Regression scope

Software tests cover RPM conversion, inversion, PID configuration, capability
rejection, clamping, stale-feedback latching, CAN failure, safe zeroing,
temperature conversion, monitor health cases, and validator thresholds.

A change to hardware behavior requires, in order:

1. unit and package tests;
2. mock/headless simulation contracts;
3. read-only hardware readiness;
4. guarded lifted-wheel regression;
5. measured low-speed floor regression;
6. updated baseline report and recording.

Documentation-only changes do not initiate a physical motor regression.
