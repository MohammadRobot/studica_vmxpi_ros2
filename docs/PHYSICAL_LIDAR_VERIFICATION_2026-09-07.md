# Physical LiDAR verification — 2026-09-07

Scope: one standalone USB LiDAR on VMXPi `192.168.1.173`. The operator confirmed
wheels lifted/secured, E-stop pressed and Start released. They also requested
a motor test; no powered drive-motor test was performed because independent
motor-power removal remains absent under the commissioning hold.

No VMX/Titan HAL, drive controller, camera, production service, arm request or
drive command was started. The staged release and activation guards were not
changed. The existing development-workspace LiDAR executable ran as `vmx`,
without sudo, in ROS domain 78 using the checked-in loopback-only Cyclone
profile. Library resolution showed no VMX/Titan dynamic library dependency.

## Attempts and configuration finding

1. A process-check guard initially matched the planned executable path inside
   its own shell command. It stopped the attempt before any driver started.
   A separate read-only process check eliminated that false positive.
2. The generic Tmini preset opened `/dev/ydlidar` at 230400 baud, but failed
   health/device-information retrieval and scan startup. It produced no scans
   and exited. This was not a passing test despite the driver returning zero.
3. Both the local and VMXPi `stack_4wd/robot_profile.yaml` specify
   `hardware.lidar_type: x2`. Retried using the existing **X2.yaml**, 115200
   baud, single-channel mode and its DTR motor support, overriding only the
   serial alias to `/dev/ydlidar` and frame to `laser_scan_frame`.

The X2 configuration controls the scanner's motor, not the robot drivetrain.
No installed parameter file was edited. The SDK reported version 1.2.11 and
the ROS driver reported 1.0.1. The X2 run began at approximately 17:40:32 Gulf
Standard Time; scanning began at approximately 17:40:39.

## Measured scan-stream result

The read-only subscriber received six seconds of data with sensor-data QoS:

| Observation | Result |
|---|---|
| Received scans | 69 |
| Observed rate | 11.233 Hz |
| Frame ID | `laser_scan_frame` |
| Points per scan | 270 |
| Finite, in-range points per scan | 122–131 |
| Invalid frame/timestamp/angular/range metadata samples | 0 |
| Message timestamps | Strictly increasing |
| Last scan's valid range span | Approximately 0.301–4.990 m |
| `/scan` publishers | 1 |
| `/cmd_vel` publishers | 0 |
| `/robot_base_controller/cmd_vel` publishers | 0 |
| Discovered nodes | LiDAR driver and read-only test subscriber only |

This passes the bounded **scan-stream** check. `/start_scan` and `/stop_scan`
were advertised as `std_srvs/srv/Empty`; service behavior was not exercised.
Distance accuracy, extrinsic calibration, student-PC reception and recording
were not qualified by this test.

The SDK still logged `Fail to get baseplate device information!` during the
successful X2 run. Do not describe device metadata retrieval as passed. Valid
scanning was measured independently; the metadata limitation remains recorded
for follow-up rather than inferred away.

## Shutdown and final state

Sent SIGINT through the owned test terminal. The driver reported scanning
stopped at 17:41:26. At 17:41:33, read-only checks found no matching hardware,
LiDAR or probe process and no serial-port owner. The production target was
absent/inactive and `/opt/studica/current` remained absent. No undervoltage,
brownout or USB-disconnect entry was returned by the current-boot journal check.
Raw ROS logs remain under `/tmp/studica-lidar-training-20260907` on VMXPi;
that temporary directory is not durable across cleanup/reboot.

## Deployment mismatch and local correction

At the time of the physical test, `scripts/studica_sensor_runtime` hard-coded
`lidar_type:=tmini`, whereas the robot profile selected X2. That production
script was **not** used for the passing physical test.

Following the operator's explicit X2 confirmation, the local source launcher
now selects `lidar_type:=x2`. Regression coverage checks its command using a
stub ROS executable, agreement with the stack_4wd profile and the launch preset
mapping to `X2.yaml`. Camera arguments remain unchanged. This is a local source
correction, not a rebuilt or deployed release, and the corrected production
service still needs on-robot verification. No release activation guard changed.

The subsequent [platform deployment attempt](PLATFORM_SETUP_DEPLOYMENT_2026-09-07.md)
installed a separate X2 training service and initially found a ROS
environment-loading failure before scanning started. After SSH recovered,
the corrected service passed two scan/stop/start checks, including a clean
service restart, and was enabled for boot. Those separate measurements are
recorded in the deployment report; neither test qualifies a cold boot or the
motion-capable production platform.

Next physical sensor work can validate camera streaming, LiDAR service
behavior, student observation and recording. Drive-motor commissioning remains
separate; this scan result does not qualify physical driving or navigation.
