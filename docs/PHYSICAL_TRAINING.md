# Physical-robot training track

The classroom includes the actual robot as well as simulation. Distinguish
stationary live-sensor exercises from exercises that energize the drivetrain;
they have different prerequisites. The simulation-only `training.sh` wrapper
must not be changed to reach the robot.

## Revised training scope — 2026-09-07

The operator has removed the independent motor-power stop wiring redesign
from the immediate training deliverables because it cannot be implemented
with the current setup. Treat this work as a supervised training prototype,
not a production-approved appliance. Do not keep requesting that same wiring
redesign as the next software task.

This scope decision is not a passing safety result and does not disable DIO8
monitoring, Start/Reset/Stop logic, joystick deadman, command timeouts, disarmed
startup or release activation guards. The original production qualification
record remains incomplete; no acceptance flag or artifact marker is changed.

Before deciding whether a powered motor test can proceed under a revised
training procedure, establish whether an existing, reachable physical main
switch cuts battery power to **both** Titan and VMXPi without relying on
software. Preserving VMXPi logic power need not be a requirement of that
alternative assessment. The switch's existence, circuit and suitability are
not yet physically verified, and identifying one alone does not qualify a
motor test. The operator has now answered "yes" to the question of whether a
reachable physical main switch cuts battery power to both Titan and VMXPi;
record this as an operator report, not a witnessed power-cut test. The current
motion hold remains pending that assessment; sensor-only work can continue.

### Main-switch check preparation

At 19:13:45 Gulf Standard Time on 2026-09-07, SSH preflight found no matching
robot control, sensor, build or package-installation process. Both apt-daily
services were inactive; the unattended-upgrades service was active but no
matching package transaction was observed. Robot load was near zero, the
production target was absent/inactive, and the active-release pointer absent.
The recorded boot ID was `afdf5273-da76-488b-a650-e4b9f2b890d9`.

Preserved the two LiDAR driver ROS logs from the robot's temporary directory
under `robot_test_results/physical_lidar_20260907` on the workstation. Then
issued an orderly `systemctl poweroff`; the command returned successfully and
SSH closed. No drive runtime or motor command was started.

The operator subsequently reported "full robot power is off" after the
main-switch OFF instruction. After the supervised power-on instruction, they
reported the robot powered with E-stop pressed, controls released, wheels
secured and no motion. Record these as operator observations of a no-drive
power cycle, not a driven emergency-stop test.

At 19:16:54 Gulf Standard Time, SSH showed a new boot ID,
`89c28f0c-8e50-490d-88a6-c73997b548b3`. The production target was still
absent/inactive and the active-release pointer absent. Approximately 3.3 GiB
RAM and 4.9 GiB storage were available. No undervoltage or brownout entry was
returned by the current-boot journal query. A separate process-only query
confirmed no matching control, sensor or safety-check process; the initial
combined diagnostic query matched its own shell's diagnostic file paths.
No VMX HAL or drive process was started during these checks.

SSH loss after an OS shutdown is **not** proof that the physical switch
removed power. A new boot ID confirms a new OS boot, not independently the
power-switch circuit. The operator's no-motion/power-off observations do not
establish switch DC ratings, circuit topology, stopping time or behavior under
a driven load. Obtain the main switch's model/DC rating or a qualified hardware
assessment before deciding whether it is suitable for the proposed powered
test. No release guard or motor authorization changed during this check.

## Current inventory — 2026-09-07

See the subsequent [camera and observation deployment](OBSERVER_CAMERA_DEPLOYMENT_2026-09-07.md)
for the latest HTTPS, depth-streaming and map-registry results. Camera streaming
passed and was stopped; the read-only observer is enabled, but browser login
and cold-boot acceptance remain pending.

The later [platform setup deployment](PLATFORM_SETUP_DEPLOYMENT_2026-09-07.md)
installed disabled platform service files and a separate training LiDAR unit.
After SSH access recovered, the corrected X2 service passed scan and scanner
stop/start checks, including a clean systemd restart. Only that loopback-only
training LiDAR service is now enabled and running. This supersedes the earlier
"target absent" inventory below, but does not activate the production platform
or verify a cold boot.

A read-only SSH preflight at 17:24 Gulf Standard Time found:

- VMXPi reachable at `192.168.1.173`, with its existing development workspace;
- `/dev/ydlidar` resolving to `/dev/ttyUSB0`;
- Orbbec Gemini E RGB and depth devices on USB;
- no matching VMX control, safety supervisor, ROS launch, LiDAR or camera process;
- `studica-robot.target` absent/inactive and no active-release pointer.

USB detection is not evidence that either sensor is publishing valid data.
No sensor, HAL, drive controller or production service was started by this
preflight. The E-stop is confirmed connected only to DIO8; it does not
independently remove motor power.

## Track A: stationary physical sensors and programming

The existing `_lidar_hw.launch.py` and `_camera_hw.launch.py` are separate
sensor entry points. Their launch definitions do not include Titan control,
the hardware runtime or a drive controller. This permits a scoped sensor-only
session without activating the staged production release.

Instructor preparation, before students connect:

1. Reconfirm the robot is secured with wheels lifted, controls released and
   E-stop pressed. This is **not** a claim that motor power is isolated.
2. Verify the motor-control runtime and other hardware launches are stopped.
   Do not launch `robot.launch.py`, `bringup.launch.py` hardware mode or the
   production target as a shortcut to obtaining sensor data.
3. Review the existing development-workspace sensor configuration, serial
   device permissions and installed driver versions. Do not change release
   pointers, remove activation guards or use the inactive release as a runtime.
4. Start LiDAR alone as the normal user through its dedicated sensor entry
   point. Check frame ID, valid ranges, topic rate and the process graph.
5. Add camera only for its exercise, starting with low-rate depth, color/IR
   and point cloud off. Verify its supported resolution/rate and VMXPi load.
6. Configure a separate instructor-controlled observation path for student
   PCs. Do not repoint the simulation wrapper or enable unrestricted student
   access to motor-control services. Sensor start/stop stays with the instructor.
7. Stop the sensor session completely after the lesson and recheck processes.

The live exercises are:

| Exercise | Student work | Evidence |
|---|---|---|
| Robot components | Identify VMXPi, Titan, LiDAR, camera and controls without altering wiring | Labelled component sheet |
| Real LiDAR | Inspect `sensor_msgs/msg/LaserScan`, sensor QoS and ranges; place an object in view without touching the robot | Annotated scan and Python subscriber output |
| ROS services/parameters | Inspect the LiDAR driver's `/start_scan` and `/stop_scan` Empty services; instructor demonstrates sensor control | Service/type and parameter notes |
| RGB/depth camera | Compare image and CameraInfo data using the available camera streams | Saved frame and calibration-field notes |
| Record/replay | Instructor records selected sensor topics; students replay the bag in an isolated PC environment | Bag metadata and repeatable subscriber exercise |

LiDAR RViz can use the scan's own frame as Fixed Frame. Do not invent odometry
or publish fake TF to make navigation appear ready. This sensor-only graph
does **not** provide validated wheel odometry, Titan telemetry, VMX IMU data,
or a moving SLAM/navigation exercise. LiDAR start/stop controls the scanner,
not the drivetrain and not an emergency-stop function.

The subsequent [physical LiDAR check](PHYSICAL_LIDAR_VERIFICATION_2026-09-07.md)
passed with the robot's **X2** configuration, not the sensor launcher's generic
Tmini default. Camera streaming, LiDAR service behavior, student connectivity
and recording still require validation. It is not yet a passed complete
physical classroom session. Following the operator's X2 confirmation, the
production sensor script now selects X2 in local source with regression
coverage; the staged robot release remains unchanged. Rebuild, deployment and
verification of that service are still outstanding.

## Track B: physical driving, SLAM and navigation

These remain part of the intended training, not replacements with simulation:

1. instructor-supervised lifted-wheel arming, stopping and command-loss checks;
2. one operator's low-speed joystick exercise in a controlled area;
3. real LiDAR/odometry mapping and a saved map pair;
4. localization and a short navigation goal using the matching physical map,
   including the existing `office_nav` where appropriate;
5. observed cancellation, stop and orderly shutdown.

Do not begin Track B merely because the wiring redesign was removed from the
current scope. Its revised stopping procedure still needs assessment as
described above. The original production requirements in
[Hardware safety gate](HARDWARE_SAFETY_GATE.md) remain unqualified. DIO8, an
operator beside the button, low commanded speed or a simulation test does not
establish independent motor-power removal. No safety acceptance evidence is
fabricated for the training deadline.

The broader signed-update, appliance UX and production-image work is separate
from organizing these lessons. Production autostart still requires the full
[Safety acceptance](SAFETY_ACCEPTANCE.md) qualification. Track A does not
enable it or qualify Track B.
