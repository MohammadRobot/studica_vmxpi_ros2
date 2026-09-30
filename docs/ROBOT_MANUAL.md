# Robot Manual

Updated 2026-09-30 · Ubuntu 22.04 · ROS 2 Humble · Gazebo Harmonic

The Pi runs hardware, sensors and odometry. The PC runs RViz, joystick, SLAM and
Nav2. Remote driving, live RViz data, map saving and navigation movement have been
confirmed during supervised testing. Full production acceptance remains pending.
**The classroom runtime now starts at power-on, with motion disabled.**
A software reboot confirmed automatic startup with E-stop detected and all four
wheel targets/speeds zero; a full cold-boot campaign is still pending.
Local Reset/Start remains required. This is the tested native development
installation; signed production activation and full acceptance remain pending.

## 1. Prepare a PC terminal

For the real robot, run once in each PC terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.config/studica/robot-wifi.sh
```

This workstation uses domain **78**, Wi-Fi **192.168.1.181**, and robot
**192.168.1.173**. Keep both on the configured network. Check connectivity with:

```bash
ssh vmx@192.168.1.173
# Type exit to return to the PC.
```

## 2. Power on the robot

Power on and allow the Pi to boot. The `studica-classroom.service` starts hardware,
LiDAR, odometry and TF automatically. It uses the same application command input
for **drive, SLAM and navigation**. No SSH command or robot restart is needed to
switch PC modes. Camera is off by default.

Use physical **Stop** before changing modes. Close the old PC launch with Ctrl+C,
then start the desired launch below. Wait for live robot data before pressing
Reset/Start. Only one PC session may command the robot at a time.

## 3. Drive with joystick and RViz

Connect the joystick to the PC, then use the prepared PC terminal:

```bash
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=drive
```

Wait for the robot model and scan. With a clear test area and E-stop reachable,
release E-stop, press/release **Reset**, then **Start**.

| Control | Action |
|---|---|
| L1 + left stick | Forward/reverse, up to 0.20 m/s |
| L1 + right stick | Turn, up to 0.60 rad/s |
| L1 + R1 | Turbo: 0.30 m/s, 0.90 rad/s |
| Release L1 | Stop joystick motion |

To use lower speeds:

```bash
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=drive \
  linear_speed:=0.10 angular_speed:=0.30 \
  turbo_linear_speed:=0.10 turbo_angular_speed:=0.30
```

Stop the previous launch before starting another. Do not run separate joystick
nodes alongside this launch. First-time wheel tests should use raised wheels.

## 4. SLAM and save a map

Stop the PC drive launch, then:

```bash
ros2 launch studica_vmxpi_ros2 remote.launch.py mode:=slam
```

Drive slowly, avoiding turbo, with overlapping scans. Stop the robot before saving.
In another **prepared PC terminal**, while SLAM is still running:

```bash
mkdir -p ~/studica_ws/project_maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/studica_ws/project_maps/my_room
```

Keep **my_room.yaml** and **my_room.pgm** together. Wait for `Map saved successfully`.

## 5. Navigate with the saved map

Press physical **Stop** and stop SLAM with Ctrl+C. In the prepared PC terminal:

```bash
ros2 launch studica_vmxpi_ros2 remote.launch.py \
  mode:=navigation \
  map:=$HOME/studica_ws/project_maps/my_room.yaml
```

1. In RViz, select **2D Pose Estimate** and set the actual position and heading.
2. Confirm the laser scan aligns with the mapped walls.
3. Press/release **Reset**, then **Start**.
4. Send a short, clear **Nav2 Goal**. Keep E-stop within reach.

Navigation uses 0.10 m/s, 0.30 rad/s limits and now includes joystick takeover.
Do not run SLAM, keyboard control or another joystick publisher at the same time.

**Manual takeover during navigation:**

- Connect the PC joystick and leave L1 released initially.
- Hold **L1**: the current goal is canceled and joystick takes priority.
- Left stick drives; right stick turns, limited to 0.20 m/s and 0.60 rad/s.
  Turbo is disabled during takeover.
- Release **L1**: motion stops. The old goal never resumes automatically.
- Wait for cancellation to finish, then send a **new Nav2 Goal** to resume autonomy.
  Goals sent while L1 is held are canceled; send another after release.
- If the joystick disconnects during takeover, motion stops. Reconnect with L1
  released before taking control again.

Check ownership with `ros2 topic echo /robot/control_owner`. Expected states are
`NAVIGATION`, `MANUAL`, and `WAITING_FOR_NEW_GOAL`. Nav2 now publishes to
`/cmd_vel/navigation`; `navigation_override` is the sole `/cmd_vel` publisher.
The robot stays in application mode throughout. Physical Stop/E-stop always applies.
This feature passed isolated software tests; perform a supervised raised-wheel
check before relying on takeover during floor navigation.

**If Nav2 plans but the robot does not move:** cancel the goal, press/release
**Stop → Reset → Start**, then check:

```bash
ros2 topic echo /robot/state --once --no-daemon
```

Expect `ARMED` before sending a **new** goal. This sequence cleared the observed
`HARDWARE_DISARMED_WAITING_FOR_LOCAL_RELEASE` fault. If `FAULT` persists, inspect
`/robot/safety_reason`; do not repeatedly resend goals.

## 6. Stop the session

Cancel navigation / release L1, press physical **Stop**, then Ctrl+C the PC launch.
The robot stays powered with sensors running. For maintenance only, stop its runtime:

```bash
ssh -t vmx@192.168.1.173 sudo systemctl stop studica-classroom.service
```

For a complete Pi shutdown after motion has stopped:

```bash
ssh -t vmx@192.168.1.173 sudo poweroff
```

Wait for shutdown before removing power.

## 7. Simulation — PC only

Create the isolated simulation environment once if it is missing:

```bash
mkdir -p ~/.ros
cat > ~/.ros/studica_sim.xml <<'XML'
<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="any">
<General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General>
<Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
</Domain></CycloneDDS>
XML
cat > ~/.ros/studica_sim.env <<'ENV'
export ROS_DOMAIN_ID=80 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/.ros/studica_sim.xml"
ENV
```

In every simulation terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/setup.bash
source ~/.ros/studica_sim.env
```

Choose **one** launch:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py
# Or: ros2 launch studica_vmxpi_ros2 mapping.launch.py
# Or: ros2 launch studica_vmxpi_ros2 navigation.launch.py
```

In another simulation terminal, arm:

```bash
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
```

Drive/mapping use L1 and the sticks. Navigation uses RViz goals. Never use the
simulation arm service or simulated clock for the real robot.

## 8. Build the code

**PC, existing workspace:**

```bash
source /opt/ros/humble/setup.bash
cd ~/studica_ws
colcon build --symlink-install --packages-up-to studica_vmxpi_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select studica_vmxpi_ros2 --return-code-on-test-failure
colcon test-result --verbose
```

Fresh PC setup: [Installation](INSTALL.md).

**Robot:** follow [Direct native build](NATIVE_BUILD.md)—one worker, no Docker,
separate build directory. Do not compile while driving or overwrite the tested
installation. Never copy PC x86 binaries to the ARM64 Pi.

The tested Pi prefixes are `~/studica-direct-build-20260929/install` plus
`~/studica-pilot-overlay-20260929/install` (monitor fix). Keep both; the original
native archive excludes the overlay. The monitor revision is pinned in `dependencies/hardware.repos`.

## Quick troubleshooting

| Symptom | Action |
|---|---|
| Missing robot TF/odometry | Check `ssh vmx@192.168.1.173 systemctl status studica-classroom.service`; do not start a second hardware launch. |
| RViz drops scans / missing `odom` | Check hardware runtime and TF; do not just increase queue size. |
| Stale `map → odom` | Cancel goal; run `ros2 run tf2_ros tf2_echo map odom`. Timestamps must advance. |
| No topics on PC | Check domain, Wi-Fi addresses, DDS peers and firewall; use `ros2 topic list --no-daemon`. |
| `FAULT`, wheels stationary | Cancel goal → physical Stop → Reset → Start; inspect safety reason if persistent. |
| RViz window freezes | Press physical Stop; restart RViz. Software rendering (`LIBGL_ALWAYS_SOFTWARE=1`) and disabling unused displays restored the tested session; the underlying cause is not confirmed. |
| Joystick reconnects but stays neutral | Stop/restart the single PC launch; release L1 before enabling again. |

Useful read-only checks in a prepared PC terminal:

```bash
ros2 topic echo /robot/safety_reason --once --qos-durability transient_local --no-daemon
ros2 topic echo /robot_status/motors --once --no-daemon
ros2 topic hz /scan
ros2 topic hz /odom
```

For support, send the launch command, error text, robot state and whether the issue
occurs in simulation or hardware. More detail: [Networking](NETWORKING.md),
[Joystick](JOYSTICK.md), [Mapping/Nav2](MAPPING_NAVIGATION.md),
[Camera](CAMERA_POINT_CLOUD.md), [Production acceptance](SAFETY_ACCEPTANCE.md).

Maintenance recovery (SSH; not needed for normal mode changes):

```bash
sudo systemctl restart studica-classroom.service
journalctl -u studica-classroom.service -n 100 --no-pager
# Disable boot runtime if reverting to manual testing:
sudo systemctl disable --now studica-classroom.service
```

Restart only while stationary with E-stop pressed. The service has bounded
restarts and requires the local physical enable sequence again. Do not enable the
old training LiDAR service alongside it; the classroom runtime owns LiDAR.
