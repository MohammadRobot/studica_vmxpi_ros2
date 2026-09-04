# Studica production robot platform

This package implements the local software side of the managed robot appliance. It does not authorize deployment to physical hardware. Production autostart remains blocked until the physical E-stop removes motor torque independently of VMXPi software and the release passes the hardware qualification in `SAFETY_ACCEPTANCE.md`.

## Runtime boundary

```text
joystick ──> /robot/control/joystick ┐
browser  ──> /robot/control/web      ├─> mode manager ─> /robot/platform/cmd_vel
Nav2     ──> /cmd_vel                │                         │
developer──> /robot/control/developer┘                         v
                                                  safety supervisor
                                                           │
                                                           v
                                            /robot_base_controller/cmd_vel
```

The mode manager is the only publisher of `/robot/platform/cmd_vel`. The existing hardware safety supervisor remains the only process that can forward commands to the drive controller. Both layers enforce a 250 ms receive timeout. Manual joystick modes also require the joystick deadman. All motion still requires a fresh physical Start press.

Boot produces `IDLE` and `READY_DISARMED`. Neither mode nor armed state is persisted. A mode transition publishes zero, clears every cached command, requests `/robot/disarm`, waits for its acknowledgement, then validates the selected workload. The companion does not start SLAM or Nav2 during `WAITING_FOR_DISARM`. When an autonomy workload becomes ready, the manager performs a final acknowledged disarm before reporting `READY`; therefore a Start press made while SLAM or Nav2 was loading cannot authorize motion.

## Services

`studica-robot.target` coordinates these bounded-restart units:

- `studica-hardware.service`: root-isolated VMX/Titan process, safety supervisor, odometry, TF and diagnostics.
- `studica-mode-manager.service`: non-root lifecycle manager and command arbiter.
- `studica-lidar.service`: starts at boot.
- `studica-camera.service`: on-demand only.
- `studica-joystick.service`: direct BlueZ/evdev joystick input.
- `studica-web.service`: authenticated HTTPS UI and API.
- `studica-orchestrator.service`: root-only Unix-socket allowlist for sensors, network, Bluetooth, support and update activation.
- `studica-peer-apply.timer`: one-shot delayed reload after a safely authorized companion pairing response.
- `studica-update.timer`: signed background update checks when a publisher key is provisioned.
- `studica-update-recovery.service`: boot-time rollback of any activation interrupted before its health check committed.

Camera activation requires at least 20% available memory and no more than 80% normalized one-minute compute load; its unit is also capped at 120% CPU and 700 MiB. The hardware loop remains outside that cgroup.

The installer creates unique identity, TLS material, API token and fallback hotspot state below `/var/lib/studica`; configuration lives under `/etc/studica`; immutable releases live under `/opt/studica/releases` with `/opt/studica/current` as the atomic pointer. The fixed firewall is rebuilt on boot, companion presence travels over scoped HTTPS, and both incoming and outgoing DDS traffic to the paired address remain blocked until autonomy or explicit Developer Mode owns the system. It never runs `git pull`.

## Local build

```bash
cd /home/mohammadrobot/studica_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select studica_vmxpi_ros2 --symlink-install
source install/setup.bash
colcon test --base-paths src --packages-select studica_vmxpi_ros2
colcon test-result --verbose
```

Use an offline temporary root to inspect installation without changing this computer or the robot:

```bash
stage=$(mktemp -d)
mkdir -p "$stage/etc"
ros2 run studica_vmxpi_ros2 install_robot_platform.py \
  --root "$stage" \
  --assets-root /home/mohammadrobot/studica_ws/install/studica_vmxpi_ros2/share/studica_vmxpi_ros2/deployment
```

Do not pass `--enable-autostart` until the exact ARM64 release has a complete safety qualification record.
