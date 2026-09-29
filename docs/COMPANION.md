# Ubuntu companion

The companion runs SLAM Toolbox and Nav2 on Ubuntu 22.04/ROS 2 Humble while the VMXPi retains motion safety, manual driving, LiDAR, diagnostics and the web API. Loss or crash of the companion changes autonomy to `IDLE`, requests disarm and leaves manual modes available.

Install ROS 2 Humble desktop plus `ros-humble-nav2-bringup`, `ros-humble-nav2-map-server`, `ros-humble-nav2-msgs`, `ros-humble-rmw-cyclonedds-cpp`, `ros-humble-slam-toolbox`, and this workspace on the companion. The paired PC is part of the trusted local control boundary: its scoped HTTPS credential cannot drive, but its allowlisted DDS participant supplies SLAM/Nav2 while an autonomy mode is active.

Install the workspace on the PC and copy the robot's public TLS certificate over a trusted local channel. In the authenticated web UI, leave the robot `IDLE/READY_DISARMED`, select **Create pairing code**, then run this command before the ten-minute code expires:

```bash
source /opt/ros/humble/setup.bash
source /home/$USER/studica_ws/install/setup.bash
ros2 run studica_vmxpi_ros2 install_companion.py \
  --robot-id robot01 --domain-id 11 \
  --robot-url https://studica-DEVICE.local \
  --pairing-code 12345678 \
  --robot-certificate /secure/input/robot.crt
```

The code works once. The installer exchanges it over certificate-verified HTTPS for a hash-only, companion-scoped credential and installs a restricted Cyclone DDS peer configuration. The credential can read status, report readiness, download maps and upload completed maps; it cannot change modes, drive, pair devices, activate updates or enable Developer Mode. The redeeming address becomes the robot's allowlisted DDS peer, and re-pairing revokes the previous companion credential. Pairing schedules a safe robot-platform restart after the HTTPS response so the new peer configuration is loaded. Re-pair if the companion's address changes. The administrative `--token-file` option remains available only as a recovery path and requires the peer to be provisioned separately.

Named installs create private token, CA and DDS files below `~/.config/studica/robots/<id>`, a map cache below `~/.cache/studica/<id>/maps`, and enable `studica-companion-<id>.service`. Match the domain to the provisioned robot. Repeat with another ID and domain for each robot. Omitting `--robot-id` retains the legacy singleton layout. See [Product runtime](PRODUCT_RUNTIME.md). Use `--no-enable` for inspection-only installation.

The daemon exchanges status and readiness through the scoped HTTPS API, so discovery does not require DDS to be exposed at idle. The robot opens DDS only to the paired address while SLAM, Navigation or explicit Developer Mode owns the system, and removes that exception on mode exit, companion loss, process restart and boot.

When `SLAM` is selected, the daemon starts the hardware mapping launch without a second joystick. Saving a map runs `nav2_map_server/map_saver_cli`, validates the YAML/image pair locally and uploads it to the robot registry. When `NAVIGATION` is selected, it downloads the selected immutable map and starts Nav2. Nav2's single final `/cmd_vel` publisher is consumed by the robot mode manager; behavior-server recovery commands are remapped through the velocity smoother.
