# Platform setup deployment — 2026-09-07

The later [camera and HTTPS observation deployment](OBSERVER_CAMERA_DEPLOYMENT_2026-09-07.md)
adds tested on-demand depth streaming, an enabled read-only HTTPS observer,
`robot.local` advertisement and the validated `office_nav` registry entry.
It supersedes the web/camera inventory below but does not activate motor control.

## Latest state — successful SSH retry, 21:00 GST

SSH access recovered using a persistent connection. Transferred and verified
the ROS environment-loading fix, installed the corrected sensor wrapper, and
successfully tested the isolated **X2** service. The hardware wrapper fix was
copied to the setup-source directory only; no hardware runtime was executed
and no staged production release was modified.

Two sensor checks passed, including one after a clean systemd stop/restart:

| Check | First pass | After service restart |
|---|---:|---:|
| Scan rate before scanner stop | 11.181 Hz | 11.458 Hz |
| Scan rate after scanner restart | 11.244 Hz | 11.448 Hz |
| Scans during stopped observation | 0 | 0 |
| Scanner stop response | 0.703 s | 0.701 s |
| Scanner start response | 7.609 s | 7.611 s |
| Motion publishers in isolated domain 78 | 0 | 0 |

Both checks validated increasing timestamps, `laser_scan_frame`, 270 points
per scan and valid ranges. The initial three-second scanner-start test timed
out because the SDK performs about seven seconds of intensity detection. The
probe now allows 15 seconds specifically for `/start_scan`; the scanner-stop
bound remains three seconds. Neither value is a drive-command lease or an
emergency-stop measurement.

The service stop returned `Result=success`, `MainPID=0`, released the serial
port, and left no matching LiDAR or hardware control process. The final unit
now waits for `dev-ttyUSB0.device`, has a 30-second startup job bound and stops
if that device disappears. Automatic recovery after unplug/replug has not
been validated.

At `2026-09-07T21:00:39+04:00`, `studica-training-lidar.service` was **enabled
and running**, with zero automatic restarts and 35,512,320 bytes (about 34 MiB)
of cgroup memory. Approximately 3.3 GiB system memory remained available.
Only this stationary, loopback-only LiDAR service is enabled for boot. The
robot target, hardware and web services remained disabled/inactive and
`/opt/studica/current` remained absent. No reboot, motor command or production
activation was performed. A service restart is not a verified cold boot.

Verified deployed SHA-256 values:

```text
sensor wrapper: f899d3583c3e0e3a24a8da549ca64006470e1e98e8e93c5cbffa0bec97fa91cc
training unit:  663c0d5a4710b124d6fdf0fb374152dddd4145ed739d57c9759311136ab3fc7f
```

Machine-readable measurements are preserved in the workstation workspace at
`robot_test_results/physical_lidar_20260907/deployment_retry.json`.

The SDK still emits the previously observed baseplate-information warning and
logs failed-scan messages while intentionally stopped through `/stop_scan`.
The measured scan checks pass, but these are not claims of warning-free driver
operation. Camera, web integration, student access and cold-boot verification
remain separate work; the full robot control system is not activated.

## Result: partial installation, not an activated appliance

The operator requested completion and deployment. Installed the prepared
platform files without enabling the motion-capable target or choosing an
active release. The existing DIO8-only E-stop and deferred main-switch
assessment were not marked as qualified. No motor command was issued.

## Installed on VMXPi

- Setup source: `/home/vmx/studica-platform-setup-20260907`.
- Prepared offline root: that directory's `prepared-root` subdirectory.
- Platform systemd units under `/etc/systemd/system`: hardware, mode manager,
  orchestrator, web, joystick, sensors, update services/timers and robot target.
- Configuration under `/etc/studica`.
- Dedicated `studica` and `studica-update` accounts; per-device identity, TLS
  material, private API token and persistent state below `/var/lib/studica`.
- Bounded journald configuration; no journald restart was performed.
- Separate `studica-training-lidar.service` and root-owned sensor launcher at
  `/opt/studica/training-20260907/studica_sensor_runtime`.

The scoped helper `deployment/training/install_prepared_setup.sh` applies an
offline preparation without installing its hostname, NetworkManager hotspot or
SSH policy changes. Existing network and SSH access configuration were left
unchanged; live hostname remained `vmx`. This is not a completed first-boot
network setup or a working `robot.local` endpoint. The prepared provisioning
metadata describes the intended endpoint, not a verified live one.

The source archive transferred successfully with SHA-256:

```text
84e32fc7d7eace741ea9a88b7053027753d3c5bb4d0498c81b25cfed4962e6d0
```

This is a setup-source archive, not a signed ARM64 release. The later local
ROS environment-loading fix is **not** included in that original archive.
It and the sensor acceptance probe were transferred separately during the
successful retry described above.

## LiDAR service test

The separate service uses the existing development workspace for ROS/driver
dependencies and explicitly selects X2. It is a stationary training service,
not a production release. Its systemd device policy is `closed`, allowing
only `/dev/ttyUSB0` in addition to systemd's standard pseudo-devices, with no
capabilities, no motor runtime dependencies, loopback-only Cyclone DDS and ROS
domain 78. Its resource limits are 100% of one CPU and 384 MiB memory.

The first start failed before scanner initialization:

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

At 20:38 GST, the bounded restart limit stopped retries. The last verified
service state was `failed`, `MainPID=0`, `NRestarts=3`. The sensor probe failed
because it found no scan publisher. This is **not** a passing scan test.

The local sensor and hardware wrappers now suspend Bash `nounset` only while
loading ROS setup files and restore it afterward; `errexit` stays enabled.
Regression tests cover unset optional setup variables and aborting when setup
fails. Forty local sensor/training/deployment/model tests passed. The existing
web authentication/WebSocket security test also passed with loopback socket
access (its first sandboxed attempt was blocked from creating a socket).
No hardware runtime was executed to test its wrapper.

## Initial connection interruption and state before retry

Transfer attempts for the environment fix and a bounded persistent SSH retry
timed out connecting to TCP port 22. Ping still returned 3/3 replies. The client
route was from `192.168.2.118` via `192.168.2.1` to VMXPi `192.168.1.173`.
The earlier [SSH connectivity audit](SSH_CONNECTIVITY_AUDIT_2026-08-31.md)
documents similar cross-subnet behavior; its historical diagnosis is not proof
of the current cause.

- `studica-robot.target`: installed, disabled and inactive at verification.
- Production services/timers: not started or enabled.
- `/opt/studica/current`: still absent at verification.
- Existing staged ARM64 release and activation guards: unchanged.
- Training LiDAR: installed, **not enabled at boot**, failed startup test.
- Web control, camera, joystick, SLAM/Nav2 and signed updates: not activated.
- No reboot or hardware/motor test was performed during this deployment.

## Original recovery checklist (steps 1–4 completed in retry)

1. Recheck robot services, release pointer and current safety/operator state.
2. Transfer and verify the corrected local sensor wrapper; install it over the
   training copy only, leaving the guarded release unchanged.
3. Reset the failed training service and start it, then run the bounded
   `scripts/verify_stationary_lidar.py` in loopback ROS domain 78.
4. Only after valid scan data and scanner stop/start pass, consider enabling
   **the training LiDAR service alone** at boot. Validate a service restart and
   record resource use; a restart is not a cold-boot test.
5. Continue camera and UI integration separately. Do not start the existing
   production web unit as a standalone UI: its dependency chain starts the
   hardware service. Motion-capable autostart remains unqualified.
