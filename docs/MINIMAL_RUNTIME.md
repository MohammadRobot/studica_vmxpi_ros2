# Minimal runtime candidate

Prepared locally on 2026-11-10. **Not installed, activated or qualified.** Existing
classroom and managed-runtime records remain historical evidence. Do not enable
this service alongside them. Normal PC drive/SLAM/navigation uses the same
application command input and requires no SSH mode switching.

## Files and ownership

| Candidate file | Reviewed destination / purpose |
|---|---|
| `deployment/minimal/minimal.example.json` | `/etc/studica/minimal.json`: device identity, domain, profile and sensor selection |
| `deployment/minimal/minimal.env.example` | `/etc/studica/minimal.env`: matching domain and DDS environment |
| `deployment/minimal/studica-minimal.service` | Hardware runtime, bounded restarts, no dependency on network-online |
| `deployment/minimal/studica-minimal-recovery.service` | Recover interrupted activation before hardware startup |
| `deployment/minimal/studica-minimal-activate@.service` | Explicit instructor-triggered update candidate |
| `/etc/studica/profiles` | Persistent calibrated profiles outside replaceable releases |
| `/opt/studica/releases/<version>` | Immutable, verified ARM64 releases |
| `/opt/studica/current` | Atomic selected-release symlink |
| `/opt/studica/maintenance/install` | Separately provisioned known-good ARM64 recovery environment |
| `/var/lib/studica/updates` | Persistent update journal and known-good history |

Inspect configuration on the PC without starting ROS:

```bash
cd ~/studica_ws/src/studica_vmxpi_ros2
python3 -m studica_robot_platform.minimal_runtime --check \
  --config deployment/minimal/minimal.example.json
```

The hardware launcher requires ARM64, explicit Humble and matching domain settings.
It selects `stack_4wd`, 25 Hz, LiDAR on, camera off, no local joystick. The Pi does
not launch Gazebo, RViz, SLAM or Nav2. Calibration must be provisioned before first
startup; the example identity/domain is not a classroom allocation scheme.

Critical controller, supervisor, robot-state-publisher or odometry process exits
shut down the complete session. systemd restarts at most three times per 120 s,
with a five-second delay. Startup and recovery require a fresh local enable.
The HAL currently requires root; this privilege is limited to the robot hardware
unit and update operation. PC applications and observers run as ordinary users.
Do not claim the HAL can run unprivileged without device-permission qualification.

The unit limits journal message rate. Candidate logrotate and tmpfiles policies
under `deployment/minimal` rotate ROS logs at 10 MiB (checked by the host's rotation
schedule), retain two archives and expire inactive logs after seven days. Review
host journal retention and install these policies during commissioning; rotation
is periodic, not a hard disk quota. They have not been applied here.

## Readiness and update gates

Once a candidate is separately authorized and installed, its read-only probe is:

```bash
ros2 run studica_vmxpi_ros2 studica_minimal_status.py --timeout 10
```

It requires repeated fresh `READY_DISARMED`, unique publishers, healthy four-wheel
feedback, stationary targets/velocities/odometry and the supervisor's effective
hardware/input/deadline/maintenance parameters. It rejects cached transient
history. Receive times use a monotonic clock; parameter freshness includes request
latency. This is runtime evidence, not physical qualification or a stopping test.

The update agent retains managed-runtime compatibility and adds `--runtime minimal`.
Activation verifies the signed staged release, requires disarmed stationary
evidence, creates `/var/lib/studica/activation-in-progress`, stops the entire
runtime, switches the release, and checks the replacement. The supervisor treats
the lock, a symlink, or a lookup error as an inhibit and clears buffered commands.
Failed health checks restore the previous release; failures to stop/restore leave
the lock and journal for recovery. Recovery refuses an active runtime. Successful
lock removal does not resume earlier commands: local release/enable is required.

The first migration from a legacy runtime must be performed as a separate
stationary maintenance procedure. The online gate intentionally rejects a runtime
without the maintenance-lock parameter. Development bundles remain marked
`DO_NOT_ACTIVATE`; this work does not promote them to signed production releases.

## Independent recovery environment

The candidate recovery unit sources `/opt/studica/maintenance/install/setup.bash`,
not the selected application release. Provision a separately verified native
ARM64 maintenance install with the update agent, its Python modules and ROS message
libraries before enabling the unit. Keep it outside the current-release switch
and update it only through a separately reviewed maintenance procedure. It must
support the activation journal schema used by the application release.

A missing maintenance prefix fails startup; there is no fallback to the broken
candidate. The host Humble underlay and OS remain dependencies, so this is not
recovery from OS/storage failure. Test the actual clean-image unit and corrupted
candidate scenarios before claiming boot recovery is qualified.

Recovery refuses an active or unknown runtime state, symlink locks, and locks
without a matching journal transaction ID. Unexplained or legacy empty locks remain
inhibited for investigation; do not delete them merely to make startup pass.

New activations write and sync a transaction-bound maintenance lock before service
changes. Finalization writes history, then atomically replaces `activation.json`
with a terminal completion receipt. Cleanup validates the receipt, current release
pointer and lock identity, removes the lock and syncs its parent directory, then
removes the receipt and syncs the update-state directory. An interrupted cleanup
can be retried even if the lock is already absent. A committed update is not rolled
back because cleanup fails. Recovery starts no services and requires the runtime
to be stopped; normal boot still requires fresh local enable.

The separately provisioned maintenance runtime must understand this extended
journal protocol before deployment. Older maintenance code rejects terminal
journals. Local process-exit and filesystem-error fixtures cover this protocol;
actual target power-loss and systemd qualification remain pending.

## Before deployment approval

Review the exact diff and dependency pins; build and test on native ARM64; verify
release signatures, SDK/library hashes and installed effective parameters. Keep
the current dated installation and a recovery path. Then separately commission
offline cold boot, no automatic motion/resumption, update rollback/interruption,
PC/network loss and 24-hour resource headroom. The software-only E-stop and Titan
watchdog uncertainty remain unresolved; this candidate cannot establish safe
RoboHiTec physical control.
