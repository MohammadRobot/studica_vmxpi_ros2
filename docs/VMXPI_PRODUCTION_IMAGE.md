# VMXPi production image and runtime

This document defines the minimal operating-system target for the commercial
robot. It separates the production appliance from the current classroom and
development image. It does not authorize motor motion or enable ROS at boot.

## Current measured baseline

The VMXPi at `192.168.1.173` was inspected read-only on 2026-08-29 with the
E-stop pressed, Titan motor power disconnected, Start released, and no motion.
The acceptance ROS graph and the ROS CLI daemon were stopped before the idle
measurements.

| Measurement | Observed result | Assessment |
|---|---:|---|
| OS | Ubuntu 22.04.5, arm64, Linux 5.15 Raspberry Pi kernel | Selected Phase-1 platform |
| CPU | Four cores; five one-second samples were 100% idle | No idle CPU problem |
| Load | `0.00, 0.32, 1.02` after builds and hardware tests | Falling normally |
| Memory | 214 MiB used; 3.3 GiB available of 3.7 GiB | Large idle margin |
| Swap | 0 B used of 4 GiB | Healthy |
| Temperature | Approximately 42 C | Healthy idle temperature |
| Root storage | 30 GiB used of 59 GiB, 53% | Too large for a controlled appliance image |
| Journal storage | 808.3 MiB | Must be bounded and rotated |
| systemd state | `degraded` | Failed crash-upload unit must be removed or repaired |

The system is not compute-bound. Its production problems are unnecessary
packages, unnecessary network exposure, uncontrolled background maintenance,
and the absence of an immutable product release.

The externally reachable TCP listeners were SSH on port 22 and XRDP on port
3389. Port 3389 is not part of the product API and must not be present in a
production image. The XRDP session manager also listened on loopback port 3350,
and UFW reported `ENABLED=no` even though its systemd unit was active.

The following non-runtime services were active:

- XRDP and XRDP session manager;
- firmware update and PackageKit agents;
- UDisks, UPower, and real-time desktop/audio support;
- Apport and kernel crash reporting, with `apport-autoreport.service` failed;
- the desktop-triggered PipeWire user session;
- distribution unattended-upgrade and desktop notification timers.

The installed image also contains Ubuntu Desktop metapackages, GDM, Xorg,
printing, audio, Snap, XRDP, ROS desktop, and GUI tooling. Disabling processes
reduces runtime activity, but a production image must also omit these packages
so they do not add update load and attack surface.

## Read-only audit

Run the checked-in auditor before and after every image change:

```bash
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/audit_vmxpi_runtime.py
```

The command does not call `sudo`, change a file, start ROS, or send a motion
command. A non-zero result means the versioned profile found a production
deviation. JSON output is suitable for a support bundle:

```bash
./scripts/audit_vmxpi_runtime.py --json > vmxpi-runtime-audit.json
```

After systemd robot units exist, require the exact release units explicitly:

```bash
./scripts/audit_vmxpi_runtime.py \
  --require-unit studica-hal.service \
  --require-unit studica-robot.service \
  --require-unit studica-monitor.service
```

The policy is `deployment/vmxpi-production-v1.json`. It targets the selected
Ubuntu 22.04 arm64 and ROS 2 Humble Phase-1 platform. A product variant or a
future platform migration must use a new explicit, versioned policy;
exceptions passed on the command line are test tools and are not a production
configuration mechanism.

## Target operating-system services

The product uses the smallest service set that still provides recovery,
networking, Bluetooth joystick support, time, logs, and the robot runtime.

| Role | Required components | Notes |
|---|---|---|
| OS core | systemd, journald, udev, logind, OOM handling | Keep boot and fault evidence |
| Local recovery | one text console/getty | No graphical desktop |
| Network | NetworkManager, `wpa_supplicant`, resolved | Ethernet, infrastructure Wi-Fi, and fallback hotspot |
| Time | `systemd-timesyncd` | Robot must still boot and operate without internet time |
| Host firewall | UFW/nftables policy | Default-deny inbound; explicit product endpoints only |
| Remote support | OpenSSH server | Unique keys, keys-only login, no root login |
| Bluetooth | BlueZ and `hciuart` | Required for the Bluetooth joystick; audio is not required |
| Hardware | `studica-hal.service` | Future single privileged VMXPi HAL owner |
| Robot | `studica-robot.service` | Future non-root controller and safety graph |
| Health | `studica-monitor.service` | Future diagnostics and bounded support data |
| Optional hardware | named sensor units | Enabled only for the installed product variant |
| Updates | `studica-update.timer` and service | Signed, staged, disarmed-only, rollback-capable |

DBus remains because BlueZ, NetworkManager, and systemd integrations use it.
Polkit remains during the first appliance phase while the exact non-root
NetworkManager workflow is qualified. `irqbalance`, filesystem trim, temporary
file cleanup, log rotation, and one console are small reliability services, not
desktop applications.

The following are absent or disabled in the target image:

- GDM, Xorg, Wayland desktops, XRDP, VNC, and desktop portals;
- CUPS and printer discovery;
- PipeWire, PulseAudio, and Bluetooth audio profiles;
- PackageKit, update notifier, motd news, and uncontrolled APT timers;
- UDisks and UPower desktop agents;
- Apport, Whoopsie, and kernel crash upload agents;
- firmware update agents unless a specific VMXPi firmware path is qualified;
- Avahi unless a documented mDNS product requirement is approved;
- Snap unless a product release is deliberately delivered as a confined snap;
- ModemManager unless a supported cellular option is installed;
- the ROS CLI daemon and all build, test, RViz, RQt, Gazebo, and IDE processes.

`cron` and `rsyslog` are retained temporarily in profile v1. The clean image
may remove them after all jobs use systemd timers and journald is the single
bounded logging path. Removing either before that inventory is complete can
silently break maintenance or erase expected log handling.

## ROS runtime package split

The source repository supports both classroom and robot workflows, but its ROS
`package.xml` now declares only the headless robot-core dependency contract.
GUI, Gazebo, Nav2, SLAM, Foxglove, rosbag, CLI, and keyboard tools live in the
explicit `dependencies/apt/` developer bundles consumed by
`scripts/setup_ubuntu.sh`. The classroom installer still requests ROS Desktop;
the production manifest does not.

This dependency split prevents `rosdep` for the core package from silently
pulling desktop tooling into a product image. It does not yet create separate
binary packages: the repository still installs shared launch and documentation
assets, so release packaging must complete the binary and filesystem split.

Create release packages with these responsibilities:

| Package or bundle | Installed on robot | Content |
|---|---|---|
| `studica-robot-core` | Always | HAL, controller, safety supervisor, description, Cyclone DDS runtime |
| `studica-robot-monitor` | Always | Diagnostics, health API, bounded recordings |
| `studica-robot-joystick` | Product option | `joy`, deadman teleop, approved controller mapping |
| `studica-robot-sensors` | Product option | Only drivers for physically installed sensors |
| `studica-robot-navigation` | Product option | Nav2 and localization, without RViz on the robot |
| `studica-robot-development` | Developer PC only | CLI extras, build tools, tests, RViz, RQt, Foxglove tools |
| `studica-robot-simulation` | Developer PC only | Gazebo and simulation dependencies |

The robot starts from the ROS `ros-base` variant and adds only runtime
dependencies. Compilation and `git pull` happen off-robot. Each release is an
immutable, signed artifact installed under `/opt/studica/releases`, as defined
in [Release process](RELEASE_PROCESS.md).

The first machine-readable package boundary is
`deployment/vmxpi-runtime-packages-v1.json`. It selects the minimal Ubuntu
Server base, the headless Humble control runtime, Bluetooth joystick support,
and the currently installed camera and LiDAR options. Foxglove and on-robot
navigation are disabled in this baseline. Desktop, simulation, graphical, and
build packages are explicitly prohibited.

Validate the manifest and print its enabled APT package names on the development
PC with:

```bash
./scripts/validate_production_manifest.py
./scripts/validate_production_manifest.py --print-enabled-apt
```

This manifest defines the desired package boundary. The development release
builder now requires an exact target-root package inventory, a clean pinned
source overlay, and an ARM64 production install; it emits a deterministic
archive, checksums, SPDX SBOM, and rollback metadata. See
[Release process](RELEASE_PROCESS.md#development-arm64-bundle). It does not yet
provide a reproducible operating-system image, repository snapshot, artifact
signature, or authorized activation path.

## Phase-1 platform decision

The selected production baseline for now is Ubuntu 22.04 arm64 with ROS 2
Humble. It matches the current Studica VMXPi SDK and hardware integration and
keeps platform migration risk separate from the safety, packaging, update, and
support work still required to turn the POC into a product. The production
image uses the Humble `ros-base` variant plus explicitly selected runtime
packages; selecting Humble does not justify retaining ROS Desktop or Ubuntu
Desktop on the robot.

This selection is time-bounded. Ubuntu 22.04 standard security maintenance and
ROS 2 Humble support both end in May 2027. See the official
[Ubuntu release cycle](https://ubuntu.com/about/release-cycle) and
[ROS 2 distribution schedule](https://docs.ros.org/en/humble/Releases.html).
Before a commercial release, the product support matrix must define the last
supported Humble release, security-maintenance method, customer upgrade path,
and a migration deadline that occurs before upstream ROS support ends.

Do not introduce Ubuntu 24.04/Jazzy or Ubuntu 26.04/Lyrical into the Phase-1
image. Evaluate a successor only in a separate compatibility branch and only
after the VMXPi SDK, Titan firmware path, camera, LiDAR, Cyclone DDS,
`ros2_control`, update rollback, and the complete hardware-in-the-loop safety
suite pass. The production profile must be versioned again when that migration
is approved.

## Staged image migration

### Stage 0: preserve recovery

1. Clone the working storage and verify that the clone boots.
2. Export installed packages, systemd units, NetworkManager connections, and
   VMXPi SDK/HAL versions without exporting credentials into source control.
3. Provide a local console and a second tested recovery image.
4. Keep the E-stop pressed, Titan power disconnected, and Start released.

No service removal begins without a bootable recovery image. The current robot
must remain a reference POC until the replacement image passes hardware tests.

### Stage 1: reversible service reduction

On a cloned image, disable and mask one group at a time, beginning with XRDP,
desktop/crash agents, desktop storage/power agents, audio, and distribution
auto-update agents. Do not remove packages yet. After each group:

1. cold boot and run the runtime audit;
2. reconnect using a fresh SSH session;
3. verify Ethernet, infrastructure Wi-Fi, hotspot, and DNS behavior;
4. pair, disconnect, and reconnect the approved Bluetooth joystick;
5. run the VMXPi read-only HAL and input checks with Titan power disconnected;
6. inspect failed units, journal errors, memory, temperature, and open ports.

Never disable SSH, NetworkManager, `wpa_supplicant`, BlueZ, `hciuart`, resolved,
or time sync remotely without a tested local-console recovery path.

### Stage 2: clean minimal image

Build a reproducible arm64 image from a server/minimal base rather than
subtracting indefinitely from Ubuntu Desktop. Ubuntu describes Server as a
minimalist base and supports arm64; see its [server installation guide](https://ubuntu.com/server/docs/tutorial/basic-installation/).

The image build must produce:

- a locked package manifest and repository snapshot;
- a unique factory hostname, SSH host key, administrator key, hotspot secret,
  robot identity, and serial record at first boot;
- keys-only SSH with root login disabled;
- a default-deny host firewall and no public listener except approved product
  endpoints;
- bounded persistent journald storage and log-rate limits;
- read-only or integrity-protected system areas where the HAL permits it;
- a software bill of materials, build provenance, checksum, and signature;
- an offline recovery/reflash procedure.

The official Ubuntu package guide recommends `apt-get`, rather than interactive
`apt`, for scripts and warns that direct `dpkg` removal does not resolve
dependencies. Image construction therefore uses an unattended image recipe and
APT dependency resolution, not a copied terminal cleanup log. See
[Ubuntu package management](https://ubuntu.com/server/docs/how-to/software/package-management/).

### Stage 3: systemd robot services

Install versioned systemd units only after the physical hardware gate passes.
Each unit has an explicit user, environment file, working directory, dependency
order, bounded restart policy, start timeout, stop timeout, watchdog, and
resource limits. Apply service sandboxing only after testing VMXPi device,
shared-memory, USB, serial, CAN, GPIO, and network access.

The shutdown acceptance test must prove that controller stop, service restart,
process kill, reboot, and power loss all cause zero target and disabled Titan.
The current charged lifted-wheel failure and the VMX HAL teardown zero-write
race mean this stage is still blocked.

### Stage 4: signed updates

Replace generic unattended upgrades with the product update agent. It may
download while disarmed, but activation occurs only in `READY_DISARMED` during
a maintenance window. It verifies signatures and compatibility, atomically
switches releases, health-checks the new version, and rolls back automatically.

Operating-system security updates still require a defined cadence. Disabling
generic automatic updates without replacing them would create an insecure
product, so the updater and fleet vulnerability process are release blockers.

### Stage 5: qualification

The minimal image must pass at least:

- 50 cold boots with zero motion and deterministic service readiness;
- 24-hour idle and full-sensor CPU, memory, temperature, and storage soak tests;
- power interruption during boot, logging, download, activation, and rollback;
- Ethernet/Wi-Fi/hotspot priority and loss/recovery tests;
- Bluetooth pairing, reconnect, deadman release, and radio-loss tests;
- full hardware safety, lifted-wheel, and low-speed floor acceptance;
- SSH, firewall, SROS2/DDS permissions, and public-port penetration tests;
- factory reset, offline reflash, failed-update recovery, and support-bundle tests.

No result from the current Desktop-derived image substitutes for qualification
of the final production image.

## Immediate next implementation work

Recovery-clone work is temporarily deferred. Until it resumes, do not harden
the live image, remove live packages, or install systemd robot units. The
bootable clone remains a release requirement before any of those operations.

1. Build the first development ARM64 application artifact in a clean hardware
   build environment and archive its inventory, checksums, and SBOM.
2. Turn the Ubuntu 22.04/Humble package manifest into a reproducible operating-
   system image recipe with locked binary repositories.
3. Diagnose and pass the charged lifted-wheel tracking failure.
4. Implement and test the systemd service chain and safe shutdown behavior in
   an offline fixture; do not activate it on the robot before the hardware gate.
5. Implement signed atomic application updates and rollback.
6. Resume the bootable clone and audit archive before changing the live image.

This order keeps image optimization independent of motor authorization while
both streams progress toward the same production release gate.
