# Independent classroom robot sessions

Use Ubuntu 22.04 / ROS 2 Humble for this release. The Pi owns drivers, local
safety, command expiry, sensor acquisition, odometry and robot TF. Keep the
25 Hz hardware default until measured. The PC owns Gazebo, SLAM, Nav2, vision,
RViz and student applications. The managed platform is described in
[Production platform](PRODUCTION_PLATFORM.md).

## Provision each robot once

Assign distinct device identities, DHCP reservations and ROS domains, for
example robot01=11, robot02=12, simulation=80. Provision the network profile
outside ROS; a domain is isolation, not authentication. The installer accepts
`--domain-id 11` with its existing provisioning arguments. This also selects
DDS firewall ports: `7410 + 250 * domain` through that value plus 65.
Use `audit_vmxpi_runtime.py --domain-id 11` for a corresponding runtime audit.

Identity, pairing, certificates and calibration live outside releases in
`/etc/studica` and `/var/lib/studica`. Installation preserves existing config;
review new defaults when upgrading. Hardware profiles are initially copied to
`/etc/studica/profiles/stack_4wd`; the hardware runtime uses that persistent
profile. Each newly provisioned robot uses its unique `studica-<id>.local`
hostname. Record that name and certificate with the classroom inventory.

Services start local control without a PC. DDS includes loopback and optional
Ethernet/Wi-Fi interfaces, allowing boot with no network. If a network interface
appears after DDS startup, a disarmed platform restart may be required for
rediscovery. Do not change domain, calibration or network while operating.
Autostart remains subject to the existing hardware qualification gate: software
tests do not certify cold boot or independent physical torque removal.

## Pair independent PC companions

Repeat pairing for each robot with its actual URL, certificate and assigned
domain (see [Companion](COMPANION.md)):

```bash
ros2 run studica_vmxpi_ros2 install_companion.py \
  --robot-id robot01 --domain-id 11 \
  --robot-url https://studica-DEVICE.local \
  --robot-certificate /secure/input/robot01.crt --pairing-code 12345678
```

Named installs use separate `~/.config/studica/robots/<id>` credentials and DDS
profiles, `~/.cache/studica/<id>/maps`, and `studica-companion-<id>.service`
user services. Pairing checks the robot domain before accepting credentials.
The robot allowlists the paired PC address only in permitted modes.

## Select targets on the PC

Copy `deployment/sessions.example.json` to `~/.config/studica/sessions.json`.
Replace URLs and keep only your provisioned sessions. Copy the installed
`bringup/config/network/cyclonedds_sim.xml` to `~/.ros/studica_sim.xml`.
The CLI validates distinct domains and loopback-only simulation configuration.
Source ROS and the workspace once, then use:

```bash
ros2 run studica_vmxpi_ros2 studica sim lab launch
ros2 run studica_vmxpi_ros2 studica robot robot01 status
ros2 run studica_vmxpi_ros2 studica robot robot01 doctor
ros2 run studica_vmxpi_ros2 studica sim lab run -- ros2 run my_course my_node
ros2 run studica_vmxpi_ros2 studica robot robot01 run -- ros2 run my_course my_node
```

Each process receives its own domain and DDS environment. Simulation receives
its own Gazebo partition and simulated clock; physical applications use wall
time. The ROS executable adapter maps student `/cmd_vel` onto the managed
`/robot/control/developer` input. Enable Developer Mode through the authenticated
UI and use local enable before driving. `doctor` also needs permitted DDS access.
A custom ROS launch must expose and propagate `use_sim_time` and
`cmd_vel_topic`; generic commands receive only the environment.

Hardware `mapping`, `navigation MAP_ID` and `update VERSION` use the existing
HTTPS API and require a separately provisioned private `admin_token_file`.
The paired companion token can read status but cannot authorize those actions.
Use `chmod 600` on token files. Certificate validation is mandatory and redirects
are refused. `update` activates an already staged signed release; downloading
is handled by the managed update service. `--dry-run` previews the command or
API operation without executing it. Direct hardware `launch` is rejected:
managed hardware starts at power-on.

Command loss after a live session inhibits motion and requests disarm. A
returning publisher cannot clear that inhibition; the operator must re-enable.
During release activation a maintenance lock rejects mode changes and inhibits
commands until health verification or rollback completes. Update services load
the ROS environment explicitly, including when launched from a clean boot.

## Release and classroom acceptance

Build pinned ARM64 artifacts off the robot. Keep development bundles immutable.
`sign_release_manifest.py --qualification REPORT.json` can bind the existing
hardware acceptance report to the exact artifact SHA-256 in the signed envelope.
The report requires schema 1, `tested_release_sha256`, at least 50
`cold_boots_passed`, and true `independent_torque_removal`,
`failure_injection_passed` and `zero_motion_all_boots`. Without this signed
qualification, development bundles remain non-activatable. Do not invent
acceptance results to enable deployment.

Activation requires fresh single-source IDLE/READY_DISARMED status. Extraction
checks the signed snapshot, archive boundaries and immutable destination;
activation and boot recovery serialize through a lock. Release contents,
pointers and journals are synced before proceeding. Failed candidate health
checks restore the previous pointer. Recovery retains its journal if stopping
or restoring fails. An existing version directory is refused: use a new release
version instead of overwriting a previous installation.

Before pilot rollout, record the exact artifact digest and verify cold boot
without network, local enable, PC loss, publisher loss, restart, network loss,
and power interruption during activation. Run two robots plus one simulation
and confirm no command, TF or clock crossover. Run a 24-hour sensor workload
and record CPU, memory, temperature, disk and network headroom. Automated
loopback domain tests support these checks but do not replace physical tests.
Roll out to one pilot before the class. Fleets, digital twins and OS upgrades
are outside this release.

See [implementation validation](PRODUCTION_IMPLEMENTATION_VALIDATION.md) for PC
test evidence and the remaining physical qualification boundary.
