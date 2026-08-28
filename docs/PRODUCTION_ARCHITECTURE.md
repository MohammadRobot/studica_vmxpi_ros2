# Production architecture target

This document defines the production target. It is not a claim that the current
robot implements these controls. A feature must not be described as production
ready until its acceptance tests are automated and have passed on VMXPi
hardware.

## Implementation status

Phase 1 now implements the state-machine core and a single-owner command
supervisor in all runtime launches. Automated tests prove boot-disarmed output,
explicit simulation arm/disarm, monotonic expiry, publisher loss, source
conflict rejection, planar/finite validation, and speed/acceleration limits.
See [Safety supervisor API](SAFETY_SUPERVISOR.md).

Phase 2 integrates the fail-closed local enable and E-stop status gate into the
VMX/Titan write boundary. It disables Titan and forces zero unless two valid
active-low DIO samples, drive health, the boot-release sequence, and a new local
enable edge all agree. Exact FlexDIO channels remain intentionally set to `-1`,
so physical hardware startup is blocked pending wiring inspection and lifted
acceptance. See [Physical hardware safety gate](HARDWARE_SAFETY_GATE.md).

The hardware-only monitor now publishes a fail-closed
`Robot/Control/HardwareSafety` diagnostic from the exported state. Supervisor
hardware-state mirroring, source-specific joystick/Nav2/remote arbitration,
authenticated leases, systemd activation, and atomic updates remain gated
future work. This revision is not deployed to the physical VMX-pi.

## Safety invariants

1. Boot never authorizes motion. A healthy boot ends in `READY_DISARMED`.
2. Exactly one safety supervisor owns the controller command output.
3. Missing, expired, malformed, unauthorized, or conflicting commands produce
   zero velocity.
4. A physical emergency stop remains effective without ROS, Linux, or network.
5. Updates, configuration changes, and controller restarts are forbidden while
   armed.
6. Network or Bluetooth loss cannot leave a non-zero command active.
7. A critical drive fault is latched until the operator acknowledges it from a
   safe state.

## Robot state model

| State | Motion output | Allowed transitions |
|---|---|---|
| `BOOTING` | Disabled | `READY_DISARMED`, `FAULT` |
| `READY_DISARMED` | Disabled | `ARMED`, `UPDATING`, `FAULT`, `SHUTTING_DOWN` |
| `ARMED` | Safety-supervised | `READY_DISARMED`, `FAULT`, `SHUTTING_DOWN` |
| `FAULT` | Disabled and latched | `READY_DISARMED` after fault clearance and acknowledgement |
| `UPDATING` | Disabled | `READY_DISARMED`, `FAULT` |
| `SHUTTING_DOWN` | Disabled | Power off or reboot |

Arming requires healthy drive diagnostics, a valid emergency-stop input, no
active update, and an explicit local operator action. A network client alone
must not arm an unattended robot in the default product configuration.

## Motion command path

The existing `/cmd_vel` `geometry_msgs/msg/Twist` interface remains the simple
local application input. Other producers use distinct inputs:

| Source | Proposed input | Required guard |
|---|---|---|
| Local application | `/cmd_vel` | One application publisher and receive timeout |
| Bluetooth joystick | `/cmd_vel/joy` | Held deadman button and hot-plug detection |
| Nav2 | `/cmd_vel/nav` | Active autonomous mode and cancellable goal |
| Remote PC | `/cmd_vel/remote` | Authenticated operator lease and stamped command |

The supervisor publishes a private, stamped controller command. It applies
source priority, a monotonic receive deadline, speed and acceleration limits,
robot state, fault locks, and an operator lease. Local deadman teleoperation
normally pre-empts navigation and remote control. Only the supervisor may be a
publisher on the controller command topic.

Remote commands must carry a source timestamp, sequence number, and lease ID.
The gateway rejects replay, clock-invalid, out-of-order, and expired commands
before they enter the motion graph. The final safety timeout uses the robot's
monotonic receive clock and does not depend on synchronized wall time.

## Process boundary

The target systemd dependency chain is:

```text
studica-hal.service
        |
        v
studica-robot.service ----> sensors and diagnostics
        |
        +----> studica-network-gateway.service

studica-update.timer ----> studica-update.service
```

`studica-hal.service` is the smallest process permitted to access privileged
VMXPi HAL resources. Control, monitoring, sensors, networking, and user tools
run without root. Each service has bounded restart behavior and journald limits.
A critical process reports readiness and watchdog health to systemd; a critical
failure disables Titan before restart.

The core robot graph starts and remains usable without internet, DNS, Wi-Fi, a
remote DDS participant, or a joystick. Optional sensor failure must not obscure
drive safety state.

## Network boundary

Connection priority is Ethernet, configured infrastructure Wi-Fi, then a
fallback per-device hotspot. Factory provisioning creates a unique hostname,
hotspot credential, SSH host key, administrator credential, ROS security
identity, and device serial record.

Native DDS is never exposed to the internet. Product mode provides read-only
telemetry and a narrowly authorized control gateway. Full developer DDS access
is a visible, time-bounded mode. `ROS_DOMAIN_ID` separates classroom groups but
is not an authorization mechanism; SROS2/DDS Security policies provide identity,
permissions, and encryption.

## Update boundary

The robot runs an installed immutable release, not a Git working tree. An update
is downloaded to a staging directory, verified, activated atomically, health
checked, and either committed or rolled back. The update agent can enter
`UPDATING` only from `READY_DISARMED`.

Application releases initially use versioned directories under `/opt/studica`.
Full operating-system updates require a separately qualified A/B image and
bootloader rollback design.

## Required acceptance tests

- Repeated cold boot with no wheel motion before arming.
- Deadman release, joystick removal, DDS loss, network loss, and publisher death.
- Competing command sources and expired or replayed remote commands.
- Controller, encoder, IMU, over-temperature, and HAL fault injection.
- Shutdown and process crash while commanded motion is non-zero.
- Power removal during every update stage followed by rollback verification.
- Unauthorized discovery, telemetry, parameter, service, and motion attempts.
- Long-duration full-sensor thermal, memory, CPU, and storage testing.

Implementation begins with the state model and command supervisor. Systemd
autostart is not enabled on a physical robot until the disarmed boot invariant
has a hardware test.
