# Physical Hardware Safety Gate

This document records the Phase 2A hardware decision for the physical
`stack_4wd` robot. It is a wiring and implementation contract, not permission
to move the robot or deploy boot services.

## Evidence from the robot

A read-only inventory of `vmx@192.168.1.173` on 2026-08-28 confirmed:

- Ubuntu 22.04 arm64 on the VMX-pi, with no project systemd unit installed;
- no ROS, joystick, or motor-control process running at inspection time;
- no Bluetooth joystick input device attached;
- VMX DIO support exists in `studica_drivers`, while the optional DIO accessory
  is disabled;
- the drivetrain hardware already exports Titan PID support, controller
  temperature/age, fault latch, commanded velocity, encoder freshness, and
  feedback age through `ros2_control` state interfaces.

The inspected robot still runs an older source revision. Phase 1 and this
design are not installed on it.

Studica documents 30 VMX digital channels and says all FlexDIO pins are
direction-selectable in software. The high-current DIO bank depends on a
physical input/output jumper and defaults to output. Therefore the safety
fixture will use two reviewed, otherwise-unused **FlexDIO** channels, not the
high-current bank. See Studica's [connector guide](https://docs.wsr.studica.com/en/latest/docs/VMX/setup.html)
and [channel-addressing guide](https://docs.wsr.studica.com/en/latest/docs/VMX/wpi-channel-addressing.html).

## Physical circuit contract

The design needs two independent inputs:

| Input | Physical device | Fail-safe meaning |
|---|---|---|
| `ESTOP_OK` | Auxiliary normally-closed contact on the physical emergency-stop circuit | Closed/grounded only when the E-stop loop is healthy |
| `LOCAL_ENABLE` | Guarded maintained enable switch at the robot | ON requests local motion authorization; OFF immediately removes it |

Both inputs use VMX pull-ups and active-low wiring. An open connector or broken
wire therefore reads inactive/not-OK and disables motion. The physical E-stop's
primary contacts must remove Titan motor power or hardware enable without Linux,
ROS, the VMX firmware, or the network. Its DIO auxiliary contact reports status;
it is not the stopping mechanism.

Exact DIO channel numbers remain unset until an operator opens the VMX enclosure,
identifies the connector labels and jumper state, checks all existing wiring,
and records two unused FlexDIO channels. Guessing channel numbers is forbidden.

## Local-enable sequence

The deterministic gate has four states:

| State | Motion | Exit condition |
|---|---|---|
| `WAITING_FOR_SAFE_RELEASE` | Disabled | Healthy inputs and enable OFF continuously for 500 ms |
| `READY` | Disabled | A new enable ON edge stable for 100 ms |
| `ENABLED` | Hardware-gated | Enable OFF, E-stop loss, invalid read, or drive fault |
| `FAULT_LATCHED` | Disabled | Cause cleared and enable held OFF for 500 ms |

An enable switch left ON across boot cannot authorize motion. Releasing the
switch disables motion immediately without debounce. Turning it OFF after a
cleared fault is the local acknowledgement, but a new OFF-to-ON transition is
still required to enable.

The testable reference logic is in `local_enable_gate.hpp`. It uses a monotonic
time input and treats invalid time, DIO sample failure, E-stop loss, and unhealthy
drive state as fail-closed faults.

## Enforcement boundary

The final gate must be implemented inside `VmxSystemHardware` using the same
`VMXPi` instance that owns Titan and the IMU:

1. initialize both DIO resources during hardware configuration;
2. read them during each hardware cycle with an API that distinguishes a valid
   LOW value from a read failure;
3. combine them with PID, temperature, encoder freshness, and fault-latch health;
4. force every wheel target to zero in `write()` unless the local gate is
   `ENABLED`;
5. export read-only gate state for the supervisor and diagnostics.

The ROS safety supervisor will mirror the hardware state so applications see
`READY_DISARMED`, `ARMED`, and `FAULT`. That DDS message is not the authority.
Even a forged ROS topic or service request must still encounter the local gate
inside the Titan write path.

Do not launch the separate `studica_ros2_control` accessory container to read
these safety inputs. It creates another VMX HAL owner and places the safety
decision behind a network-visible topic.

## Driver prerequisite

The current `studica_driver::DIO::Get()` returns `false` both for a legitimate
LOW input and for some read failures. That ambiguity is unsafe for active-low
`ESTOP_OK`. Before hardware integration, the driver must add a non-throwing
`TryGet(bool & value)`-style API whose return value reports read success. A
failed initialization or read must make `sample_valid=false` and latch the gate.

## Hardware acceptance fixture

Before any boot service or floor motion:

1. record the selected FlexDIO labels, channel numbers, voltage jumper, switch
   part numbers, contact type, and wiring diagram;
2. continuity-test the E-stop's primary power contacts and auxiliary status
   contact with motor power disconnected;
3. test each DIO state with Titan motor power disconnected;
4. lift all wheels and place an operator at the physical E-stop;
5. cold-boot at least 20 times with enable OFF, ON, disconnected, and bouncing;
6. verify zero targets for E-stop press, enable release, wire removal, DIO read
   failure, encoder loss, over-temperature, supervisor crash, and controller
   restart;
7. verify recovery always requires enable OFF followed by a new ON edge;
8. archive timestamps, logs, wiring photos, and the signed result.

Only a passing fixture authorizes the later systemd/autostart phase.
