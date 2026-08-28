# Physical Hardware Safety Gate

This document records the Phase 2 hardware gate for the physical `stack_4wd`
robot. The deterministic logic and VMX/Titan enforcement are implemented, and
the operator has confirmed the E-stop status input on FlexDIO channel 8 and a
separate local-enable input on channel 9. Software input and lifted-wheel
acceptance are not complete. This is not permission to move the robot or deploy
boot services.

## Evidence from the robot

A read-only inventory of `vmx@192.168.1.173` on 2026-08-28 confirmed:

- Ubuntu 22.04 arm64 on the VMX-pi, with no project systemd unit installed;
- no ROS, joystick, or motor-control process running at inspection time;
- no Bluetooth joystick input device attached;
- VMX DIO support exists in `studica_drivers`, while the optional DIO accessory
  is disabled;
- the old optional accessory example mentions channel 8 for disabled duty-cycle
  and ultrasonic devices; neither is active, but production must not launch
  that separate VMX HAL owner;
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

## Commissioning record

| Field | Recorded value | Acceptance status |
|---|---|---|
| Robot profile | `stack_4wd` | Confirmed physical profile |
| `ESTOP_OK` input | VMX FlexDIO channel `8` | Operator-confirmed connection on 2026-08-28 |
| VMX input bias | Pull-up | Confirmed in `studica_driver::DIO` input initialization |
| E-stop status contact | Normally closed from channel 8 signal to VMX ground when healthy | Operator-confirmed; software state sequence pending |
| E-stop primary contacts | Remove Titan motor power or hardware enable | Operator-confirmed; lifted fixture verification pending |
| Channel collision | Old optional duty-cycle and ultrasonic examples mention channels 8/9 but are disabled | Inspected inactive; accessory container remains forbidden |
| `LOCAL_ENABLE` input | VMX FlexDIO channel `9` | Operator-confirmed separate maintained switch on 2026-08-28 |

The checked-in `stack_4wd` profile records channels 8 and 9 as a pair. Other
physical profiles retain fail-closed `-1` placeholders. This mapping is not a
deployment authorization: the revision remains off the robot until both inputs
pass the motor-power-disconnected state sequence below.

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

The gate is implemented inside `VmxSystemHardware` using the same `VMXPi`
instance that owns Titan and the IMU. It:

1. initializes both DIO resources during hardware configuration;
2. reads them during each hardware cycle with an API that distinguishes a valid
   LOW value from a read failure;
3. combines them with PID, temperature, encoder freshness, and fault-latch health;
4. forces every wheel target to zero in `write()` unless the local gate is
   `ENABLED`;
5. disables Titan while the gate is closed and establishes zero targets for a
   complete control cycle before accepting motion after enable;
6. exports read-only gate state for the supervisor and diagnostics.

The `stack_4wd` profile uses the confirmed channel 8/9 pair. Other physical
profiles keep `-1` for both channel parameters, which `VmxSystemHardware`
rejects before it opens motor control. The configured `stack_4wd` revision is
not deployed until its acceptance fixtures pass.

The ROS safety supervisor mirrors the hardware state so applications see
`BOOTING`, `READY_DISARMED`, `ARMED`, and `FAULT`. It rejects malformed,
conflicting, or older-than-500-ms state and requires local OFF after a software
disarm or supervisor restart before it accepts a later hardware enable. That
DDS message is not the authority. Even a forged ROS topic or service request
must still encounter the local gate inside the Titan write path.

Do not launch the separate `studica_ros2_control` accessory container to read
these safety inputs. It creates another VMX HAL owner and places the safety
decision behind a network-visible topic.

## Driver support

`studica_drivers` commit `5a866ff` adds `DIO::TryGet(bool & value)`. Its return
value reports read success while the output reports HIGH/LOW, so a legitimate
LOW cannot be confused with failure. Failed initialization or reads make
`sample_valid=false` and latch the local gate. The legacy ambiguous `Get()` is
not used for either safety input.

## Exported hardware state

The `hardware_safety` sensor exports these numeric `ros2_control` state
interfaces for read-only diagnostics:

| Interface | Values |
|---|---|
| `input_valid` | `1` only when both DIO reads succeeded |
| `estop_ok` | `1` only when the active-low E-stop status contact is closed |
| `enable_active` | `1` while the active-low local switch requests enable |
| `drive_healthy` | `1` when PID, temperature, feedback, and fault state permit motion |
| `motion_enabled` | `1` only while the hardware gate authorizes Titan output |
| `gate_state` | `0` waiting, `1` ready, `2` enabled, `3` fault-latched |
| `fault_reason` | `0` none, `1` input, `2` E-stop, `3` drive, `4` time |

These interfaces are observability only and do not accept commands.

In hardware mode, `studica_robot_monitor` decodes them into the
`Robot/Control/HardwareSafety` diagnostic. Missing, malformed, input-invalid,
E-stop-not-OK, drive-unhealthy, fault-latched, or internally inconsistent state
is an error and makes the read-only `robot_check --mode hardware` fail. The
diagnostic is not emitted in simulation mode.

## Hardware acceptance fixture

Before any boot service or floor motion:

1. record the selected FlexDIO labels, channel numbers, voltage jumper, switch
   part numbers, contact type, and wiring diagram;
2. continuity-test the E-stop's primary power contacts and auxiliary status
   contact with motor power disconnected;
3. build the hardware packages on the VMXPi without starting robot bringup;
4. with Titan motor power physically disconnected and no other VMX HAL owner,
   run the input-only acceptance tool and archive its complete output:

   ```bash
   set -o pipefail
   source /opt/ros/humble/setup.bash
   source "$HOME/studica_ws/install/setup.bash"
   check_bin="$(ros2 pkg prefix studica_vmxpi_ros2)/lib/studica_vmxpi_ros2/safety_input_check"
   sudo env "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "$check_bin" \
     --estop-channel 8 \
     --enable-channel 9 \
     --confirm-motor-power-disconnected |& tee safety-input-check.log
   ```

   The tool never initializes Titan. It verifies healthy baseline, E-stop
   press/release, enable ON/OFF, E-stop status-wire removal, and reconnection.
   Any read failure, wrong polarity, unstable state, or timeout fails the test.
5. lift all wheels and place an operator at the physical E-stop;
6. cold-boot at least 20 times with enable OFF, ON, disconnected, and bouncing;
7. verify zero targets for E-stop press, enable release, wire removal, DIO read
   failure, encoder loss, over-temperature, supervisor crash, and controller
   restart;
8. verify recovery always requires enable OFF followed by a new ON edge;
9. archive timestamps, logs, wiring photos, and the signed result.

Only a passing fixture authorizes the later systemd/autostart phase.
