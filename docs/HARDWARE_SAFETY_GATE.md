# Physical Hardware Safety Gate

This document records the Phase 2 hardware gate for the physical `stack_4wd`
robot. The deterministic logic and VMX/Titan enforcement are implemented, and
the operator has confirmed the E-stop status input on FlexDIO channel 8 and a
separate local-enable input on channel 9. Motor-power-disconnected input
acceptance was completed on 2026-08-28; lifted-wheel acceptance is not complete.
This is not permission to move the robot or deploy boot services.

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

The robot's original workspace still contains an older, modified source tree.
The reviewed commits were built in an isolated `studica_acceptance_ws`; no
systemd unit or production launch was installed or started.

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
| `LOCAL_ENABLE` | Momentary Start button at the robot | Held requests motion authorization; release immediately removes it |

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
| E-stop status contact | Normally closed from channel 8 signal to VMX ground when healthy | Input sequence passed on 2026-08-28 |
| E-stop primary contacts | Remove Titan motor power or hardware enable | Operator-confirmed; lifted fixture verification pending |
| Channel collision | Old optional duty-cycle and ultrasonic examples mention channels 8/9 but are disabled | Inspected inactive; accessory container remains forbidden |
| `LOCAL_ENABLE` input | VMX FlexDIO channel `9`, momentary Start button | Input sequence passed on 2026-08-28; powered hold/release sequence passed on 2026-08-29 |

The checked-in `stack_4wd` profile records channels 8 and 9 as a pair. Other
physical profiles retain fail-closed `-1` placeholders. This mapping is not a
deployment authorization: lifted-wheel fault and recovery testing remains a
required gate.

### Input acceptance evidence

The motor-power-disconnected fixture used `studica_drivers` commit
`5a866ff2eb3164795d32d71764702bd50e4b2dfe` and `studica_vmxpi_ros2` commit
`d6a2a350369cbd1641dfdd1d5b79ef89985f2748` on the ARM64 VMXPi.

- the first run failed closed because released channel 8 remained HIGH/open;
- after the NC-to-ground connection was corrected, the rerun passed healthy
  baseline, E-stop press/release, enable ON/OFF, open-wire, and reconnection;
- every required state remained stable for at least 500 ms;
- brief enable transitions while the shared wiring was handled never coincided
  with a healthy E-stop input and must be rechecked for connector strain during
  lifted acceptance;
- the VMX HAL reported zero read, retry, CRC, and write failures, then closed all
  DIO and pigpio resources;
- the successful log is
  `/home/vmx/studica_acceptance_ws/safety-input-check-20260828-rerun1.log`, with
  SHA-256 `bac299e1b9ca7be578ab92c6a567ac01581a047c3ed60aafb70d9c469a807460`.

### First lifted attempt: invalid low-battery run

On 2026-08-29, the isolated stack reached a 13 PASS, 0 WARN, 0 FAIL
readiness result with the E-stop released and Start released. The powered
hold/release sequence then demonstrated:

- Start held: `enable_active=1`, `motion_enabled=1`, gate `ENABLED`, and all
  four target and measured velocities remained zero;
- Start released: the gate returned to `READY`, motion authorization became
  zero, and all four target and measured velocities remained zero;
- VMX temperature was 50.6 C with `throttled=0x0`; Titan temperature was
  28.6 C before the wheel test.

The wheel run is **not acceptance evidence**. Six attempted directions logged
FAIL before the sequence was manually cancelled. The operator then identified
that the battery was very low, and the VMXPi subsequently rebooted. A low or
sagging supply invalidates motor tracking results and can explain severe
tracking loss, intermittent stuck-encoder diagnostics, and network/compute
instability. MCAP extraction found a 2.0 rad/s maximum target but only 1.063,
1.259, 0.946, and 0.317 rad/s maximum measured speed for front-left,
front-right, rear-left, and rear-right respectively. The hardware gate remained
consistently enabled for all 651 recorded state samples. The report and
26.43-second MCAP are retained as failure evidence:

- report SHA-256:
  `3146d30c10ab21380b241e45c266e14c9a6cf4ba8a1953b90b6de11d24d39b56`;
- MCAP SHA-256:
  `64c1f5d862434505923267d6921b42c5c734871f4d37b6f0ad1f59a6f5f849d3`;
- run log SHA-256:
  `a3d11eb8e9485613344c7152fb8a9182bf7b35e4a978d3bde949f21a2391a600`.

`studica_robot_monitor` commit
`182be121ae0ed08b368d505b5e65b0250642f929` now requires explicit charged
battery confirmation and a live `ENABLED` hardware gate, stops on the first
failed trial or blocking diagnostic, and atomically retains partial results.
The complete lifted fixture remains blocked until that revision passes with a
charged, measured battery.

## Local-enable sequence

The deterministic gate has four states:

| State | Motion | Exit condition |
|---|---|---|
| `WAITING_FOR_SAFE_RELEASE` | Disabled | Healthy inputs and enable OFF continuously for 500 ms |
| `READY` | Disabled | A new enable ON edge stable for 100 ms |
| `ENABLED` | Hardware-gated | Enable OFF, E-stop loss, invalid read, or drive fault |
| `FAULT_LATCHED` | Disabled | Cause cleared and enable held OFF for 500 ms |

An enable control held active across boot cannot authorize motion. Releasing
the momentary Start button disables motion immediately without debounce. A
released button after a cleared fault is the local acknowledgement, but a new
released-to-held transition is still required to enable.

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
| `enable_active` | `1` while the active-low local Start button requests enable |
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

1. record the selected FlexDIO labels, channel numbers, voltage jumper, control
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
6. measure and record a charged battery within its manufacturer limits, then
   confirm it remains stable under the expected test load;
7. cold-boot at least 20 times with Start released, held, disconnected, and
   bouncing;
8. verify zero targets for E-stop press, Start release, wire removal, DIO read
   failure, encoder loss, over-temperature, supervisor crash, and controller
   restart;
9. verify recovery always requires Start released followed by a new press;
10. archive timestamps, logs, wiring photos, battery measurements, and the
    signed result.

Only a passing fixture authorizes the later systemd/autostart phase.
