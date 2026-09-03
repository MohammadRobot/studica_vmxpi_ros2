# Physical Hardware Safety Gate

This document records the Phase 2 hardware gate for the physical `stack_4wd`
robot. The deterministic logic and VMX/Titan enforcement are implemented for a
four-button panel and two status LEDs. Input acceptance began on 2026-08-28,
lifted-wheel validation completed on 2026-09-02, and the complete DIO 8--13 panel
plus supervised joystick/SLAM floor operation passed on 2026-09-03. This is not
permission to deploy boot services; the independent hardware torque-removal and
cold-boot production gates remain open.

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

| Signal | DIO | Physical device | Fail-safe meaning |
|---|---:|---|---|
| `ESTOP_OK` | 8 | E-stop auxiliary NC contact | Grounded only when the E-stop loop is healthy |
| `START` | 9 | Momentary NO Start button | A fresh press requests the enable latch; release does not cancel it |
| `RESET` | 10 | Momentary NO Reset button | Acknowledges a cleared fault; never enables motion |
| `STOP_OK` | 11 | Momentary NC Stop button | Press or broken/open wire immediately clears authorization |
| `START_LED` | 12 | Active-high 5 V LED module | Lit only while the local hardware gate is enabled |
| `STOP_LED` | 13 | Active-high 5 V LED module | Lit while the local hardware gate is not enabled |

The four inputs use VMX pull-ups and active-low wiring. An open E-stop or Stop
status connector therefore disables motion while the control loop remains
healthy. The two high-current outputs require the VMX output-voltage jumper at
5 V and the high-current direction jumper in OUTPUT mode. The physical E-stop's
primary contacts must remove
Titan motor power or a hardware drive-enable signal without Linux, ROS, the VMX
firmware, or the network. Its DIO auxiliary contact reports status; it is not the
stopping mechanism. The present robot powers the VMXPi from the Titan power
output, so independent removal of motor power while retaining VMXPi power has
not been demonstrated and is a production hardware blocker.

## Commissioning record

| Field | Recorded value | Acceptance status |
|---|---|---|
| Robot profile | `stack_4wd` | Confirmed physical profile |
| `ESTOP_OK` input | VMX FlexDIO channel `8` | Operator-confirmed connection on 2026-08-28 |
| VMX input bias | Pull-up | Confirmed in `studica_driver::DIO` input initialization |
| E-stop status contact | Normally closed from channel 8 signal to VMX ground when healthy | Input sequence passed on 2026-08-28 |
| E-stop primary contacts | Remove Titan motor power or hardware enable independently of VMXPi power | Not demonstrated; current shared Titan/VMXPi supply requires redesign or an independently controlled motor-enable/power stage |
| Channel collision | Old optional duty-cycle and ultrasonic examples mention channels 8/9 but are disabled | Inspected inactive; accessory container remains forbidden |
| Start input | DIO `9`, momentary NO-to-ground | Full input and live latch sequence passed 2026-09-03 |
| Reset input | DIO `10`, momentary NO-to-ground | Full input and live fault-reset sequence passed 2026-09-03 |
| Stop input | DIO `11`, momentary NC-to-ground | Press, release, and open-wire checks passed 2026-09-03 |
| Start/Stop LEDs | DIO `12`/`13`, active-high 5 V modules | Output-only both-off/Start/Stop/both-off sequence passed 2026-09-03 |

### Momentary Start commissioning resolution

The first floor attempt exposed that hold-to-run Start did not match the
physical momentary button. The gate was changed to press-to-arm only after the
dedicated NC Stop, Reset, and two status LEDs were installed. The current design:

1. boots and recovers from every fault without motion authorization;
2. requires released controls for a complete safe interval and then a fresh,
   debounced Start press;
3. keeps authorization latched when Start is released;
4. clears authorization immediately on Stop press/open wire, E-stop, invalid
   input, drive fault, shutdown, or reboot;
5. requires a deliberate Reset after a cleared fault, followed by a new Start;
6. retains the joystick L1 deadman as an additional ROS command gate.

On 2026-09-03 the dedicated input-only checker passed every DIO 8--11 state and
open-wire transition with 500 ms stability and zero VMX read/write/CRC failures.
The output-only checker passed the DIO 12/13 LED sequence and left both outputs
off. The live runtime then completed Reset/Start operation, supervised floor
teleoperation, LiDAR at approximately 11.2 Hz, and live SLAM mapping. These
results close the momentary-control and supervised floor-teleoperation gates,
but do not close the independent hardware torque-removal or boot-service gates.

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
readiness result with the E-stop released and Start released. The old
hold-to-run implementation then demonstrated:

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
`f51a2fc0f3c1d23264bdfa7651d7f118d87eaaf2` now requires explicit charged
battery confirmation and a live `ENABLED` hardware gate, stops on the first
failed trial or blocking diagnostic, and atomically retains partial results.
The complete lifted fixture remains blocked until that revision passes with a
charged, measured battery.

### Charged lifted retry: safe first-trial failure

The two 12 V, 3000 mAh NiMH packs measured 13.33 V individually and 13.36 V in
parallel at open circuit. On 2026-08-29, with all wheels lifted and an operator
at the E-stop, the hardened validator was retried against
`studica_robot_monitor` commit
`f51a2fc0f3c1d23264bdfa7651d7f118d87eaaf2`. It stopped automatically after
the first `front_left_wheel_joint` `+2.0 rad/s` trial failed tracking:

- all 285 samples had fresh encoders and the measured direction was correct;
- the maximum uncommanded-wheel velocity was `0.0 rad/s`;
- the peak commanded-wheel measurement was only `0.4681 rad/s`;
- settled mean absolute tracking error was `1.8546 rad/s`, above the
  `0.300 rad/s` limit;
- the wheel stopped within one second and no blocking diagnostic error was
  present;
- after the failure, the local gate was `READY`, motion authorization was off,
  all targets and measured velocities were zero, and Titan temperature was
  approximately 32.6 C.

The retained 4.08-second evidence is:

- report:
  `/home/vmx/studica_acceptance_ws/charged_results/motor_validation_20260829T144204Z/report.yaml`,
  SHA-256 `f31a9bc1e97a837090e10da44553189cc0914f1c5614590f9860e1643aac8716`;
- MCAP:
  `/home/vmx/studica_acceptance_ws/charged_results/motor_validation_20260829T144204Z/telemetry/telemetry_0.mcap`,
  SHA-256 `dea45276adc30abcdcf738924d7d29542d1cc518a0c58fd7c699a0634adb4ac7`.

This controlled charged failure shows that low battery was not the sole cause
of the earlier tracking loss. PID configuration, encoder scaling, Titan motor
configuration, wiring, and mechanical load must be diagnosed with motor power
disconnected before another commanded wheel test. No boot service or floor
motion is authorized.

### CPR-corrected trial and periodic-enable regression

On 2026-08-31, Titan firmware 2.0.5 was programmed with the documented 732 CPR
value for its internal S-curve controller and a single guarded
`front_left_wheel_joint` `+2.0 rad/s` trial was run. All encoder samples were
fresh, direction was correct, the other three wheels remained at zero, and the
wheel stopped within one second. The trial still failed tracking: peak speed
was `0.7697 rad/s` and settled mean absolute error was `1.7135 rad/s`.

The evidence was copied off the robot before further changes:

- report: `robot_test_results/single_wheel_cpr732_20260831T164434Z/report.json`,
  SHA-256 `7d7f1646faa614e92a713268a39611bd26875ee52a5ab2e20ba4522593dea0d4`;
- MCAP: `robot_test_results/single_wheel_cpr732_20260831T164434Z/telemetry/telemetry_0.mcap`,
  SHA-256 `9e1a4629f9096d7f15b01c053225323de3cd3c35734f3716ee282d9bbaf82754`.

Review of the low-level driver then found a deterministic regression in the
VMXCAN periodic-frame protocol. `TryEnable(false)` scheduled the Titan
`DISABLED_FLAG` every 10 ms. A later `TryEnable(true)` scheduled the distinct
`ENABLED_FLAG` every 100 ms without first cancelling the retained periodic
disable frame. The hardware safety gate was the first path to exercise
disable-then-enable in one process, so the Titan continued receiving both
states while the wheel command was active. Earlier successful eight-direction
reports from 2026-08-05 through 2026-08-11 did not exercise that transition and
show that the automated readings changed after the safety-gate path was added.
Those reports did not include an independent physical wheel-motion observation
and cannot establish that the measured RPM represented output-wheel motion.

The driver now cancels the opposite periodic CAN ID before scheduling either
state and refuses to enable if cancellation fails. Unit tests cover both
directions and the fail-closed cancellation case. The patched driver and robot
stack build on ARM64, and all relevant ARM64 driver tests pass. With Titan
power disconnected and the E-stop pressed, a live startup check reported all
four commanded and measured velocities at zero, fresh encoder state, no Titan
fault, and gate state `FAULT_LATCHED/ESTOP_NOT_OK`.

This is not lifted-wheel acceptance. A repeat of the same single-wheel trial is
required before the eight-direction validator or any floor test. A
shutdown-time diagnostic produced after SIGINT began closing the VMX HAL must
also be treated as a separate lifecycle race; the test stack exited cleanly,
but powered shutdown behavior remains an explicit production acceptance item.

### Patched retry: physical motion contradicted Titan feedback

On 2026-08-31, the periodic-enable patch was exercised in the same guarded
single-wheel fixture. The automated report recorded `2.5698 rad/s` peak RPM,
`0.5234 rad/s` settled mean error, `28.49%` overshoot, fresh feedback, no
uncommanded-wheel velocity, and a stop within one second. The operator then
confirmed that the front-left wheel **did not physically rotate during the
commanded test**. The run is therefore invalid as velocity-tracking evidence;
its apparent improvement must not be described as physical wheel motion.

The copied evidence is:

- report: `robot_test_results/single_wheel_cpr732_20260831T174127Z/report.json`,
  SHA-256 `eec8a7f2610708e63f09a5d9aeea950daa51cf0079b0c4eb13cccf72d6143fb4`;
- MCAP: `robot_test_results/single_wheel_cpr732_20260831T174127Z/telemetry/telemetry_0.mcap`,
  SHA-256 `8c3622d46bc1abcbde1d4b1fd3375c2d3dcc428a1976c582b784a158f60b69e5`.

Offline MCAP analysis found 801 encoder-count equivalents of position change
(`3.437726 rad` using 1464 counts per output revolution), while integrating
the reported RPM over the same samples gives `6.833380 rad`, or approximately
1592 count equivalents. The near-exact factor-of-two disagreement is
consistent with the separate 732-CPR Titan S-curve setting and 1464-count ROS
position conversion, but position and velocity exported for one joint must be
kinematically consistent. More importantly, neither changing feedback stream
is proof of output-wheel motion after the operator's direct observation.

No further commanded test is authorized until a powered-disabled manual
fixture verifies all of the following:

1. Titan M2 motor output and M2 encoder/limit input both lead to the physical
   front-left assembly;
2. one manually applied front-left wheel revolution changes only the expected
   encoder channel by the documented count and sign;
3. the output shaft, hub, and 61:1 gearbox transmit the same manual rotation;
4. stationary encoder counts remain stable with controller power present;
5. the RPM/position scaling contract is corrected and tested against a timed
   manual revolution.

Until those checks pass, likely causes include a motor/encoder channel mismatch,
an encoder cable or electrically induced count signal, a loose hub or damaged
gearbox between the motor encoder and wheel, and firmware/driver RPM scaling.
PID sensitivity and current-limit tuning are explicitly deferred.

### Powered-disabled manual encoder fixture

On 2026-09-01, the operator traced Titan M2 motor output and M2 encoder input to
the physical front-left assembly, verified the M2 fuse, and confirmed that the
wheel, hub, output shaft, and gearbox were mechanically coupled. With the
E-stop pressed, Start released, motion authorization off, and every target at
zero, the operator then rotated only that wheel forward by one marked physical
revolution over several seconds.

Only `front_left_wheel_joint` changed. Position reported `6.386188 rad`, or
1.016 physical revolutions, which is consistent with the manual mark and 1464
counts per output revolution. Integrating Titan RPM over the same interval gave
`12.798602 rad`, or 2.037 revolutions. The 2.004 ratio proves that programming
732 makes firmware 2.0.5 report twice the physical output rate; it is not a PID
tuning effect and occurred without any motor command.

Evidence copied from the VMXPi:

- MCAP: `robot_test_results/manual_encoder_front_left_20260901T0605Z/manual_encoder_front_left_20260901T0605Z_0.mcap`,
  SHA-256 `855aeeafef4be1bb29488280b27f302b663c4d1d7dd8d14ef376852f82af3e3c`;
- metadata: `robot_test_results/manual_encoder_front_left_20260901T0605Z/metadata.yaml`,
  SHA-256 `2ad7b0fdb178dd1f56e448d86b32a68df1246287272ab2eadf51fa157f2296e0`.

For comparison, the 2026-08-11 passing MCAP recorded before the explicit 732
override had a front-left position delta of `-0.223173 rad` and integrated RPM
of `-0.239977 rad`, a `0.016803 rad` difference rather than a factor-of-two
error. The retained MCAP SHA-256 is
`9bfbc042f015e3be79e1d6491d84bf564c790c0a8034671b26ace0f9282e7b33`.

The physical mapping, fuse, and mechanical-coupling prerequisites now pass.
The profile was changed to program 1464, and the validator now enforces
position-versus-RPM consistency.

The rebuilt ARM64 artifact passed the required second powered-disabled manual
revolution on 2026-09-01. With E-stop pressed, Start released, all wheels
lifted, and every target at zero, the operator rotated only the front-left
wheel forward by one marked revolution. Position changed by `6.167307 rad`
(`0.981557` revolution) and integrated Titan RPM changed by `6.145676 rad`
(`0.978115` revolution). Their ratio was `0.996493`, with only `0.021631 rad`
difference; the earlier factor-of-two error is gone. Front-right and rear-right
were unchanged. Rear-left changed by one encoder count (`-0.004292 rad`), which
is below the fixture tolerance and was not accompanied by physical movement.

Retained evidence:

- analysis: `robot_test_results/manual_encoder_front_left_cpr1464_20260901T023033Z/analysis.json`,
  SHA-256 `afbc745a6a1b351c98e8cfb7cbb0b835047af3bd484d01722147b08f9a8dab59`;
- MCAP: `robot_test_results/manual_encoder_front_left_cpr1464_20260901T023033Z/manual_encoder_front_left_cpr1464_20260901T023033Z_0.mcap`,
  SHA-256 `d2a78c167e11ee126a5e564a065647c79411efe32936507fdc0fb886aa6ac942`;
- metadata: `robot_test_results/manual_encoder_front_left_cpr1464_20260901T023033Z/metadata.yaml`,
  SHA-256 `afb48bcbc47ff5c208864a0c0b1b20523c7cd9346793029be95e78a7097618ed`.

This closes the encoder scaling and channel-mapping gate. It did not by itself
authorize commanded motion; the subsequent managed-shutdown and guarded
lifted-wheel results are recorded below.

### VMX HAL shutdown ownership

The powered-disabled shutdown immediately after this fixture reproduced a
vendor lifecycle race. The VMX HAL's SIGINT handler closed pigpio while the
Humble controller-manager loop was still executing, after which the loop
reported DIO board-communication errors and failed zero-target/disable writes.
The process exited, but that ordering is not production-safe.

Physical hardware therefore uses the package's `vmx_control_node`, not the
generic `ros2_control_node`. It blocks SIGINT/SIGTERM before ROS, VMX, or any
worker thread is created and starts rclcpp without its process-wide signal
handler. A normal `sigtimedwait()` path receives termination, stops and joins
the real-time control loop, stops the executor, shuts down controllers,
deactivates/finalizes all hardware components, and only then destroys
VMX/pigpio resources. The controller manager is destroyed before
`rclcpp::shutdown()`, preventing its Humble pre-shutdown callback from repeating
the lifecycle transitions. Mock and simulation modes continue to use the
generic controller manager.

The powered-disabled ARM64 acceptance run on 2026-09-02 passed this ordering.
The log reported `Managed shutdown requested by SIGINT`, controller shutdown,
`Titan Successfully deactivated!`, successful hardware `deactivate` and
`shutdown`, `Managed VMX shutdown completed before HAL resource destruction`,
VMX RemoteServer stop, and pigpio close, in that order. The control executable
and launch both exited with status 0. There were no DIO/CAN communication
errors, failed zero/disable writes, vendor signal-handler stack traces, or
forced termination. This closes the managed-shutdown gate; commanded motion
still requires the guarded lifted-wheel acceptance test.

### CPR-1464 lifted-wheel acceptance

On 2026-09-02, after the managed-shutdown gate passed, the charged robot ran
the complete guarded validator at `+2.0` and `-2.0 rad/s` for each wheel. All
eight trials passed. Every sample had fresh feedback and the expected
direction, no uncommanded wheel exceeded `0.0 rad/s`, all position deltas were
consistent with integrated RPM, overshoot was `0.0%`, and every wheel stopped
below the limit within one second. Settled mean errors ranged from
`0.1431 rad/s` through `0.2997 rad/s`; the rear-left positive trial is close to
the `0.3000 rad/s` boundary and must remain under observation during the first
floor regression.

The safety operator independently confirmed that each physical wheel rotated
one at a time in both directions, with no wrong-wheel or simultaneous motion,
slipping hub, or unusual mechanical behavior. After Start was released, the
live state returned to `READY_DISARMED` with all four target and measured
velocities at zero. The powered shutdown immediately afterward also completed
in the required order and exited with status 0.

Retained evidence:

- report: `robot_test_results/motor_validation_20260902T165932Z/report.yaml`,
  SHA-256 `1c77d26b33a8bf6d48639ee2cd14358fe6d8961b7d934e4a9796216e8fad0da4`;
- MCAP: `robot_test_results/motor_validation_20260902T165932Z/telemetry/telemetry_0.mcap`,
  SHA-256 `752a6dd5a0504dcc3b9c71ec1c9272ddf2784d67c27e8204201c1f5fe4b723cd`;
- metadata: `robot_test_results/motor_validation_20260902T165932Z/telemetry/metadata.yaml`,
  SHA-256 `3872fc790e13dbb097e354e246808c2b01a60ccedd7280fe93adf37eaa565579`.

This closes the lifted-wheel gate. It authorizes only the documented,
supervised low-speed floor regression; autonomous navigation remains blocked
until floor motion, LiDAR/odometry/TF, mapping, and localization checks pass.

## Local-enable sequence

The deterministic gate has four states:

| State | Motion | Exit condition |
|---|---|---|
| `WAITING_FOR_SAFE_RELEASE` | Disabled | E-stop/Stop healthy and Start/Reset released continuously for 500 ms |
| `READY` | Disabled | A fresh Start press stable for 100 ms |
| `ENABLED` | Hardware-gated | Stop press/open wire, E-stop loss, invalid read, drive fault, shutdown, or reboot |
| `FAULT_LATCHED` | Disabled | Cause cleared, then a fresh Reset press; safe release and a later Start are still required |

A Start control held active across boot cannot authorize motion. Start is a
momentary press-to-arm control, so releasing it after a valid press leaves the
gate enabled. Pressing Stop or opening its NC circuit disables immediately.
Reset only acknowledges a cleared fault and can never authorize motion.

The testable reference logic is in `local_enable_gate.hpp`. It uses a monotonic
time input and treats invalid time, DIO sample failure, E-stop loss, and unhealthy
drive state as fail-closed faults.

## Enforcement boundary

The gate is implemented inside `VmxSystemHardware` using the same `VMXPi`
instance that owns Titan and the IMU. It:

1. initializes four DIO inputs and two DIO LED outputs during hardware configuration;
2. reads them during each hardware cycle with an API that distinguishes a valid
   LOW value from a read failure;
3. combines them with PID, temperature, encoder freshness, and fault-latch health;
4. forces every wheel target to zero in `write()` unless the local gate is
   `ENABLED`;
5. disables Titan while the gate is closed and establishes zero targets for a
   complete control cycle before accepting motion after enable;
6. exports read-only gate state for the supervisor and diagnostics.

The `stack_4wd` profile uses the confirmed channel 8--13 panel. Other physical
profiles keep `-1` for all panel channel parameters, which `VmxSystemHardware`
rejects before it opens motor control. The `stack_4wd` revision is approved for
supervised operation but not for unattended boot until the remaining production
fixtures pass.

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
| `input_valid` | `1` only when all four DIO reads succeeded |
| `estop_ok` | `1` only when the active-low E-stop status contact is closed |
| `start_active` | `1` while the active-low momentary Start button is pressed |
| `reset_active` | `1` while the active-low momentary Reset button is pressed |
| `stop_ok` | `1` only while the active-low NC Stop circuit is closed |
| `drive_healthy` | `1` when PID, feedback, fault state, and every enabled health check permit motion |
| `motion_enabled` | `1` only while the hardware gate authorizes Titan output |
| `gate_state` | `0` waiting, `1` ready, `2` enabled, `3` fault-latched |
| `fault_reason` | `0` none, `1` input, `2` E-stop, `3` drive, `4` time |
| `start_led_on` | `1` when the Start LED output is commanded on |
| `stop_led_on` | `1` when the Stop LED output is commanded on |

These interfaces are observability only and do not accept commands.

The `titan_controller/temperature_safety_enabled` state interface separately
reports whether controller temperature participates in the gate. A value of
`0` preserves raw temperature telemetry for diagnosis while excluding the
known-unreliable signal from motion authorization and fault latching.

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
4. with robot bringup stopped, all wheels secured off the floor, and no other VMX HAL owner,
   run the input-only acceptance tool and archive its complete output:

   ```bash
   set -o pipefail
   source /opt/ros/humble/setup.bash
   source "$HOME/studica_ws/install/setup.bash"
   check_bin="$(ros2 pkg prefix studica_vmxpi_ros2)/lib/studica_vmxpi_ros2/safety_input_check"
   sudo env "LD_LIBRARY_PATH=$LD_LIBRARY_PATH" "$check_bin" \
     --confirm-runtime-stopped \
     --confirm-wheels-lifted \
     --estop-channel 8 \
     --start-channel 9 \
     --reset-channel 10 \
     --stop-channel 11 |& tee safety-input-check.log
   ```

   The tool never initializes Titan. It verifies the baseline, every button
   press/release, E-stop and Stop open-wire behavior, and reconnection.
   Any read failure, wrong polarity, unstable state, or timeout fails the test.
5. lift all wheels and place an operator at the physical E-stop;
6. measure and record a charged battery within its manufacturer limits, then
   confirm it remains stable under the expected test load;
7. run `safety_led_check` with its four physical confirmation flags and verify
   both-off, Start-only, Stop-only, and both-off indications;
8. cold-boot at least 20 times with Start released, held, disconnected, and
   bouncing;
9. verify zero targets for E-stop press, Stop press/open wire, DIO read
   failure, encoder loss, over-temperature, supervisor crash, and controller
   restart;
10. verify fault recovery requires Reset, safe release, and a new Start press;
11. archive timestamps, logs, wiring photos, battery measurements, and the
    signed result.

Only a passing fixture authorizes the later systemd/autostart phase.
