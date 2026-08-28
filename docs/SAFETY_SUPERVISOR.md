# Safety Supervisor API

The safety supervisor is the only node allowed to publish a command to the
profile-selected drive controller. It is enabled in simulation, mock, and
hardware launches. Simulation and mock starts end in `READY_DISARMED`;
publishing `/cmd_vel` alone cannot move the robot. In hardware mode the node
remains `BOOTING` until it receives a valid, fresh `hardware_safety` state from
exactly one `/dynamic_joint_states` publisher.

Simulation and mock arm through `/robot/arm`. Hardware can reach `ARMED` only
when the local VMX gate reports its physically authorized `ENABLED` state. The
`stack_4wd` profile records the operator-confirmed channel 8/9 safety pair, but
deployment remains blocked until the lifted-wheel fixture passes. The
motor-power-disconnected input fixture passed on 2026-08-28. Other physical
profiles retain `-1` placeholders.

## Public interfaces

| Interface | Type | Contract |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Application input when `control_source:=application` |
| `/cmd_vel/joy` | `geometry_msgs/msg/Twist` | Joystick converter input when `control_source:=joystick` |
| `/joy` | `sensor_msgs/msg/Joy` | Independent joystick deadman and freshness evidence |
| `/robot/state` | `std_msgs/msg/String` | Current state, latched and repeated at 1 Hz |
| `/robot/safety_reason` | `std_msgs/msg/String` | Most recent command or transition decision |
| `/robot/arm` | `std_srvs/srv/Trigger` | Explicitly arm mock/simulation only; always rejected on hardware |
| `/robot/disarm` | `std_srvs/srv/Trigger` | Immediately zero supervisor output; hardware re-arm then requires local OFF followed by ON |

`control_source` is immutable and accepts `application` or `joystick`. There is
no automatic fallback: the supervisor subscribes to only the selected command
topic. The controller-facing `TwistStamped` topic is private. Its name depends
on the selected robot profile. Applications, joystick nodes, Nav2, and remote
clients must never publish to it.

## Simulation operator sequence

After launch, confirm the state and arm explicitly:

```bash
ros2 topic echo /robot/state --once
ros2 service call /robot/arm std_srvs/srv/Trigger '{}'
```

Then start exactly one `/cmd_vel` publisher. Disarm before ending the session:

```bash
ros2 service call /robot/disarm std_srvs/srv/Trigger '{}'
```

Arming clears every command received while disarmed. A fresh command must arrive
after the transition to `ARMED`.

## Command acceptance rules

A command reaches the controller only when all of these are true:

- robot state is `ARMED`;
- exactly one DDS publisher exists on the selected command topic;
- a command arrived within the 250 ms monotonic receive deadline;
- all six `Twist` values are finite;
- `linear.z`, `angular.x`, and `angular.y` are zero.

The default supervisor ceilings are `0.5 m/s` for planar linear speed and
`1.5 rad/s` for yaw rate. Accepted commands are acceleration-limited to
`1.0 m/s²` and `3.0 rad/s²`. The drive controller retains its independent
profile-specific limits and 500 ms timeout as a second layer.

Supervisor parameters are immutable after startup. This prevents a remote
parameter client from changing a safety ceiling or arming policy in a running
process.

Disarmed, missing, expired, malformed, non-planar, and conflicting inputs
produce an immediate zero command. Publisher loss also produces zero without
waiting for a wall-clock or synchronized timestamp.

## Joystick source contract

Built-in joystick launches select `control_source:=joystick` and remap
`teleop_twist_joy` to `/cmd_vel/joy`. The supervisor independently accepts that
command only while exactly one fresh `/joy` publisher reports button index `4`
(L1) held. R1 turbo therefore cannot bypass L1 even though the upstream teleop
node treats its turbo button as a separate enable.

Every transition into `ARMED` requires a valid L1 release followed by a new
press. Missing, stale, malformed, or competing `/joy` state and missing, stale,
malformed, or competing `/cmd_vel/joy` commands immediately produce zero and
latch the same release requirement. Reconnection while L1 is held cannot resume
motion. Application `/cmd_vel` traffic is ignored in joystick mode rather than
being used as an automatic fallback.

Common `/robot/safety_reason` values include:

| Value | Meaning |
|---|---|
| `READY_DISARMED` | Boot completed with motion disabled |
| `ARMED_WAITING_FOR_COMMAND` | Armed; pre-arm commands were cleared |
| `COMMAND_ACCEPTED` | A valid command is being supervised |
| `COMMAND_STALE` | The monotonic receive deadline expired |
| `COMMAND_SOURCE_LOST` | No `/cmd_vel` publisher remains |
| `COMMAND_SOURCE_CONFLICT` | More than one publisher was discovered |
| `COMMAND_NONFINITE` | A NaN or infinity was rejected |
| `COMMAND_UNSUPPORTED_3D` | A non-planar component was rejected |
| `WAITING_FOR_HARDWARE_SAFETY` | Hardware state has not arrived yet |
| `HARDWARE_READY_LOCAL_ENABLE_OFF` | Inputs and drive are healthy; local enable is OFF |
| `HARDWARE_ARMED_WAITING_FOR_COMMAND` | The local hardware gate authorized motion; pre-arm commands were cleared |
| `HARDWARE_DISARMED_WAITING_FOR_LOCAL_RELEASE` | Software disarm is latched until the local switch returns OFF |
| `HARDWARE_SAFETY_STALE` | No hardware gate update arrived within 500 ms |
| `HARDWARE_SAFETY_SOURCE_LOST` | The hardware-state publisher disappeared |
| `HARDWARE_SAFETY_SOURCE_CONFLICT` | More than one hardware-state publisher was discovered |
| `HARDWARE_SAFETY_MALFORMED` | A missing, non-finite, duplicate, or inconsistent gate interface was rejected |
| `HARDWARE_SAFETY_FAULT` | The VMX gate reported a latched physical/drive fault |
| `WAITING_FOR_JOYSTICK_STATE` | The selected joystick source has not reported raw state |
| `JOYSTICK_DEADMAN_RELEASED` | L1 is released and joystick output is zero |
| `JOYSTICK_DEADMAN_RELEASE_REQUIRED` | L1 must be released before another press can authorize commands |
| `JOYSTICK_STATE_STALE` | Raw joystick evidence exceeded its steady-clock deadline |
| `JOYSTICK_STATE_SOURCE_LOST` | The selected `/joy` publisher disappeared |
| `JOYSTICK_STATE_SOURCE_CONFLICT` | More than one `/joy` publisher was discovered |
| `JOYSTICK_STATE_MALFORMED` | The required L1 button field was absent or not binary |

## Hardware state mirror

The supervisor consumes the seven read-only interfaces under
`hardware_safety` in `/dynamic_joint_states`. It validates exact binary values,
finite enum values, fault/state consistency, motion/state consistency, and the
expected input/drive health invariants. The receive deadline uses the robot's
steady clock, not ROS or wall time.

The mirror applies these transitions:

| VMX gate | Supervisor result |
|---|---|
| `WAITING_FOR_SAFE_RELEASE` | `READY_DISARMED`; zero output; wait for local OFF interval |
| `READY` | `READY_DISARMED`; acknowledge a prior fault because the VMX gate already proved healthy local OFF |
| `ENABLED` | `ARMED` only if no software-disarm inhibit is active; clear all pre-arm commands |
| `FAULT_LATCHED` | `FAULT`; clear command and output zero |
| missing, malformed, stale, or conflicting source | `FAULT`; clear command and output zero |

After `/robot/disarm` in hardware mode, repeated `ENABLED` samples cannot re-arm
the supervisor. The physical switch must first produce `READY` or
`WAITING_FOR_SAFE_RELEASE`; only a later new `ENABLED` state can arm it.
The same inhibit is active at supervisor startup, so restarting the supervisor
while the VMX gate is already `ENABLED` also requires local OFF followed by ON.

This DDS mirror is not the final authority. Forging it cannot bypass the gate
inside `VmxSystemHardware`, which independently reads physical DIO and zeros or
disables Titan.

## State model and current scope

The tested core implements `BOOTING`, `READY_DISARMED`, `ARMED`, `FAULT`,
`UPDATING`, and `SHUTTING_DOWN`, including latched fault acknowledgement and
the rule that updates begin only while disarmed. Phase 1 connects boot, arm, and
disarm to the ROS runtime. Phase 2 connects hardware boot, arm, disarm, fault,
freshness, source ownership, and local acknowledgement. Update and shutdown
transitions will be connected to the service manager in later phases.

`/robot/arm` is not an authentication boundary. The node rejects it in hardware
mode, even if someone attempts to change the software-arm parameter. Native DDS
discovery or a network service alone cannot satisfy hardware authorization.

## Acceptance evidence

Unit tests cover state transitions, fault latching, update exclusion, malformed
commands, conflicts, deadlines, speed clamps, and acceleration limiting. The
black-box runtime test covers boot-disarmed behavior, explicit simulation arm,
single controller-topic ownership, pre-arm command clearing, publisher loss,
competing publishers, monotonic expiry, and disarm during an active command.
The hardware-only black-box test injects read-only state with no VMX or motor
process and covers boot wait, restart-while-enabled inhibition, local
READY/ENABLED arming, pre-arm clearing, software-disarm inhibit, required local
release, stale fault, local fault acknowledgement, malformed state, and
competing hardware-state publishers.
The joystick-only black-box test uses no physical controller and proves strict
source selection, post-arm release, R1-only rejection, L1+R1 acceptance,
immediate release stop, stale/lost source latching, reconnect inhibition,
publisher conflict rejection, and malformed button rejection.

Do not enable boot services on the physical VMX-pi until the next hardware gate
also proves repeated cold boots with no wheel motion.
