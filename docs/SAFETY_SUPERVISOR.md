# Safety Supervisor API

The safety supervisor is the only node allowed to publish a command to the
profile-selected drive controller. It is enabled in simulation, mock, and
hardware launches. Every start ends in `READY_DISARMED`; publishing `/cmd_vel`
alone cannot move the robot.

This is the Phase 1 production safety boundary. It is validated on the PC and
in simulation. Hardware arming is deliberately unavailable until a local
physical enable, emergency-stop input, and drive-health gate are integrated and
tested on a lifted robot.

## Public interfaces

| Interface | Type | Contract |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Single external planar command input |
| `/robot/state` | `std_msgs/msg/String` | Current state, latched and repeated at 1 Hz |
| `/robot/safety_reason` | `std_msgs/msg/String` | Most recent command or transition decision |
| `/robot/arm` | `std_srvs/srv/Trigger` | Explicitly arm mock/simulation only |
| `/robot/disarm` | `std_srvs/srv/Trigger` | Immediately disable motion and clear the stored command |

The controller-facing `TwistStamped` topic is private. Its name depends on the
selected robot profile. Applications, joystick nodes, Nav2, and remote clients
must never publish to it.

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
- exactly one DDS publisher exists on `/cmd_vel`;
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

## State model and current scope

The tested core implements `BOOTING`, `READY_DISARMED`, `ARMED`, `FAULT`,
`UPDATING`, and `SHUTTING_DOWN`, including latched fault acknowledgement and
the rule that updates begin only while disarmed. Phase 1 connects boot, arm, and
disarm to the ROS runtime. Fault, update, and shutdown transitions will be
connected to hardware diagnostics and the service manager in later phases.

`/robot/arm` is not an authentication boundary. The node rejects it in hardware
mode, even if someone attempts to change the software-arm parameter. The next
hardware phase must provide a robot-local physical authorization path; native
DDS discovery or a network service alone will not satisfy that requirement.

## Acceptance evidence

Unit tests cover state transitions, fault latching, update exclusion, malformed
commands, conflicts, deadlines, speed clamps, and acceleration limiting. The
black-box runtime test covers boot-disarmed behavior, explicit simulation arm,
single controller-topic ownership, pre-arm command clearing, publisher loss,
competing publishers, monotonic expiry, and disarm during an active command.

Do not enable boot services on the physical VMX-pi until the next hardware gate
also proves repeated cold boots with no wheel motion.
