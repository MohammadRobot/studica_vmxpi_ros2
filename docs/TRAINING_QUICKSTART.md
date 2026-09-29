# Immediate classroom handoff

This is the short path for teaching with the existing Ubuntu 22.04 / ROS 2
Humble workstation. It reuses the working course, Gazebo, SLAM Toolbox and Nav2;
it is **not** a production image or authorization to run physical motors.

As clarified on 2026-09-07, the physical E-stop connects only to DIO8. Independent
motor-power removal is absent. Hardware motion commissioning, unattended boot
and student driving of the physical robot remain outside this handoff. Keep
the staged ARM64 release inactive. For the actual robot, follow the separate
[physical training track](PHYSICAL_TRAINING.md): stationary live-sensor lessons
can be prepared independently of Titan control, while moving exercises retain
their hardware acceptance prerequisites. No hardware wiring change is needed
for the simulation lessons below.

## One setup for every classroom terminal

The workspace must already be built following [Installation](INSTALL.md).
In **each** terminal, define this short command:

```bash
export STUDICA_WS="$HOME/studica_ws"
train() { bash "$STUDICA_WS/src/studica_vmxpi_ros2/scripts/training.sh" "$@"; }
```

`train` sources Humble and this workspace automatically. It forces ROS domain
77 and the repository's loopback-only Cyclone DDS configuration after sourcing,
instead of inheriting a hardware peer configuration from another terminal.
It never SSHs to the robot, starts a VMX/Titan process, changes systemd, or
arms a session automatically. This is convenience isolation, not a security
boundary against someone deliberately changing the environment or source.

For the raw `ros2` commands in the course, additionally run:

```bash
source <(train env)
```

Use this setup **instead of** sourcing `studica_sim.env` in these training
sessions. Otherwise course commands may use a different ROS domain. Students
on separate PCs work independently; this launcher does not enable remote DDS.

## Instructor preflight

```bash
train check
```

This checks installed ROS packages and the source contracts without launching
a robot. Add `--dry-run` to any launcher command to inspect its environment and
command without starting processes or creating files.

Run only one simulator at a time. A workspace lock prevents overlapping
`train sim`, `train slam` and `train nav` launches. It cannot stop unrelated
launches started outside this wrapper. Do not mix the two launch methods.
Logs are under `robot_test_results/training-session/log` in the workspace.

## 1. Robot graph, sensors and Python exercises

Terminal 1:

```bash
train sim
```

Terminal 2:

```bash
train status
source <(train env)
ros2 node list --no-daemon
ros2 topic list --no-daemon -t
ros2 topic echo /scan --once
```

Start with [Labs 1–6](COURSE.md). Camera and joystick are off in this fast path.
The simulated robot must remain `READY_DISARMED` until explicitly armed.

For keyboard driving in Terminal 3:

```bash
train arm
train teleop
```

Keyboard defaults are 0.10 m/s and 0.25 rad/s. Follow the displayed keys; `k`
requests stop and `Ctrl+C` exits. The supervisor rejects stale commands. These
are teaching defaults, not a physical-robot safety qualification. Do not run
keyboard teleop alongside Nav2. To stop and switch lessons, exit teleop, run
`train disarm`, then `Ctrl+C` in Terminal 1 and wait for shutdown to finish.

## 2. SLAM and saving a new map

After the previous launch has stopped, Terminal 1:

```bash
train slam
```

Terminal 2:

```bash
train status
train arm
train teleop
```

Drive the simulated office slowly, as described in [Lab 7](labs/07_slam.md).
Exit teleop and disarm before saving, but leave the SLAM launch running:

```bash
train disarm
train save-map lesson7
```

The new files are `project_maps/training/lesson7/map.yaml` and `map.pgm`.
Existing map directories are never overwritten; choose another name for each
save, including after a failed/partial save. The existing physical
`project_maps/office_nav.yaml` and image remain untouched.

After saving, stop Terminal 1 with `Ctrl+C` before starting navigation.

## 3. Navigation

First use the bundled simulated-office map:

```bash
train nav
```

To use the map saved above, stop the previous launch and instead run:

```bash
train nav lesson7
```

In another prepared terminal run `train status`. In RViz, set **2D Pose
Estimate**, verify scan/map alignment, then explicitly run `train arm` before
sending a nearby **Nav2 Goal**. Follow [Lab 8](labs/08_navigation.md), but skip
the depth-camera checks in this fast path: camera/point cloud are off here.
Do not start teleop while navigation is active. Cancel goals in RViz, run
`train disarm`, then stop the launch before changing modes.

The physical `office_nav` map describes a different environment; do not use it
as the simulated-office baseline. Its real-robot localization/navigation
acceptance remains pending.

`train status` checks the robot base, not localization or the complete Nav2
lifecycle. Without an initial pose, the planner can remain waiting for the
`map` transform even when all eight base checks pass. An abort was observed
when interrupting that unlocalized planner; see the verification record. This
shutdown case remains an open issue, so rehearse navigation before class.

## If graphics are unavailable

Append `--headless` to `sim`, `slam` or `nav` to disable Gazebo's window and
RViz. ROS sensors and computation still run. This is useful for automated
checks, but does not by itself complete the graphical localization/goal lab.
The ordinary commands above are the instructor-facing graphical path.

## What counts as ready for class

Before students arrive, rehearse on each classroom PC:

- `train check` succeeds and `train status` reports a healthy simulator;
- Gazebo/RViz display correctly and keyboard input reaches the intended terminal;
- explicit arm, teleop, stop, disarm and complete shutdown work;
- a deliberately explored map saves as a YAML/PGM pair;
- localization, one nearby goal and goal cancellation work on the matching map;
- the Python starters/reference solutions in [the course](COURSE.md) run;
- no classroom process is connected to physical motor control.

Automated results and remaining manual checks are recorded in
[the local verification record](TRAINING_VERIFICATION_2026-09-07.md). Do not
describe a passed launch check as a completed navigation course or a passed
physical safety qualification. The longer-term appliance work remains tracked
in [Production platform](PRODUCTION_PLATFORM.md) and
[Safety acceptance](SAFETY_ACCEPTANCE.md).
