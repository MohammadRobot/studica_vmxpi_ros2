# 3-Day Robotics Workshop: Studica Robot and ROS 2

This workshop is a practical introduction to mobile robotics using the Studica VMXPi ROS 2 platform. It is designed for students, teachers, robotics club members, and beginner developers who want to understand how a real ROS 2 robot works through simulation first, then supervised hardware.

The workshop uses this repository: `studica_vmxpi_ros2`.

## Workshop summary

| Item | Details |
|---|---|
| Duration | 3 days |
| Format | Instructor-led, hands-on, simulation-first |
| Platform | Studica VMXPi robot with ROS 2 |
| Software | Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, RViz, Python 3 |
| Robot profiles | `class_4wd` for simulation, `stack_4wd` for supervised hardware |
| Main ROS interface | `/cmd_vel` using `geometry_msgs/msg/Twist` |
| Final outcome | A working ROS 2 mobile robot project: teleoperation, sensor visualization, simple autonomy, mapping, and navigation demo |

## Target audience

This workshop is suitable for:

- beginners learning ROS 2 for the first time;
- students preparing for robotics competitions;
- teachers and trainers who need a structured classroom path;
- robotics club members building a practical mobile robot foundation;
- engineers who are new to ROS 2 and want a safe hands-on introduction.

No advanced Linux or ROS 2 experience is required, but participants should be comfortable using a computer and following terminal commands.

## Learning objectives

By the end of the workshop, participants will be able to:

- explain the basic ROS 2 concepts: nodes, topics, messages, services, parameters, launch files, and TF;
- install and source a dedicated ROS 2 workspace for the Studica robot;
- launch the Studica robot in simulation;
- inspect ROS 2 nodes, topics, message types, and frame transforms;
- safely command robot motion through `/cmd_vel`;
- visualize odometry, IMU, LiDAR, TF, and robot model data in RViz;
- write simple Python ROS 2 publisher and subscriber nodes;
- understand the difference between simulation, mock mode, and real hardware;
- create and save a map with SLAM Toolbox;
- send basic Nav2 goals in simulation;
- run supervised hardware readiness checks before touching the real robot.

## Workshop philosophy

The workshop follows four rules:

1. **Simulation first.** Participants learn, test, and debug in simulation before using the real robot.
2. **One public motion interface.** All application movement commands use `/cmd_vel`; participants do not publish directly to controller-internal topics.
3. **Safety is part of the lesson.** Hardware motion is allowed only after instructor approval, a clear test area, and an emergency stop check.
4. **Evidence-based learning.** Every module has a visible checkpoint: screenshots, terminal output, saved maps, or working code.

## Required equipment

### For each participant or team

- Laptop or workstation with Ubuntu 22.04 LTS;
- ROS 2 Humble installed through the workshop setup process;
- reliable internet connection during setup;
- Git installed;
- optional DualShock 4 or compatible joystick for teleoperation;
- access to this repository.

### For instructor demo and hardware day

- Studica VMXPi robot running the supported hardware profile;
- physical emergency stop;
- clear test area;
- four-wheel lift or safe support stand for initial checks;
- spare battery and charger;
- HDMI display or projector;
- network access for robot and instructor laptop.

## Pre-workshop preparation

Participants should install the project in a dedicated workspace:

```bash
export STUDICA_WS="$HOME/studica_ws"
mkdir -p "$STUDICA_WS/src"
git clone https://github.com/MohammadRobot/studica_vmxpi_ros2.git \
  "$STUDICA_WS/src/studica_vmxpi_ros2"
cd "$STUDICA_WS/src/studica_vmxpi_ros2"
./scripts/setup_ubuntu.sh --mode simulation
```

Each new simulation terminal should source the environment:

```bash
export STUDICA_WS="$HOME/studica_ws"
source "$HOME/.ros/studica_sim.env"
source /opt/ros/humble/setup.bash
source "$STUDICA_WS/install/setup.bash"
```

The instructor should run the setup once before the workshop and confirm that simulation launches correctly.

## Day 1 — ROS 2 foundations and safe robot control

**Theme:** Understand ROS 2, launch the Studica robot in simulation, and control motion safely.

### Day 1 outcomes

Participants will finish Day 1 with a working simulation, basic ROS 2 terminal skills, and a clear understanding of safe motion through `/cmd_vel`.

### Session plan

| Time | Module | Activity | Output |
|---|---|---|---|
| 09:00–09:30 | Welcome and safety briefing | Explain workshop goals, robot safety, emergency stop, and simulation-first workflow | Signed safety understanding |
| 09:30–10:30 | ROS 2 basics | Nodes, topics, messages, graph, packages, workspaces | ROS vocabulary worksheet |
| 10:30–10:45 | Break |  |  |
| 10:45–12:00 | Environment setup | Workspace setup, sourcing, package build, validation commands | Working `studica_ws` |
| 12:00–13:00 | Lunch |  |  |
| 13:00–14:15 | Launch simulation | Run `sim.launch.py`, open Gazebo and RViz | Simulation running |
| 14:15–15:00 | Inspect the ROS graph | `ros2 node list`, `ros2 topic list`, `ros2 topic info`, `ros2 topic echo` | Node/topic sketch |
| 15:00–15:15 | Break |  |  |
| 15:15–16:30 | Command robot motion | Publish safe `/cmd_vel`, use joystick or keyboard teleop, stop motion | Annotated `Twist` command |
| 16:30–17:00 | Checkpoint and review | Common errors, cleanup, questions | Day 1 checkpoint evidence |

### Day 1 hands-on tasks

1. Launch the robot simulation:

```bash
ros2 launch studica_vmxpi_ros2 sim.launch.py
```

2. Inspect the system:

```bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /odom --once
```

3. Publish a simple motion command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

4. Stop, clean up, and confirm no command publisher is still running.

### Day 1 checkpoint

Each team must submit:

- screenshot of Gazebo and RViz;
- list of at least five active ROS 2 topics;
- one annotated `/cmd_vel` command;
- short explanation of why hardware should not be used before simulation checks.

## Day 2 — Sensors, TF, RViz, and Python ROS 2 nodes

**Theme:** Understand robot perception data and write simple ROS 2 application nodes.

### Day 2 outcomes

Participants will visualize sensor data, understand coordinate frames, and create a basic Python node that reacts to robot data.

### Session plan

| Time | Module | Activity | Output |
|---|---|---|---|
| 09:00–09:20 | Day 1 recap | Review `/cmd_vel`, topics, safe stop process | Refreshed workflow |
| 09:20–10:30 | Sensor topics | LiDAR, odometry, IMU, joint states, diagnostics | Sensor topic table |
| 10:30–10:45 | Break |  |  |
| 10:45–12:00 | RViz and TF | Robot model, frames, `odom`, `base_link`, sensor frames | TF screenshot |
| 12:00–13:00 | Lunch |  |  |
| 13:00–14:30 | Python publisher | Create a simple movement node using `rclpy` | Working publisher |
| 14:30–15:15 | Python subscriber | Subscribe to `/scan` or `/odom` and print useful information | Working subscriber |
| 15:15–15:30 | Break |  |  |
| 15:30–16:30 | Mini challenge | Build a safe obstacle response behavior in simulation | Simple autonomy demo |
| 16:30–17:00 | Checkpoint and review | Debugging, package structure, next steps | Day 2 checkpoint evidence |

### Day 2 hands-on tasks

1. Check sensor streams:

```bash
ros2 topic hz /scan
ros2 topic echo /imu --once
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo odom base_link
```

2. Create or copy a Python package from the starter examples.

3. Write a publisher that sends a slow forward command for a short duration, then stops.

4. Write a subscriber that reads `/scan` and reports the nearest obstacle distance.

5. Combine publisher and subscriber logic in simulation:

- move forward slowly;
- if an obstacle is too close, stop;
- never exceed instructor-approved speed limits.

### Suggested mini-project: obstacle stop node

The node should:

- subscribe to `/scan`;
- publish to `/cmd_vel`;
- move forward at low speed when clear;
- stop when the front LiDAR range is below the threshold;
- print a simple status message.

### Day 2 checkpoint

Each team must submit:

- RViz screenshot showing robot model, LiDAR, and TF;
- terminal output from `/scan` or `/odom`;
- Python node file or package name;
- short demo of obstacle stop behavior in simulation.

## Day 3 — SLAM, Nav2, and supervised hardware readiness

**Theme:** Build a map, run navigation in simulation, and learn how to approach real hardware safely.

### Day 3 outcomes

Participants will create a map, run a basic navigation demo, and understand the required checks before using the physical Studica robot.

### Session plan

| Time | Module | Activity | Output |
|---|---|---|---|
| 09:00–09:20 | Day 2 recap | Review sensors, TF, and Python nodes | Ready for autonomy |
| 09:20–10:45 | SLAM concepts | Mapping, odometry, LiDAR, map quality, loop closure | SLAM concept notes |
| 10:45–11:00 | Break |  |  |
| 11:00–12:00 | Create and save a map | Launch mapping, drive robot, save map | PGM/YAML map pair |
| 12:00–13:00 | Lunch |  |  |
| 13:00–14:30 | Navigation with Nav2 | Localization, costmaps, goal sending, recovery behavior | Nav2 goal demo |
| 14:30–15:15 | Capstone project | Teams combine teleop, sensor checks, map, and goal | Team demo plan |
| 15:15–15:30 | Break |  |  |
| 15:30–16:15 | Supervised hardware readiness | Instructor demo: health check, lift test, emergency stop, low-speed floor test | Hardware checklist |
| 16:15–17:00 | Final demos and review | Team presentations and next learning path | Completion evidence |

### Day 3 hands-on tasks

1. Start mapping using the repository mapping launch file.

2. Drive carefully and observe map quality.

3. Save the generated map.

4. Launch navigation using the saved map.

5. Send a simple goal in RViz.

6. Observe localization, costmaps, and path planning behavior.

7. Discuss why a simulation success does not automatically approve hardware motion.

### Hardware readiness checklist

Before real robot motion, the instructor must confirm:

- Labs or equivalent workshop checkpoints are completed;
- emergency stop is visible, reachable, and tested;
- robot is initially supported on a safe lift if required;
- battery is secured;
- wheels and wiring are clear;
- network connection is stable;
- read-only robot health check passes;
- no participant stands near the wheels;
- first motion test is low speed and short duration;
- stop conditions are agreed before motion starts.

## Capstone project

Each team will complete a short final demo:

**Mission:** Use the Studica ROS 2 robot in simulation to inspect the environment, create or use a map, and navigate to a target safely.

### Minimum requirements

- Launch simulation successfully;
- show `/cmd_vel`, `/odom`, `/scan`, and TF data;
- run at least one custom Python node;
- create or load a map;
- send one navigation goal;
- explain safety steps before real hardware use.

### Optional extensions

- add obstacle stop behavior;
- create a waypoint route;
- compare joystick, keyboard, and Nav2 motion;
- enable camera topics if the computer can handle the load;
- prepare a short competition-style maze challenge.

## Assessment plan

| Area | Weight | Evidence |
|---|---:|---|
| ROS 2 basics | 20% | Correct explanation of nodes, topics, messages, launch, and TF |
| Simulation operation | 20% | Successful launch, topic inspection, and safe cleanup |
| Motion control | 15% | Correct use of `/cmd_vel` and stop behavior |
| Sensors and visualization | 15% | RViz, `/scan`, `/odom`, `/imu`, and TF evidence |
| Python application | 15% | Working publisher/subscriber or obstacle stop node |
| SLAM/Nav2 demo | 10% | Saved map or navigation goal evidence |
| Safety awareness | 5% | Hardware readiness explanation |

## Instructor preparation checklist

- [ ] Test `./scripts/setup_ubuntu.sh --mode simulation` on a clean Ubuntu 22.04 machine.
- [ ] Confirm `sim.launch.py` runs in Gazebo and RViz.
- [ ] Prepare a projector demo laptop.
- [ ] Prepare printed or digital safety rules.
- [ ] Prepare one working joystick if joystick teleoperation will be used.
- [ ] Prepare the physical Studica robot only for Day 3 supervised demonstration.
- [ ] Test emergency stop before participants arrive.
- [ ] Prepare a clear low-speed hardware test area.
- [ ] Prepare sample answers for each checkpoint.

## Participant deliverables

At the end of the workshop, each participant or team should have:

- working ROS 2 Studica workspace;
- screenshots of simulation, RViz, TF, and sensor data;
- basic Python ROS 2 node;
- map file or navigation evidence;
- completed safety checklist;
- understanding of the next learning path toward advanced robotics projects.

## Recommended next steps after the workshop

Participants can continue with:

1. completing the full lab path in `docs/COURSE.md`;
2. improving the Python obstacle stop node;
3. building a waypoint navigation challenge;
4. learning robot localization and sensor fusion in more detail;
5. testing hardware only under instructor supervision;
6. preparing competition tasks such as line following, maze solving, or autonomous delivery.

## Notes for RoboHiTec delivery

For a professional RoboHiTec workshop proposal, this GitHub workshop can be packaged as:

- **Beginner package:** simulation-only, 3 days;
- **Standard package:** simulation plus instructor hardware demonstration;
- **Advanced package:** simulation, hardware readiness, mapping, Nav2, and mini competition.

Recommended class size: 12–20 participants, working in teams of 2–3 per robot or simulation station.
