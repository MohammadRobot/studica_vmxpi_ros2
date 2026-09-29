# Local training verification — 2026-09-07

Scope: the existing Ubuntu 22.04 / ROS 2 Humble workstation, Gazebo Harmonic
8.15.0, `class_4wd`, the current installed workspace and the new source-checkout
`scripts/training.sh`. No VMXPi connection, hardware activation, service
installation, GitHub publication or production qualification was performed.

The operator requested a fast classroom handoff with the current hardware.
The DIO8-only E-stop remains a recorded hardware limitation, not a passed gate.
This handoff covers the [isolated simulation fast path](TRAINING_QUICKSTART.md).

## Automated results

| Check | Result |
|---|---|
| Training launcher contracts | 19 passed: simulation-only commands, explicit arm/disarm, loopback profile, map names, no dry-run writes, low keyboard defaults |
| Platform model, map registry, pairing, signed updater and Python example unit tests | 18 passed |
| Web pairing/authentication and WebSocket CSRF mock integration | 1 passed on a temporary localhost server |
| `test_runtime_contract.py --mode gz` | Passed: graph, disarmed startup, explicit simulated arm, stale command rejection, competing sources, feedback, timeout and disarm |
| `test_runtime_contract.py --mode mapping` | Passed: headless startup, graph, disarmed startup and command ownership |
| `test_runtime_contract.py --mode navigation` | Passed: headless startup, graph, disarmed startup and sole smoothed command publisher, including controller/recovery pre-smoother path |
| ShellCheck and launcher test Flake8 | Passed |
| `train check` source/dependency preflight | Passed, including all six profiles and links in 44 Markdown documents |

The first sandboxed simulator attempt could not write the default ROS log
directory. A retry with a writable log directory exposed the sandbox's local
socket restriction. The three runtime checks above passed when run with
permission to open local sockets and the repository's explicit loopback-only
Cyclone DDS profile. These earlier environment failures are not claimed as
passing runs. No physical robot was substituted for the simulator.

## Actual launcher exercise

Used the new wrapper, not just its dry-run output:

1. `slam --headless` started the office simulation and SLAM.
2. `status` reported **8 PASS, 0 WARN, 0 FAIL**. `/scan` was 10.0 Hz;
   odometry, IMU alias and joint-state streams were approximately 100 Hz.
3. A concurrent `nav --headless` was rejected by the workspace session lock.
4. `save-map smoke_20260907` saved a **93 × 140**, **0.07 m/pixel** initial map
   without arming or driving the simulator.
5. Repeating that map name was rejected; before/after SHA-256 digests matched.
6. `disarm` returned success, reporting the simulator was already disarmed.
7. Stopped SLAM with Ctrl+C and waited for its launch to exit.
8. `nav smoke_20260907 --headless` started successfully after the lock released.
9. Both `/map_server` and `/controller_server` were lifecycle `active`.
   `/map_server.yaml_filename` resolved to the saved training YAML, and
   `/robot/state` was `READY_DISARMED`.
10. Navigation `status` also reported **8 PASS, 0 WARN, 0 FAIL**.

The smoke map is under `project_maps/training/smoke_20260907` in the workspace:

- `map.yaml`: `fdab667760c70e6dd4b625947b63cfe679612edd6c9fac7a3c5529f51d710ec4`
- `map.pgm`: `9315f946f1c0c4200a94e85945e3bff5a722cc9ed5f9d8043a95ff9406b055b4`

This is an initial stationary scan map, **not a completed mapping exercise**.
The physical `project_maps/office_nav.yaml` and PNG were not modified.
Wrapper launch logs are in `robot_test_results/training-session/log`.

### Open navigation shutdown issue

No initial pose or navigation goal was provided in the smoke-map round trip.
The planner remained waiting for `map -> base_link`, so the complete navigation
lifecycle was **not** established by the map-server/controller-server or base
health checks. On Ctrl+C, `planner_server` exited with signal 6 (abort) during
shutdown; the navigation lifecycle manager subsequently reported its failed
state request. Gazebo exited on the requested interrupt. The launch exited
and a process check found no remaining simulator/launch/supervisor processes.

This is not a clean Nav2 shutdown pass. Preserve the logs and reproduce/fix
the unlocalized shutdown case before calling the full navigation lesson
qualified. The wrapper explicitly warns that base health does not establish
Nav2 readiness. Physical services were not affected.

## Remaining instructor checks

- Rehearse graphical Gazebo/RViz and keyboard focus on each student PC.
- Explore the complete simulated office, save a map, localize against it,
  complete a nearby goal, and demonstrate goal cancellation.
- Rehearse the Python labs and cleanup sequence with the student environment.
- Web UI operation against a live platform/companion, real `office_nav`
  navigation, camera performance and hardware fail-safe behavior are not
  established by this simulation handoff or the mock web test.
- Production image/autostart, independent hardware torque removal and the
  full cold-boot/failure-injection qualification remain open.
