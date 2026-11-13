# Two simultaneous simulation sessions — 2026-11-13

Result: **PASS for the observed local simulation run**. Two actual headless
Gazebo Harmonic instances ran concurrently through the standard `sim.launch.py`
entry point, with joystick, camera and RViz disabled. Both used the same robot
profile, world and ROS topic/frame names. No robot or RoboHiTec project was accessed.

## Isolation and positive controls

- Both processes shared one newly created network namespace containing only
  loopback. This excluded the host LAN and physical robots while allowing a
  meaningful cross-domain test on the same network.
- The current profile generator created separate loopback DDS profiles with
  domains 20 and 21. Each Gazebo instance used a different `GZ_PARTITION`.
- Both robots reached READY_DISARMED and were explicitly armed in simulation.
- Each domain received its own marker on the same `/isolation_marker` topic and
  never received the other domain's marker.
- `/scan`, `/odom` and `/clock` each had exactly one publisher per domain;
  publisher GID sets were disjoint across domains, including `/tf`.
- Sensor, clock, odometry and TF messages continued arriving in both domains
  during both command phases; absence of crossover was not caused by a dead robot.

## Measured command results

Each phase published forward velocity 0.10 m/s for five seconds. Displacement
was measured from controller odometry. Acceptance required commanded displacement
above 0.03 m, idle displacement below 0.02 m, more than 20 locally received command
samples, and exactly zero command samples in the other domain. These are test
criteria for isolation, not changes to robot safety limits.

| Command target | Domain 20 displacement | Domain 21 displacement | Command samples in 20 / 21 |
|---|---:|---:|---:|
| Domain 20 | 0.486500 m | effectively 0 m (4.64e-18) | 81 / 0 |
| Domain 21 | 0 m | 0.487594 m | 0 / 82 |

The first robot was disarmed after its commanded phase. The second phase therefore
also relied on direct `/cmd_vel` observation to detect crossover, not merely on the
first robot remaining stationary. End-of-run telemetry counts were 193/192 scans,
339/334 odometry messages, 403/401 clocks and 373/368 TF messages.

All 24 launch-recorded child process IDs were absent after cleanup. Logs may show
SIGINT/exit -2 messages during teardown; the test completed its assertions before
stopping both simulation process groups.

## Evidence and reproduction

Raw local evidence is under
`/home/mrobot/studica_ws/log/two_simulations_20261113_c/`: `result.json`, generated
DDS profiles, per-session ROS logs and `launch_20.log`/`launch_21.log`.
The source fixture is `test/test_two_simulations.py`. It refuses execution unless
it sees only loopback and a network namespace different from the caller's recorded
namespace. It is intentionally not an unrestricted automatic hardware test.

From the workspace root, choose a **new output directory** on every run:

```bash
cd ~/studica_ws
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  STUDICA_PARENT_NETNS="$(readlink /proc/self/ns/net)" \
  unshare --user --map-root-user --net bash --noprofile --norc -c '
    ip link set lo up
    source /opt/ros/humble/setup.bash
    source /home/mrobot/studica_ws/install/local_setup.bash
    python3 /home/mrobot/studica_ws/src/studica_vmxpi_ros2/test/test_two_simulations.py \
      --output /home/mrobot/studica_ws/log/two_simulations_repeat
  '
```

Adapt workspace paths for another PC. This requires an already built Humble/Gazebo
workspace and user network namespace support. The test launches simulators and
publishes simulation commands; it is not a read-only ROS discovery command.

Earlier harness attempts encountered namespace permission/global-executor handling
and a duplicate simulation arm request. Those were corrected without relaxing
assertions or changing runtime safety settings. Failed evidence directories were
retained. The final fixture passed Python lint and `git diff --check`.

## Limits

This establishes domain and Gazebo-partition isolation for two local sessions over
the observed interval. It does not qualify physical Wi-Fi/Ethernet, malicious
publishers, shared-domain namespaces, hardware stopping, SLAM/Nav2 concurrency,
or long-duration performance. Separate domains remain mandatory for independent
beginner sessions; this test does not authorize a physical deployment.


For the subsequent concurrent SLAM/map-saving and Nav2 goal tests, see
[dual SLAM and navigation acceptance](TWO_SLAM_NAVIGATION_ACCEPTANCE.md).
