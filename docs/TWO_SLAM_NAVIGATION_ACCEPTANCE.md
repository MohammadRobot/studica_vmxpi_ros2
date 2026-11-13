# Concurrent SLAM and navigation acceptance — 2026-11-13

This test uses two real headless Gazebo Harmonic instances, ROS 2 Humble, the
`class_4wd` profile and `office_map` world. It runs two SLAM sessions concurrently,
saves their maps, stops those sessions, then runs two navigation sessions
concurrently using those saved maps. SLAM and AMCL are not run simultaneously
for the same robot.

## Observed result

**PASS** on the final run in
`/home/mrobot/studica_ws/log/two_slam_navigation_20261113_b/`.
Both sessions published 36 SLAM map updates, saved different maps, and then
completed their distinct navigation goals with status SUCCEEDED. Saved-map grid
hashes exactly matched the corresponding received navigation map hashes.
Only `isolation_20` was seen in domain 20; only `isolation_21` in domain 21.
Each action status stream contained exactly its own accepted UUID.

| Session | Goal in map frame | Final odometry position | Action result |
|---|---|---|---|
| Domain 20 | (0.45, 0.00) m | (0.219, -0.007) m | SUCCEEDED |
| Domain 21 | (-0.45, 0.00) m | (-0.313, -0.189) m | SUCCEEDED |

Final positions are odometry measurements, not independent ground-truth map-frame
accuracy measurements. The action result uses Nav2's configured goal tolerance.
All 72 launch-recorded child process IDs were absent after cleanup. See
`result.json` for complete hashes, publisher identities, goal UUIDs and TF marker
observations, and `cleanup.json` for the process check.

The first run (`two_slam_navigation_20261113_a`) also passed; saved-map content
was verified separately afterward. The final run incorporates content verification
and post-goal TF ownership checks directly into the reusable fixture. Python lint,
documentation links and `git diff --check` passed.

## Test boundaries

Both sessions share one isolated network namespace containing only loopback.
Domains are 20 and 21; Gazebo transport partitions are distinct per session and
phase. Both retain identical ROS topic, frame and action names, including `/map`,
`map`, `odom`, `/tf` and `/navigate_to_pose`. No host network, physical robot,
joystick or RoboHiTec project is used. No runtime safety limit is changed.

The fixture uses standard `mapping.launch.py mode:=gz_sim` and
`navigation.launch.py mode:=gz_sim`, with GUI/joystick disabled and navigation
point clouds disabled. The maps come from live SLAM; they are not synthetic
sentinel maps or copies of the bundled navigation map.

## Assertions

- Both simulators reach READY_DISARMED with sensor, TF, clock and odometry data.
- Simulated movement in different directions produces distinct occupancy maps;
  both SLAM nodes publish multiple map updates, and each map saves successfully.
- `/map` and `/clock` have exactly one publisher in each domain. Their publisher
  GIDs, and the `/tf` publisher sets, are disjoint across domains.
- Both sessions receive advancing `map -> odom` transforms. Each also receives
  only its own distinct TF marker, a leaf named `isolation_20` or `isolation_21`
  that does not affect robot localization.
- Nav2's received map hash matches the decoded occupancy grid of its own saved
  PGM/YAML file, including the image-to-map row reversal and trinary thresholds.
  The two maps differ, making a map swap detectable. Each remains unchanged
  throughout navigation.
- Different NavigateToPose goals are accepted concurrently: `(0.45, 0)` in domain
  20 and `(-0.45, 0)` in domain 21, in each session's `map` frame.
- Both goals must return action status 4 (SUCCEEDED), both robots must move more
  than 0.10 m, and each action status stream must contain only its own goal UUID.
- Map/TF/clock publisher ownership and TF markers are checked again after both
  goals complete, not only before navigation starts.

Goal success uses the configured Nav2 tolerances; it does not mean the final pose
is exactly the requested coordinate. The TF marker checks supplement actual
map-to-odom and publisher ownership checks; they do not replace them.

## Reproduction

The standalone fixture is `test/test_two_slam_navigation.py`. It needs the existing
built Humble/Gazebo/Nav2/SLAM workspace, Python Pillow/PyYAML, and user network
namespace support. It refuses to run on the caller's host network or with any
interface other than loopback. Choose a new output directory for every run:

```bash
cd ~/studica_ws
env -i HOME="$HOME" USER="$USER" PATH=/usr/bin:/bin LANG=C.UTF-8 \
  STUDICA_PARENT_NETNS="$(readlink /proc/self/ns/net)" \
  unshare --user --map-root-user --net bash --noprofile --norc -c '
    ip link set lo up
    source /opt/ros/humble/setup.bash
    source /home/mrobot/studica_ws/install/local_setup.bash
    python3 /home/mrobot/studica_ws/src/studica_vmxpi_ros2/test/test_two_slam_navigation.py \
      --output /home/mrobot/studica_ws/log/two_slam_navigation_repeat
  '
```

Adapt absolute workspace paths on another PC. This starts simulation and sends
simulation motion commands/goals. It is not an unrestricted discovery-only test.
The output contains JSON results, DDS profiles, launch logs, map-saver logs,
ROS logs and the two saved PGM/YAML maps. Failed runs retain their evidence.

## Scope of the result

This qualifies only the observed local simulation isolation and short navigation
routes. It does not qualify physical networks, hardware stopping, full-building
map quality, long-duration performance, shared-domain namespacing or hostile
publishers. No deployment, robot access, commit, push or RoboHiTec change is part
of this test.
