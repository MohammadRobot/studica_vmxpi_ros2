# Changelog

## Unreleased

- Selected Ubuntu 22.04 arm64 with ROS 2 Humble as the Phase-1 VMXPi
  production baseline and documented its May 2027 lifecycle gate.
- Added a validated minimal runtime package manifest that keeps Bluetooth
  joystick support while excluding desktop, simulation, GUI, and build tooling.
- Added a read-only, machine-readable VMXPi production runtime audit with a
  versioned minimal-service/package/port/SSH profile, contract tests, a measured
  physical-robot idle baseline, and a staged appliance-image migration plan.
- Made the safety supervisor timer fail closed on live ROS graph errors and
  exit cleanly when shutdown invalidates the ROS context mid-callback.
- Locked first-party dependency manifests and CI checkouts to immutable commits,
  pinned GitHub Actions, and added an offline release-contract check.
- Replaced the installer's mutable ROS repository setup with a checksum-verified
  `ros2-apt-source` package and retry-bounded APT operations.
- Added the production safety architecture, release process, security,
  contribution, and support policies for the pre-production phase.
- Added a developer quick-start page covering installation, simulation, the
  edit/build loop, supervised VMXPi bringup, two-computer physical mapping,
  map saving, and simulation or two-computer physical navigation.
- Added `navigation.launch.py mode:=hardware` for PC-side map server, AMCL,
  Nav2, and RViz operation against the VMXPi. The confirmed physical default is
  0.20 m/s linear and 0.35 rad/s angular, with launch-time overrides bounded at
  0.30 m/s and 0.60 rad/s.
- Added an automatically selected physical-robot SLAM profile for the YDLidar
  X2 with full scan throughput, closer pose spacing, 5 cm cells, calibrated
  odometry-first scan placement, strict return-to-start loop closure, and lower
  joystick mapping speeds; simulation retains its existing profile.
- Refined the `class_4wd` symmetric rolling-radius multiplier to `1.0173`
  using matched 20-second forward and reverse measured floor runs.
- Added a two-launch physical mapping workflow: `robot.launch.py` owns hardware
  on the VMXPi while `mapping.launch.py mode:=hardware` starts only PC-side
  SLAM, map-frame RViz, and conservative deadman joystick teleoperation.
- Added an opt-in physical point-cloud path to `robot.launch.py` with a tested
  Gemini E 320×240×5 depth-only profile, hardware point stride, explicit color
  opt-in, and sustained VMXPi thermal/throttling guidance.
- Corrected VMX `/imu` linear acceleration to use REP-145 sensor-frame raw
  accelerometer data in m/s^2 instead of labeling world-frame acceleration as
  `imu_link` data.

- Replaced the approximate `class_4wd` dimensions with confirmed physical body,
  wheelbase, track, clearance, mass, and sensor poses shared by hardware and
  simulation; added a profile-derived Nav2 footprint and retained separately
  validated drivetrain calibration.
- Added a hardware-only 25 Hz control/odometry default, validated across all
  eight guarded wheel directions with controller recovery and no active VMXPi
  throttling, while preserving 100 Hz simulation profiles; also added a
  complete `robot_check --skip-lidar` mode for deliberate motor-only bringup.
- Isolated generated controller YAML files by process so a root hardware launch
  cannot block later normal-account builds or checks through `/tmp` ownership.
- Normalized the confirmed Titan 2.0.5 Fahrenheit `MCU_TEMP` firmware payload
  to Celsius while retaining fail-safe handling for unknown firmware versions.
- Prevented intermittent read-only TF-check failures when multiple
  transient-local `/tf_static` publishers replay retained transforms together.
- Added an optional simulated depth-to-`PointCloud2` pipeline, ground-frame
  floor/range/body filter, raw and filtered RViz displays, focused tests, and a
  read-only application observer example.
- Integrated filtered depth obstacles into the Nav2 local costmap, with raw
  optical rays used only to clear stale costs and a launch-time overlay that
  preserves complete custom Nav2 parameter files.
- Generalized learner-facing names across packages, examples, topics, and
  generated artifact paths; application examples now use `/apps/...`, while
  saved maps and routes use `project_maps` and `project_routes`.
- Added terminal-scoped Cyclone DDS profiles for local simulation and explicit
  PC-to-VMXPi Wi-Fi/Ethernet peers.
- Selected Gazebo Harmonic in the generated simulation environment and
  documented copied-versus-symlink Colcon build consistency.
- Started L1-deadman DualShock teleoperation by default in simulation and
  mapping while keeping navigation and hardware opt-in.
- Added the mapping/save/navigation workflow, colored global/local Nav2
  costmaps, and a reviewed explicit-start office waypoint route.
- Updated Labs 1–8 so every simulation terminal sources the same workspace and
  DDS environment.

## 0.1.0 - 2026-03-18

- Added Apache-2.0 project licensing files (`LICENSE`, `NOTICE`) and SPDX headers.
- Simplified bringup launch structure for readability and easier profile-driven workflows.
- Improved controller spawning behavior in Gazebo Sim to avoid repeated active-state retries.
- Added project validation tooling (`scripts/check_project.sh`) and profile validation integration.
- Updated simulation/navigation documentation and RViz defaults for general workflows.
