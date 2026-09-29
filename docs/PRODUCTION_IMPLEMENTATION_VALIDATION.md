# Production implementation validation — 2026-09-29

Source baseline: `d3861cacc8c9b7e3bc515f3306408bd4cc00b63b` plus the local
production-session changes. Hardware dependency checkouts match `hardware.repos`:
`studica_drivers` at `0b45ad72aeaceeff2ed47e4429eac140d54a9d5c` and
`studica_robot_monitor` at `745a5150dec5e4a40e16cab28c842963c22db2f4`.
This record is software validation on the PC, not hardware acceptance.

## Verified

Final consolidated CTest run: **36/36 checks passed** in 152 seconds, including
all three headless simulation contracts (none skipped). The grouped platform
Python check passed 46 cases.

- ROS Humble symlink build of drivers, monitor and platform succeeds. The PC
  uses stub drivers because VMXPi headers/libraries are unavailable here.
- Static project contracts, documentation links, profile validation, release
  contracts and linters pass.
- Three simultaneous loopback DDS sessions isolate telemetry and `/clock`.
- Mock runtime, hardware-state supervisor, joystick supervision and mode-manager
  runtime tests pass. Mode-manager coverage includes command expiry, no motion
  after publisher return, and maintenance-mode rejection/disarm.
- Platform tests cover named companion separation, domain firewall ranges,
  provisioning persistence, HTTPS authorization, signed archive integrity,
  qualification digest binding, rollback and interrupted-activation journals.
  Activation fault injection uses temporary files and mocked systemctl calls;
  it does not certify real power-failure behavior.
- Headless Gazebo, mapping and navigation runtime contracts pass.
- The update wrapper runs `--help` from an empty environment with only HOME,
  PATH and a test release root. No hardware or host services were started.

The PC lacked the declared `python3-aiohttp` dependency. Tests used an isolated
copy under `/tmp/studica-platform-test-deps`; production images must install the
package dependencies. The prototype made before fetching the pushed platform
is preserved in a Git stash and is not part of these changes.

## Physical qualification still required

No physical robot was moved, no live services were installed/enabled, and no
release was activated. The hardware-specific plugin, ARM64 artifact, real
network isolation, cold boot without networking, local-enable recovery,
power-loss update recovery and 24-hour resource workload require pilot testing.
Record the exact tested artifact digest rather than treating this source commit
or PC tests as that evidence. Preserve the earlier physical results in
[Hardware safety gate](HARDWARE_SAFETY_GATE.md).

Use [Product runtime](PRODUCT_RUNTIME.md) for provisioning, target selection,
updates and rollout. This implementation keeps the existing hardware rate and
production qualification gates.
