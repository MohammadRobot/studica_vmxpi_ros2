# Corrected ARM64 build attempt: 2026-09-06

This record covers an interrupted corrected development build. No corrected
artifact was produced or staged. It does not authorize activation, systemd
autostart, or robot motion. It follows
the [commissioning preflight](COMMISSIONING_PREFLIGHT_2026-09-06.md), which found
that the previously staged release omitted the Titan heartbeat correction.

## Approved source inputs

The operator approved publication of the two corrected dependency commits.
Both pushes succeeded and GitHub `main` was independently read back at:

| Repository | Published commit |
|---|---|
| `studica_drivers` | `0b45ad72aeaceeff2ed47e4429eac140d54a9d5c` |
| `studica_robot_monitor` | `745a5150dec5e4a40e16cab28c842963c22db2f4` |

The platform build source is local commit
`eb5e11aa6544b37eef58b6f578a15a8878db3aa7`. Its complete Git bundle was
transferred to the VMXPi and verified against SHA-256
`74169ffbf52b8500bd07a1871a4925c2d72d2e4119f00f53f20bec4f9331befb`.
The platform commit was not pushed to GitHub in this operation.

The separate checkout `/home/vmx/studica-release-src-eb5e11a` was clean and at
the expected commit. The live `/home/vmx/studica_ws` was not modified.

## Build preflight

- Host: `vmx`, native AArch64, Ubuntu 22.04; ROS 2 Humble build target.
- SDK root: `/usr/local`, mounted read-only into the offline build container.
- Compile/test worker limit: one.
- Initial available storage: 6.6 GiB; available RAM: 3.3 GiB; swap use: zero.
- Initial firmware power/throttling status: `throttled=0x0`.
- Initial boot ID: `e5761a75-7bfa-4f2d-9c08-c151cfca33e4`.
- No robot-control processes, no installed `studica-robot.target`, and no
  `/opt/studica/current` pointer were present.

Builder prerequisite validation passed. This uses the previously approved
temporary VMXPi development-builder exception; it is not an independently
qualified production build worker.

Inspection of the running compiler container confirmed `network=none`,
`readonly=true`, `privileged=false`, `cpus=1000000000` (one CPU), and
`devices=[]`. Prepared sources, VMX SDK headers/library, and the runtime
package inventory were mounted read-only. Only the isolated workspace and
artifact output were writable bind mounts.

## Result

Both container images were built and all six hardware repository pins were
verified before offline compilation. The LiDAR SDK, camera messages and
description, corrected Titan driver, corrected monitor, ROS LiDAR driver, and
camera driver compiled successfully. The native Titan enable-protocol test
executable was built, but the automated test phase had not run.

During `studica_ros2_control` compilation, the robot kernel reported:

```text
Sep 06 18:28:54 vmx kernel: hwmon hwmon1: Undervoltage detected!
```

The timestamp is from the robot journal. The identified offline build
container was stopped with a ten-second termination timeout. The build
returned 137 after this deliberate stop; it was not a reported compiler error
or test failure. The temporary build workspace was removed by the builder's
existing cleanup handler. No live workspace, map, or previous release was
removed.

Post-stop checks confirmed:

- `/home/vmx/studica-release-artifacts/eb5e11a` was empty;
- no build container or robot-control process was running;
- `studica-robot.target` remained absent/inactive;
- `/opt/studica/current` remained absent;
- the boot ID matched the initial observation (no reboot observed);
- 5.0 GiB storage and 3.2 GiB RAM were available;
- no out-of-memory or OOM-kill message was observed.

The existing `b6b6c86` staged candidate remains guarded and must not be
activated. Neither hardware commissioning nor cold-boot qualification was
performed in this attempt.

## Retained evidence and retry condition

The complete build log remains at
`/home/vmx/studica-release-eb5e11a-build.log` and was copied to the development
workspace at
`robot_test_results/arm64_build_eb5e11a_20260906/build.log`.
Both copies have SHA-256
`fc0b0d91ede88c5923d7a6dfa6607ea3a9a29b221182a17b95d882dccf625fac`.

Local project validation and all ten builder-contract tests passed. There is
no completed native test result and no releasable archive for this attempt.

Keep motion disabled and resolve the VMXPi power-supply instability before
retrying. The kernel warning alone does not identify whether the battery,
regulator, wiring, or another supply component caused the voltage drop.
After stable power is established, repeat the clean native build against
`eb5e11aa6544b37eef58b6f578a15a8878db3aa7`; do not resume or accept outputs
from the interrupted compilation. Verify a new archive before inactive
staging, and retain the separate physical-safety and cold-boot gates.
