# ARM64 build after battery replacement: 2026-09-06

The operator reported replacing the battery after the
[interrupted build](ARM64_BUILD_ATTEMPT_2026-09-06.md). This record covers a
fresh development build; it is not permission for motion or boot autostart.

## Inputs and power baseline

- Exact platform source: `eb5e11aa6544b37eef58b6f578a15a8878db3aa7`.
- Driver pin: `0b45ad72aeaceeff2ed47e4429eac140d54a9d5c`.
- Monitor pin: `745a5150dec5e4a40e16cab28c842963c22db2f4`.
- Clean source checkout: `/home/vmx/studica-release-src-eb5e11a`.
- Retry output: `/home/vmx/studica-release-artifacts/eb5e11a-retry1`.
- Retry log: `/home/vmx/studica-release-eb5e11a-retry1-build.log`.
- Start: `2026-09-06T18:39:59+04:00`, as reported by the robot clock.
- Boot ID: `e5761a75-7bfa-4f2d-9c08-c151cfca33e4`, unchanged from the prior run.
- Initial available storage: 5.0 GiB; available RAM: 3.2 GiB.
- No robot-control process or build container was running before the retry.
- `studica-robot.target` and `/opt/studica/current` remained absent.

The firmware returned `throttled=0x50000`: only the historical undervoltage
and throttling bits were set; the current-status bits were clear. These bit
meanings are documented in the official
[Raspberry Pi command reference](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled).
The journal recorded voltage normalisation at 18:28:58 following the earlier
18:28:54 warning. No reboot or clearing of historical flags was performed.
These readings establish an initial observation, not proof that replacement
resolved power stability under sustained load.

Monitoring distinguishes new events using this pre-build kernel-journal
cursor, also preserved at the start of the retry log:

```text
s=53f882c1f0e545db9e37bec662b1e742;i=b1a;b=e5761a757bfa4f2d9c08c151cfca33e4;m=15013285b;t=65ad152740338;x=8673ed3cab417101
```

The builder reuses cached dependency images, not the interrupted compilation
outputs. Source preparation and compilation use new temporary directories,
with one compile worker. Existing logs, inactive releases, maps, and the live
workspace are preserved.

## Result

The fresh native build completed successfully. All eight ROS packages compiled
in 32 minutes 46 seconds, following the separate LiDAR SDK build. The four
selected package test suites completed in 3 minutes 55 seconds. Colcon reported
361 test entries, zero errors, zero failures, and 108 skipped entries.

All six Titan enable-protocol cases ran and passed on ARM64. The container-only
mock runtime, hardware-supervisor, joystick-supervisor, and mode-manager
integration checks also passed. Gazebo, simulated mapping, and simulated
navigation runtime checks explicitly skipped because the hardware-only image
does not include Gazebo. These results do not establish physical wheel motion,
E-stop torque removal, SLAM operation, or navigation acceptance.

The compiler container was verified to have no network, no hardware devices,
no privileged mode, a read-only root filesystem, and a one-CPU limit. Prepared
sources, the SDK, and the package inventory were read-only mounts. No robot
service was started outside the test container.

Monitoring after the recorded journal cursor found no new undervoltage,
brownout, or out-of-memory warning during the retry and post-install checks.
The boot ID stayed unchanged. Firmware flags were still `0x50000` after
staging, with current-status bits clear. This is evidence for this build
interval, not long-term power-supply qualification.

## Verified inactive release

- Version: `0.1.0-dev.eb5e11aa6544`.
- Archive: `studica-robot-0.1.0-dev.eb5e11aa6544-ubuntu22.04-humble-arm64.tar.gz`.
- Archive SHA-256:
  `f94cfbb2579d0106b71b7956a0cf86e911d5ae6d4579ecdbadefc91736d5cc44`.
- Builder image ID:
  `sha256:034a2670bf8415ef3eca34a02b3c7b5b52725d5a92c15396bcbe26282bcdfec6`.
- Staged path: `/opt/studica/releases/0.1.0-dev.eb5e11aa6544`.
- Installation timestamp: `2026-09-06T15:25:13.412756+00:00`.
- Installer SHA-256:
  `777cb4c36c9d16a0fc056a8dd9e0a4a05d0d7c677a2300f4630c493a45335a9b`.

Both the VMXPi and workstation passed the independent archive verifier,
including external/internal checksums, exact source provenance, platform,
development channel, and activation denial. Direct inspection of the archive's
own `metadata/hardware.repos` confirmed both corrected dependency pins.

The inactive installer completed successfully. A subsequent non-root status
query could not read the protected update-state directory; rerunning that
read-only query with `sudo` succeeded without changing permissions. The
separate post-install `sha256sum -c --quiet SHA256SUMS` passed.

Post-install checks confirmed:

- `metadata/DO_NOT_ACTIVATE` exists and `activation_authorized` is false;
- `/opt/studica/current` and the previous-release rollback pointer are absent;
- no partial staging entries were left;
- the two earlier inactive releases remain in place;
- the web module and corrected monitor message interfaces import on the host;
- VMX control, hardware interface, Titan driver, LiDAR node, and camera node
  shared-library checks resolve without missing dependencies;
- `studica-robot.target` remains absent/inactive and no build container or
  robot-control process is running;
- approximately 4.9 GiB storage and 3.2 GiB RAM remain available.

The artifact is **development-only**. No cryptographic publisher signature was
verified. Hardware commissioning, independent torque-removal verification,
cold-boot/failure-injection qualification, and production autostart remain
separate unfinished gates. The old `b6b6c86` candidate must not be used for
commissioning.

## Retained evidence

The workstation archive and checksum are in
`release_artifacts/eb5e11aa6544b37eef58b6f578a15a8878db3aa7/`. The complete
build/test log and native Titan protocol XML are in
`robot_test_results/arm64_build_eb5e11a_retry1_20260906/`:

| Evidence | SHA-256 |
|---|---|
| `build.log` | `c2a85ab1b9ac5cf19c512ef67af45c506a0b39cede7cbdb1f8808d736897b0f1` |
| `test_titan_enable_protocol.gtest.xml` | `add7ff5d99287059150452ff48c7e6350463f168e3cfa955773245cb43327110` |

The full XML collection was not captured before the builder's normal temporary
workspace cleanup; its complete test output and aggregate result remain in
the retained log. The protocol XML was copied before cleanup. The prior failed
build log was preserved separately and is not replaced by this successful run.
