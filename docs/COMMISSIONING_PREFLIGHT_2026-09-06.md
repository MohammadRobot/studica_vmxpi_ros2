# Commissioning preflight: 2026-09-06

The staged `0.1.0-dev.b6b6c86a73a8` release must not be used for hardware
commissioning. Its source dependencies predate corrections used during the
successful supervised wheel tests. Build success did not establish equivalence
with that tested working tree.

## Hardware status

The operator replied "yes" when asked whether the E-stop now independently
removes motor power while the VMXPi stays powered. This is recorded as an
operator report of changed wiring, not as a witnessed torque-removal test or
completed cold-boot qualification. The physical implementation and its observed
behavior still need to be recorded before commissioning motion.

Read-only SSH checks found the robot reachable at `192.168.1.173`, with no
robot control processes, no installed `studica-robot.target`, and no
`/opt/studica/current` pointer. The staged release retains its
`metadata/DO_NOT_ACTIVATE` marker. The host had 6.6 GiB free.

## Release dependency mismatch

The archive digest was rechecked as
`2f27def1e39c741a3d96790c0090031061ee10434f665ee9972c0ab716be087c`.
Its own `metadata/hardware.repos` confirmed these older dependency revisions:

| Dependency | Staged revision | Corrected revision |
|---|---|---|
| `studica_drivers` | `3b0082eb2148e64ab3e3195cdf37793e3f6634c4` | `0b45ad72aeaceeff2ed47e4429eac140d54a9d5c` |
| `studica_robot_monitor` | `f51a2fc0f3c1d23264bdfa7651d7f118d87eaaf2` | `745a5150dec5e4a40e16cab28c842963c22db2f4` |

The staged Titan implementation schedules periodic enable and disable messages
without cancelling the opposite CAN ID. The corrected implementation cancels
that opposing schedule before changing state. The VMXPi's installed
`VMXCAN.h` explicitly documents a negative period as cancellation of periodic
transmission for the specified CAN ID. The correction was already present in
the local working tree; this preflight committed it along with additional
tests for retained schedules and failed writes.

The corrected monitor includes the existing untrusted-temperature reporting
change and encoder-position versus integrated-RPM validation used during
supervised acceptance. These changes have now been committed, preserving their
existing implementation.

Hardware, simulation, and ROS CI manifests now pin the corrected revisions.
The ARM64 build also requires the Titan enable-protocol regression executable
to exist before running the test suite. An older driver revision without this
test must fail release construction even if the remaining packages compile.

## Verification and next action

Both dependencies built in an isolated local directory. Local tests reported
134 tests, zero errors, zero failures, and 66 skipped entries. These include
six Titan enable-protocol cases and the monitor's factor-of-two RPM rejection
test. Local driver builds use the no-SDK stub; the complete corrected driver
still needs the native ARM64 build against the VMX SDK.

An initial monitor test invocation failed because inherited workstation
Cyclone DDS settings selected loopback twice. Re-running with explicit
localhost-only Fast DDS settings passed. This was a test-environment issue;
the robot DDS settings were not changed.

The staged VMX control executable and hardware library resolved their shared
libraries on the host. Python import checks found `cryptography` and `yaml`,
but the host lacked `aiohttp`. Installed Ubuntu's `python3-aiohttp`
`3.8.1-4ubuntu0.2` and its seven dependencies, with no upgrades or removals.
The staged `studica_robot_platform.web_server` then imported successfully
against the host packages. This import check started no web listener or robot
service; the target remained absent and the current-release pointer absent.

Publishing the new driver commit to GitHub `main` was rejected by automatic
approval review because the previous permission covered only the earlier
dependency commit. Neither new dependency revision has been published during
this preflight. Operator approval of the two corrected dependency commits is
required before the normal immutable builder can fetch them from GitHub.

After publication, rebuild and verify a new ARM64 development artifact, stage
it inactive, and perform supervised hardware commissioning against its new
digest. The previous staged artifact and its activation guard remain intact.
The production acceptance requirements are in
[Safety acceptance](SAFETY_ACCEPTANCE.md).

## Follow-up

The operator subsequently approved both dependency publications. The
[corrected ARM64 build attempt](ARM64_BUILD_ATTEMPT_2026-09-06.md) records
successful publication and native compilation progress, followed by a
required stop after a VMXPi undervoltage warning. No corrected artifact was
produced or staged; stable power is required before retrying.

After the operator replaced the battery, the
[fresh native retry](ARM64_BATTERY_RETRY_2026-09-06.md) completed and the
corrected development release was verified and staged inactive. Physical
commissioning and production autostart are still not authorized by that result.
