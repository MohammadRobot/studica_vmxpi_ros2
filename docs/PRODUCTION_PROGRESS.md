# Production implementation progress

2026-11-12 — local source only, no deployment, commits or robot access.

## Configuration composition delivered

Model: existing `bringup/config/profiles/<name>` and shared Xacro/meshes.
Device selection: separate device JSON (identity, domain, environment, sensors).
Calibration: separate device/profile-bound correction JSON. Initially only
explicit differential wheel-radius/separation multipliers are accepted; no PID,
CAN mapping or safety setting can be overridden through calibration JSON.
The conservative [0.5, 2] multiplier bounds are input review bounds, not physical
acceptance. Existing profile multipliers remain unchanged when corrections are empty.

Fresh Humble PC terminal, after building:

```bash
source /opt/ros/humble/setup.bash
source ~/studica_ws/install/local_setup.bash
cd ~/studica_ws/src/studica_vmxpi_ros2
ros2 run studica_vmxpi_ros2 studica_configuration.py \
  --profile-root "$PWD/bringup/config/profiles" \
  --device "$PWD/contracts/robot-configuration-v1/device.example.json" \
  --calibration "$PWD/contracts/robot-configuration-v1/calibration.example.json" \
  --output /tmp/studica-config-review
```

The output directory must not exist. Source profiles are never edited. The export
contains effective `profiles/<name>/*.yaml`, input JSON and metadata JSON. It does
not switch an existing launch or update `/etc/studica`. A failed copy can leave an
incomplete output directory; that directory is not an activated configuration.
Use a new directory for another review. Do not delete existing calibration to retry.
Future deployment should install reviewed effective files under the persistent
profile root already supported by `STUDICA_PROFILE_ROOT`.

See [RoboHiTec handoff](ROBOHITEC_HANDOFF.md) for the proposed contract and limits.

## Support matrix for this candidate

| Profile | Configuration export | Physical production qualification |
|---|---|---|
| stack_4wd | Tested | Pending; prior hardware trials are historical evidence |
| class_4wd | Tested | Pending; classroom simulation baseline |
| class_2wd | Tested | Not qualified |
| class_mecanum | Tested | Not qualified; placeholder safety wiring |
| class_omni | Tested | Not qualified; placeholder safety wiring |

These results do not extend the minimal hardware launch beyond stack_4wd.

## Remaining gates

- Contract agreement with RoboHiTec, then an effective-runtime configuration and
  installed-version announcement with independent freshness/ownership checks.
- Native ARM64 build and runtime library closure, including vendor libraries.
- Reviewed commits and real dependency pin updates. Current driver/platform dirty
  changes cannot be represented by invented SHAs. Existing baseline pins remain.
- Clean-image service installation and offline cold-boot acceptance.
- Signed artifact promotion, inactive installation and interruption/rollback tests
  on a disposable target before a stationary pilot activation.
- Independent stop engineering and Titan watchdog qualification; unchanged
  80/250/500 ms gates. No physical test is authorized by this document.
- Two physical robots plus simulation and a 24-hour resource workload.

The earlier minimal boot, recovery, native packaging and safety improvements remain
local candidates; see [previous audit](ROS2_AUDIT_2026-11-10.md). No current robot
parity or production readiness is claimed.

## Validation of this increment

- Scoped Humble PC build: all three packages passed; hardware plugin remains
  intentionally excluded on x86 without the native VMX SDK library.
- Platform Python gate: 82 tests passed, including 17 new composition/fixture tests.
- Project source/profile/documentation checker and Python lint passed.
- Installed ROS CLI smoke test exported configuration and rejected a second export
  to the same directory without modifying its metadata.
- Tests ran in a loopback-only network namespace. No hardware launch or ROS motion
  publisher was started. No native build, physical test, or deployment was performed.

Baseline HEADs remain platform `472117864f2db97eb59a929889238ed617611575`,
drivers `0b45ad72aeaceeff2ed47e4429eac140d54a9d5c`, monitor
`e5e8c3ffa1a380e6d9d42e58b74c6e525f1e8d3c`; dirty candidate changes are preserved.

## Release-validation and acceptance tooling increment

- Native builds require fresh outputs and recheck source identity after compilation.
- Native bundles now contain a verified static ELF dependency inventory; the
  verifier binds every installed ELF to its digest and rejects incomplete inventories.
- Missing/ambiguous dependencies, wrong architectures and library symlink escapes
  fail validation. No binary is executed by this inspection.
- Activation recovery rejects dangling journals and malformed schema documents
  while retaining evidence and leaving the current pointer unchanged.
- A bounded read-only host resource sampler and a detailed acceptance record are
  prepared in [production acceptance](PRODUCTION_ACCEPTANCE.md).

Native library availability is now checked by tooling, but actual ARM64 loading,
ABI/plugin behavior, cold boot, deployed recovery and physical acceptance remain
unexecuted gates. The production plan is not complete. In particular, the recovery
service previously depended on the selected release's Python/ROS environment;
the candidate unit now uses a separately provisioned maintenance prefix.
Provision and qualify that runtime before enabling the service.

### Local validation on 2026-11-12

- Scoped Humble PC build passed for all three packages; this is not an ARM64 build.
- Six selected CTest gates passed: platform Python (97 tests), release construction,
  release verification, ARM64 builder checks, source checks and Python lint.
- Release verifier fixtures (9 tests) and bundle fixtures (12 tests) also passed
  standalone. Native inventory fixtures do not demonstrate actual ARM64 loading.
- Documentation links and `git diff --check` passed.
- Installed sampler smoke test wrote PC observations and refused to overwrite an
  existing record. No 24-hour workload was run.
- ROS checks used a loopback-only network namespace. No robot access, deployment,
  RoboHiTec project changes, commit or push occurred.

## Recovery interruption increment

- Recovery and finalization validate the complete journal schema and release
  version before changing pointers or status. Invalid version/type/path evidence
  is retained. Malformed staged status is rejected explicitly.
- Disposable child processes exit at four filesystem transaction boundaries.
  Recovery restores the previous release for incomplete transactions, retains
  the finalized selection, and is idempotent after completion.
- The update agent rejects recovery against active/unknown runtime states and
  preserves unexplained or symlink maintenance locks. Service calls are mocked
  in these tests; no host service is controlled.
- Automatic recovery after finalization but before lock removal remains a gap:
  the orphan lock now requires investigation instead of being silently cleared.
  A durable completion protocol and clean-image qualification are still needed.

This increment does not claim power-loss tolerance or target-system acceptance.

Validation: 116 platform Python tests passed (19 added in this increment), along
with the source checker, Python lint, documentation links and `git diff --check`.
The three selected CTest gates ran in a loopback-only network namespace using the
explicit Humble underlay. No C++ or build configuration changed in this increment;
the earlier PC build was not repeated. All edits remain local and uncommitted.


## Completion recovery protocol — 2026-11-13

The previously recorded finalization-to-unlock gap is addressed in the local
candidate. Each new activation has a random transaction ID, persisted in both the
journal and an exclusively created, synced maintenance lock. Terminal journal
receipts survive until lock deletion has been directory-synced. Recovery validates
the receipt, lock binding and expected current pointer before finishing cleanup;
repeated recovery does not rewrite history or resume commands. Cleanup errors after
commit no longer cause a contradictory rollback.

Unexplained, stale, symlink and legacy empty locks still require investigation.
Provision compatible maintenance code before any future deployment. This work is
local software evidence only; target power interruption, service ordering and
physical qualification remain open gates.

Validation on 2026-11-13: 137 platform Python tests passed, including 12 child-process
exit cases across three terminal outcomes, six invalid-evidence cases and three
cleanup/finalization error cases. Existing activation tests now use real journal
and finalization primitives with only hardware/service boundaries mocked. Source
checks, Python lint, documentation links and `git diff --check` passed. The three
scoped CTest gates ran with explicit Humble in a loopback-only network namespace.
No robot access, deployment, RoboHiTec changes, commits or pushes were performed.

## Standard beginner ROS 2 workflow — 2026-11-13

Added [Beginner ROS 2 manual](BEGINNER_MANUAL.md), linked from the README,
documentation index and historical robot manual. It uses standard ROS environment
variables and existing `ros2 launch` entry points for simulation, drive, SLAM and
navigation, plus map saving, introspection, stopping and concurrent sessions.
New classroom example domains 10/11/20 do not replace the recorded robot domain 78.

The profile generator defaults simulation to domain 20, requires an explicit
robot peer domain, and accepts domains 0–100 consistently with provisioning.
Robot and companion templates now match developer participant indices 0–50;
production firewall ranges match that capacity. Existing configuration files and
firewall rules were not regenerated or applied. Migration requires instructor
review of both profiles and rules.

Validation: four scoped CTest gates passed, including 138 platform Python tests,
eight setup-contract tests, source checks and lint. All four public beginner
launches accepted `--show-args` in a loopback-only namespace with explicit Humble;
no simulator, joystick or hardware launch was started. Documentation links and
`git diff --check` passed. A previous test invocation from the package directory
failed because no build existed there; validation was rerun successfully from the
workspace root. No fresh build or physical multi-robot acceptance was performed.
No robot access, deployment, RoboHiTec changes, commits or pushes occurred.

## Two simultaneous Gazebo sessions — 2026-11-13

Completed the local [two-simulation acceptance](TWO_SIMULATIONS_ACCEPTANCE.md).
Actual headless robots in domains 20/21 and distinct Gazebo partitions ran on the
same isolated loopback network. Each moved about 0.49 m when commanded; the other
received zero commands and stayed stationary. Marker traffic and telemetry
publisher identities remained isolated, with fresh sensor/TF/clock/odom samples
in both domains. All recorded launch children were stopped afterward. The fixture,
raw evidence paths, exact reproduction command and limitations are documented.
No deployment, robot access, RoboHiTec changes, commits or pushes occurred.


## Concurrent SLAM and navigation — 2026-11-13

The [dual SLAM/Nav2 acceptance](TWO_SLAM_NAVIGATION_ACCEPTANCE.md) passed with two
actual Gazebo sessions in domains 20/21. Both built distinct maps, saved them and
used the matching maps for concurrent navigation. Both distinct goals succeeded;
map hashes, TF markers, publisher identities and action goal UUIDs remained
isolated. All 72 recorded launch children were stopped. Evidence and reproduction
commands are documented; physical-network and hardware gates remain unchanged.
Everything remains local, with no robot access or RoboHiTec project modifications.

## Pre-push review — 2026-11-13

Reviewed the accumulated platform and driver changes for Git publication without
robot deployment. The driver now has a real reviewed commit,
`c74b1faf7482d6fc28c2dcd45530306b95928f69`; hardware/simulation manifests and ROS CI
pin that revision for the checked duty-write API. Monitor and other dependency
repositories were unchanged. Historical baseline revisions above remain evidence
of earlier work, not the new dependency selection.

Review corrections aligned minimal-runtime and configuration-export domain bounds
with the 0–100 provisioning policy, added rejection coverage for 101, and placed
CI/installer validation inside a loopback-only network namespace. No system
network profiles or firewall rules were applied.

A fresh scoped PC build passed for all three packages. The 35 selected platform
CTest gates (excluding cppcheck and headless Gazebo) passed after correcting the
CI pin and a test formatting error; the platform Python gate reported 140 passes.
All six driver CTest entries returned success, but its cppcheck invocation skipped
analysis due to the installed version policy. The aggregate platform result cache
reports 202 tests, zero errors/failures and 19 skips; this includes cached entries
and is not a claim that the entire suite was rerun. The separately documented dual
Gazebo and dual SLAM/navigation runs remain the simulation acceptance evidence.
Shell/workflow syntax, documentation links and staged-diff whitespace checks pass.

No native ARM64 build, target power-loss test, physical acceptance or deployment
was performed. Release artifacts remain development candidates. The monitor and
RoboHiTec OS were not changed. No credentials or generated build/install/log
artifacts are included in the commits.
