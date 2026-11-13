# Studica → RoboHiTec configuration handoff

Prepared 2026-11-12. Proposal version `studica.robot-configuration/1`.
All changes are local. This is an offline configuration exchange, not a deployed
ROS API and not proof of runtime state. No changes were made to RoboHiTec here.

## Files to consume

`contracts/robot-configuration-v1/` contains device/calibration input examples
and generated `stack_4wd.fixture.json` and `class_2wd.fixture.json` outputs.
The authoritative generator is `studica_robot_platform/configuration.py`.
The ROS executable is `studica_configuration.py`. It invokes the existing launch
profile validator before and after applying calibration.

Read `robot-configuration.json` as a proposal for a stable adapter input:

| Field | Meaning / consumer rule |
|---|---|
| `schema` | Require exactly the supported version; unknown versions are incompatible. |
| `evidence_scope` | `offline_configuration`: never promote to observed runtime or readiness. |
| `device_id`, `domain_id`, `environment` | Explicit provisioned selection, not authenticated identity. |
| `profile_id` | Selected model/configuration name. |
| `configuration_sha256` | SHA256 of canonical composed device, calibration, profile and controller JSON. |
| `profile_sha256`, `calibration_sha256` | Individual input identities; not hashes of meshes/binaries. |
| `software` | Expected Humble; actual runtime revision/artifact hash remain null until independently established. |
| `drive.layout`, `controller_type` | Configured kinematic family and ROS controller. |
| `drive.joints` | Configured controller joints. |
| `drive.motors` | Active Titan channel/slot records; unused channels omitted. Do not assume four records or positional correspondence with joints. |
| `sensors` | Configured/disabled selection, not discovered/healthy sensor status. |
| `interfaces` | Stable topic/type contract, not a claim that publishers exist. |
| `clock` | Expected simulation or system clock policy. |
| `qualification`, `motion_authorized` | Always unqualified / false in this exporter. |

Canonical JSON uses sorted keys, no whitespace separators and rejects NaN/Infinity.
The combined digest includes effective controller configuration and calibration;
it intentionally changes with identity/domain/sensor selection. It does not prove
which configuration the controller has loaded, nor include launch-time rate/TF
parameter injection. Keep that distinction in the UI and future runtime handshake.

## Requested RoboHiTec work

1. Review this proposal against your integration contract before freezing it.
2. Add a Studica adapter reader for these fixtures. Derive expected motor count
   from `drive.motors`; compare live channel IDs against that set, not just length.
3. Show configuration identity separately from installed/runtime provenance.
4. Treat a schema mismatch, absent contract or telemetry mismatch as unavailable
   or incompatible. Never infer qualification from this file or graph discovery.
5. Keep the core generic and qualification/control policy unchanged.

Do not create a second robot description or copy Studica source into RoboHiTec.
Model/mesh resolution continues through the installed Studica package.

## Next integration gate

Both projects must agree on this document before a stable wire interface is added.
A live announcement needs actual effective launch settings, installed release
identity, freshness and unique publisher ownership, without allowing a static file
to authorize motion. This implementation deliberately does not start a publisher
or add an automatic hardware launch. A physical control capability remains blocked
by the existing qualification and independent-stop/watchdog gates.
