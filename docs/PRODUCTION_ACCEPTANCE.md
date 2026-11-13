# Production acceptance record and resource collection

Prepared locally, 2026-11-12. This is a commissioning template, not authorization
to deploy, access a robot, inject failures or command motion. It grants no new
qualification. Record pass/fail/not-run and evidence for each row.

## Evidence header

Record device ID, robot profile and configuration digest; source commits and dirty
status; ARM64 artifact hash and signature verification; installed prefix and
runtime executable hashes; OS/kernel/ROS/RMW versions; VMX SDK hash; Titan firmware;
physical wiring and independent-stop assessment; operator and test authorization;
network/domain allocation; test start/end times and evidence paths.

PC simulation, a source checkout and an installed robot are separate evidence sets.
A successful discovery or stale cached READY_DISARMED message does not prove ready.

## Gates and stop conditions

| Gate | Required evidence | Reject/stop condition |
|---|---|---|
| Build | Clean pinned sources, native ARM64 build, complete dependency inventory and isolated tests | Missing pins, dirty input, unresolved dependencies or skipped required checks |
| Install | Inactive installation, signature and runtime library/SDK matching on clean target image | Any unverified artifact or incompatible dependency |
| Offline boot | No PC/network required; fresh disarmed state, zero targets/feedback, one owner, no startup motion | Unexpected motion, duplicate owner, stale evidence or restart loop |
| Restart recovery | Whole-session restart and fresh local release/enable required | Earlier commands resume or motion authorization survives |
| Update | Armed update rejected; corrupt/incompatible candidate rejected; health failure rolls back | Runtime pointer changes without gate or previous release unavailable |
| Power interruption | Recover before runtime startup at each journal/pointer transition | Unknown pointer/journal state, lost calibration or failure to retain inhibition |
| Broken candidate environment | Recovery works even if selected release cannot import Python modules or load ROS libraries | Recovery depends on the broken candidate; provision independent maintenance runtime |
| Physical stop/command loss | Independently reviewed stop arrangement and verified watchdog behavior, then authorized measured tests | Stop path unresolved; do not start motion tests |
| Classroom isolation | Two robots and simulation, independent process environments; no command/TF/clock crossover | Any cross-session data/control ownership conflict |
| Resource soak | 24-hour stationary sensor workload, bounded logs and measured headroom | Thermal/power faults, disk exhaustion, lost samples, or agreed resource budget exceeded |

The 80 ms provenance, 250 ms supervisor and 500 ms drive limits remain unchanged.
Do not substitute the resource sampler's interval or adapter receive-age limits.
Physical tests require a separate commissioned procedure, operator, and an
independent stopping means; the present VMX-input-only E-stop is unresolved.

For update interruption tests, use disposable non-actuating target environments
first. Cover staging, journal creation, stop, pointer switch, start, health check,
history finalization and lock removal. Preserve the existing dated installation
and calibration; never delete it to free space for a candidate.

Local process-exit fixtures exercise durable journal creation, release-pointer
switching, history writing and journal finalization using temporary directories.
They verify pointer recovery and repeated recovery without executing target code
or controlling services. Additional fixtures cover terminal receipts and exits
before/after lock removal and directory synchronization for successful and rolled
back transactions. Repeated cleanup retains the committed pointer and history.
Unbound locks and mismatched pointers remain inhibited. These are not power-loss
or systemd tests: qualify the filesystem's persistence behavior and the compatible
maintenance runtime on a disposable target before production acceptance.

## Read-only resource sampler

The installed `studica_resource_sampler.py` reads Linux CPU, memory, disk, thermal
and network counters. It starts no ROS node, sensor workload, stress test, service
or SSH connection. Run it only on the host being measured after separately
arranging the intended workload. A PC sample must not be labeled robot telemetry.

Example for a later authorized robot soak, in its selected runtime environment:

```bash
ros2 run studica_vmxpi_ros2 studica_resource_sampler.py \
  --duration-seconds 86400 --interval-seconds 5 \
  --output /var/tmp/studica-resource-soak.jsonl
```

Output must be a new file. Collection is bounded to 24 hours, with intervals of
1–60 seconds. Ctrl+C preserves completed records. Each JSON line contains UTC time,
monotonic elapsed time, host identity, CPU percentage when deltas are valid,
memory/swap bytes, disk capacity, per-interface cumulative RX/TX counters and
available thermal-zone values. Counter resets produce unknown CPU utilization;
unavailable observations are recorded explicitly. No credentials are collected.

Set resource budgets before the campaign using the target hardware specification
and measured control workload. Do not invent a pass threshold after seeing data.
Review sample gaps, thermal zones, throttling/power evidence, CPU/memory peaks,
storage growth and per-interface bandwidth. The sampler does not inspect
undervoltage, control-loop latency, actuator response or live ROS node health;
record those with separately reviewed instruments. A complete file alone is not
acceptance. Sampling errors or missing temperature data must remain visible.

## Release decision

Attach raw evidence and a concise interpretation. A reviewer signs acceptance for
an exact artifact/configuration/device combination, not a branch name. Keep
unqualified capabilities unavailable. Pilot one robot before classroom rollout.
