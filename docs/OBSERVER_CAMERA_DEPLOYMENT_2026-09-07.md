# Camera and HTTPS observation deployment — 2026-09-07

## Outcome at 21:28 Gulf Standard Time

The non-motion training services are installed on VMXPi `192.168.1.173`.
This is **not** activation of the motion-capable production platform.

| Service | Current state | Boot state |
|---|---|---|
| `studica-training-lidar.service` | Running, zero automatic restarts | Enabled |
| `studica-training-web.service` | Running, zero automatic restarts after fixes | Enabled |
| `studica-training-mdns.service` | Running; advertises `robot.local` | Enabled |
| `studica-training-camera.service` | Stopped, systemd result success | On demand only; no install target |
| `studica-robot.target` and `studica-hardware.service` | Inactive | Disabled |

The active-release pointer `/opt/studica/current` is still absent. No motor
command, hardware runtime, production activation or reboot was performed.
The staged ARM64 release and its activation guards were not modified.

LiDAR cgroup memory was 36,937,728 bytes and web cgroup memory 37,142,528 bytes.
About 3.2 GiB RAM remained available with the camera stopped. This is not a
camera-plus-25-Hz-hardware-loop qualification: that hardware loop was not running.

## Camera evidence

Tested the existing Orbbec Gemini E driver with depth enabled at 320×240,
requested 5 fps; color, IR and point cloud disabled. The camera service is
non-root, device-restricted to USB/video, capped at 120% of one CPU and 700 MiB.
No hardware-control service is a dependency.

The 10-second probe observed:

- 56 depth images, 5.721 Hz measured;
- `16UC1`, 320×240, 640-byte stride, 153,600 bytes/frame;
- increasing timestamps and `camera_depth_optical_frame`;
- 57 CameraInfo messages with matching dimensions/frame and positive focal lengths;
- 50,327 valid pixels in the last image, raw values 150–272;
- simultaneous LiDAR at 11.692 Hz;
- zero motion publishers on isolated ROS domain 78.

These are data-format and streaming checks, not distance-accuracy calibration.
The SDK logged an unsuccessful property-2035 setting during initialization;
streaming continued, but this is not a warning-free driver qualification.
Wrapper version was 1.5.15 and SDK version 1.10.35. Camera memory while running
was approximately 107 MiB, with no automatic restart.

The camera was stopped at 21:24:53. Systemd reported successful deactivation;
the ROS launch log reported its component container exiting on SIGINT (-2).
This was the requested shutdown, not a spontaneous streaming crash. Camera
restart, USB unplug/replug recovery and a cold boot remain untested.

## Observation-only HTTPS

Access the endpoint at `https://192.168.1.173`. On the same mDNS-capable LAN,
`https://robot.local` is advertised; VMXPi resolved that name to its own IP.
The current PC did **not** resolve `robot.local` across its separate subnet,
but HTTPS by IP worked. No PC hostname, DNS, Wi-Fi or firewall settings were
changed. The training mDNS unit and leaf certificate currently use the fixed
address `192.168.1.173`; DHCP-address changes need configuration review.

The service reuses the existing UI/API with `--observation-only`. Its backend
uses standard sensor subscriptions, not the production control interfaces.
The runtime does not source the hardware workspace, runs as `studica`, has
private devices, and has only the low-port-binding capability. Persistent
maps and pairing state are read-only within the service; the unused companion
pairing store is isolated under `/run/studica-training-web/pairing`.

Status explicitly reports `OBSERVATION_ONLY`, `NOT_MONITORED`, `ready=false`
and `armed=null`. It does not invent E-stop, Start, battery or motor telemetry.
Missing sensor messages mean off **or** unavailable, not proof of a disabled
device. The UI includes an observation-only notice and disables its controls.
It currently shows sensor metadata/rates, not an image/video viewer.

Verified with real HTTPS before and after a web-service restart:

- certificate chain and IP SAN validate using the device CA, without insecure TLS flags;
- unauthenticated status returns HTTP 401;
- authenticated status and map listing succeed;
- authenticated mode changes, sensor changes, teleop, navigation and update
  activation return HTTP 403;
- health reports `read_only=true`, `ros_ready=false`;
- live LiDAR and camera data appear while streaming; camera reports no recent
  data after shutdown.

The browser could not open the page because it does not trust the local device
CA (`ERR_CERT_AUTHORITY_INVALID`). No browser certificate warning was bypassed
and no trust store was changed. Consequently, interactive sign-in and visual
browser acceptance are **not** complete. The operator must review the local
certificate before browser use.

Public CA certificate saved on this PC: `/tmp/studica-device-ca-20260907.crt`.
SHA-256 certificate fingerprint:

```text
40:A1:BB:DF:92:A7:D7:5B:40:EA:05:3C:BE:F0:55:9A:E5:7A:88:55:94:BA:CE:08:53:FE:A1:B0:8C:2C:A8:40
```

Copying the existing login token from the robot to a private PC file was
blocked by permission review pending explicit operator approval. No token
was exported or printed in the transcript; the empty local destination was
removed. The canonical token remains `/var/lib/studica/secrets/api-token`.
A mode-0600 preparation copy remains in the operator's robot setup directory
as `observer-access-token`, pending the user's decision. Do not export it
through another method without approval.

## Existing office map

Imported the workstation's `project_maps/office_nav.yaml` and `office_nav.png`
through `MapRegistry` validation, without overwriting an existing map. The
registry entry is `office_nav`, at `/var/lib/studica/maps/office_nav/map.yaml`
with `map.png`, resolution 0.05 m/pixel and origin [-8.0, -10.5, 0.0].
Original workstation map files were not modified.

Source SHA-256 values:

```text
office_nav.yaml: b525614a101f3ea724978bc6205d4cd77bb9241d78a3e0a7603d3364fddf2e08
office_nav.png:  0323bef60d99952310d2b566ff3498a0180a5d490b394616e94a6798f9df0dab
```

Import does not start localization or navigation or establish that the physical
environment still matches the map.

## Fixes and tests

Added lazy loading of production message/service types so observation can run
using standard Humble sensor messages only. Normal control routes retain their
existing behavior outside observation mode. Added regression coverage for
authentication, the read-only route allowlist, rejection even with a fabricated
armed status, truthful freshness/unknown state, and no observer command-client
or publisher construction.

On hardware, fixed two startup issues before enabling the web service:

1. Cyclone DDS needs AF_NETLINK for interface discovery; this socket family was
   added without adding network-administration capabilities or device access.
2. Pairing-store initialization chmod was blocked by the read-only filesystem;
   the observation service now uses its private runtime directory instead of
   making persistent pairing state writable.

The relevant local suite passed **44 tests**. JavaScript syntax, Python compile
checks, selected fatal-error lint checks and whitespace checks passed.
Remote deployed checksums are recorded on the workstation under
`robot_test_results/physical_lidar_20260907/observer_deployed.sha256`.

## Remaining acceptance

- Operator certificate review and browser login/visual verification.
- Login-token export only if explicitly approved.
- Coordinated cold boot of the non-motion services; no unattended reboot was performed.
- Camera controls through the UI are still disabled; current sensor control is
  instructor-managed systemd operation, not the completed no-SSH appliance UX.
- Direct joystick integration, companion SLAM/Nav2, motion qualification,
  signed updates and the full production target remain outside this deployment.
