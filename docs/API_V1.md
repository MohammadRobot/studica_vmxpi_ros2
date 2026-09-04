# Robot API v1

The browser UI and automation API are served at `https://robot.local`. `/api/v1/health`, `/api/v1/session`, `/api/v1/companion/pair`, and `/api/v1/openapi.json` are public; all other endpoints require an authorized bearer token or authenticated browser session. Browser mutations also require the session CSRF token.

The administrative device token is generated once at first boot and stored as a private file. Control clients must not put it in URLs or logs. A safely disarmed operator can issue a ten-minute, one-use companion pairing code. Redeeming it returns a separate credential whose plaintext exists only on that companion; the robot stores its SHA-256 digest. Browser teleoperation is a WebSocket lease: every frame has an increasing sequence number, `deadman=true`, and is accepted for no more than 250 ms. Blur, page hiding, socket loss, stale frames and mode changes publish zero.

## REST and WebSocket endpoints

| Interface | Purpose |
|---|---|
| `POST /api/v1/companion/pair` | Public redemption of a one-use pairing code |
| `POST /api/v1/companion/pairing-code` | Authenticated code issuance; requires `IDLE/READY_DISARMED` |
| `POST /api/v1/companion/heartbeat` | Paired companion presence/readiness over HTTPS |
| `GET /api/v1/status` | Safety, mode, sensors, companion, compute, motors and diagnostics |
| `PUT /api/v1/mode` | Select `IDLE`, `MANUAL_JOYSTICK`, `MANUAL_WEB`, `SLAM`, `NAVIGATION`, or `DEVELOPER` |
| `PUT /api/v1/sensors/{lidar,camera}` | Start or stop an optional sensor while disarmed |
| `PUT /api/v1/developer-mode` | Open or close restricted DDS access for the configured peer |
| `GET/POST /api/v1/maps` | List or import validated YAML/image ZIP maps |
| `GET /api/v1/maps/{map_id}/bundle` | Download a portable map pair |
| `POST /api/v1/maps/save` | Ask the companion to save and upload the active SLAM map |
| `POST /api/v1/navigation/goal` | Send an `(x,y,yaw)` goal in the map frame |
| `GET /api/v1/bluetooth/devices` | List controllers; add `?scan=true` to scan |
| `POST /api/v1/bluetooth/pair` | Pair, trust and connect one validated Bluetooth MAC |
| `GET/POST /api/v1/network/wifi` | Scan or provision infrastructure Wi-Fi |
| `GET /api/v1/updates` | List signed staged releases |
| `POST /api/v1/updates/activate` | Approve activation; independently requires `IDLE/READY_DISARMED` |
| `POST /api/v1/support-bundle` | Download a bounded redacted support archive |
| `WS /api/v1/telemetry` | Status stream |
| `WS /api/v1/teleop` | Hold-to-drive command lease |

Companion bearer credentials are restricted to `GET /api/v1/status`, `GET /api/v1/maps/{map_id}/bundle`, `POST /api/v1/maps`, and `POST /api/v1/companion/heartbeat`. The redeeming address becomes the allowlisted DDS peer; re-pair after that address changes. All other API calls return `403`. Browser WebSockets additionally prove the session CSRF secret in the `studica-v1.<csrf>` subprotocol; non-browser bearer clients do not use that subprotocol.

The live machine-readable contract is `GET /api/v1/openapi.json`.

## ROS 2 interface

Public ROS API topics:

- `/robot/platform/status` — `studica_vmxpi_ros2/msg/PlatformStatus`, reliable and transient-local.
- `/robot/platform/events` — JSON event records in `std_msgs/msg/String`.
- `/robot/companion/heartbeat` — companion availability and autonomy readiness.
- `/robot/navigation/goal` — map-frame `geometry_msgs/msg/PoseStamped`.

Public services:

- `/robot/platform/set_mode` — `SetMode`.
- `/robot/platform/set_sensor` — `SetSensor`.
- `/robot/platform/set_developer_mode` — `SetDeveloperMode`.
- `/robot/companion/save_map` — `SaveMap`.
- `/robot/disarm` — existing safety-supervisor `std_srvs/srv/Trigger`.

The internal command topics are `/robot/control/joystick`, `/robot/control/web`, `/cmd_vel` (Nav2), and `/robot/control/developer`. Applications must not publish `/robot/platform/cmd_vel` or the drive controller topic.
