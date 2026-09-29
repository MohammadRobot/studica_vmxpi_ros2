import asyncio
from pathlib import Path

from aiohttp import WSServerHandshakeError
from aiohttp.test_utils import TestClient, TestServer

from studica_robot_platform import API_VERSION
from studica_robot_platform.companion import RobotApi
from studica_robot_platform.map_registry import MapRegistry
from studica_robot_platform.pairing import CompanionPairingStore
import studica_robot_platform.web_server as web_server
from studica_robot_platform.web_server import (
    ALLOW_INSECURE_HTTP,
    AuthStore,
    create_app,
)


class FakeBridge:
    def __init__(self):
        self.heartbeat = None
        self.commands = []
        self.value = {
            "mode": "IDLE",
            "requested_mode": "IDLE",
            "transition": "READY",
            "safety_state": "READY_DISARMED",
            "armed": False,
            "ready": True,
        }

    def status(self):
        return self.value

    def publish_teleop(self, linear, angular):
        self.commands.append((linear, angular))

    def publish_companion_heartbeat(self, *values):
        self.heartbeat = values


def test_companion_pairing_scope_and_browser_websocket_csrf(
    tmp_path: Path, monkeypatch
):
    async def scenario():
        web_root = tmp_path / "web"
        (web_root / "assets").mkdir(parents=True)
        (web_root / "index.html").write_text("test", encoding="utf-8")
        pairing = CompanionPairingStore(tmp_path / "pairing")
        bridge = FakeBridge()
        monkeypatch.setattr(
            web_server,
            "orchestrator_request",
            lambda payload: {"ok": True, "restart_scheduled": True},
        )
        app = create_app(
            bridge,
            AuthStore("a" * 32, pairing),
            MapRegistry(tmp_path / "maps"),
            pairing,
            web_root,
        )
        app[ALLOW_INSECURE_HTTP] = True
        browser = TestClient(TestServer(app))
        await browser.start_server()
        try:
            response = await browser.post(
                "/api/v1/session", json={"token": "a" * 32}
            )
            assert response.status == 200
            csrf = (await response.json())["csrf"]
            response = await browser.post(
                "/api/v1/companion/pairing-code",
                json={},
                headers={"X-CSRF-Token": csrf},
            )
            assert response.status == 201
            code = (await response.json())["code"]

            response = await browser.post(
                "/api/v1/companion/pair",
                json={"code": code, "companion_id": "test-pc"},
            )
            assert response.status == 200
            token = (await response.json())["token"]
            bearer = {"Authorization": f"Bearer {token}"}
            api = RobotApi(str(browser.make_url("")), token, insecure=True)
            assert (await asyncio.to_thread(api.status))["mode"] == "IDLE"
            await asyncio.to_thread(
                api.heartbeat,
                {
                    "api_version": API_VERSION,
                    "companion_id": "test-pc",
                    "connected": True,
                    "active_mode": 0,
                    "ready": True,
                    "detail": "",
                },
            )
            assert (await browser.get("/api/v1/status", headers=bearer)).status == 200
            assert (
                await browser.post(
                    "/api/v1/companion/heartbeat",
                    json={
                        "api_version": API_VERSION,
                        "companion_id": "test-pc",
                        "connected": True,
                        "active_mode": 0,
                        "ready": True,
                        "detail": "",
                    },
                    headers=bearer,
                )
            ).status == 200
            assert bridge.heartbeat is not None
            assert (
                await browser.put(
                    "/api/v1/mode",
                    json={"mode": "IDLE"},
                    headers=bearer,
                )
            ).status == 403
            assert (
                await browser.post(
                    "/api/v1/companion/pair",
                    json={"code": code, "companion_id": "second-pc"},
                )
            ).status == 401

            try:
                await browser.ws_connect("/api/v1/telemetry")
                raise AssertionError("browser WebSocket accepted without CSRF proof")
            except WSServerHandshakeError as error:
                assert error.status == 403
            websocket = await browser.ws_connect(
                "/api/v1/telemetry", protocols=(f"studica-v1.{csrf}",)
            )
            await websocket.close()

            bridge.value.update(
                {"mode": "MANUAL_WEB", "armed": True, "safety_state": "ARMED"}
            )
            drive = await browser.ws_connect(
                "/api/v1/teleop", protocols=(f"studica-v1.{csrf}",)
            )
            await drive.send_json(
                {
                    "sequence": 1,
                    "linear_x": 0.2,
                    "angular_z": 0.0,
                    "deadman": True,
                }
            )
            await asyncio.sleep(0.05)
            assert (0.2, 0.0) in bridge.commands
            try:
                await browser.ws_connect(
                    "/api/v1/teleop", protocols=(f"studica-v1.{csrf}",)
                )
                raise AssertionError("second browser operator was accepted")
            except WSServerHandshakeError as error:
                assert error.status == 409
            await asyncio.sleep(0.3)
            assert bridge.commands[-1] == (0.0, 0.0)
            await drive.close()
        finally:
            await browser.close()

    asyncio.run(scenario())


def test_observation_only_rejects_all_control_even_with_armed_status(tmp_path, monkeypatch):
    async def scenario():
        root = tmp_path / "web"
        (root / "assets").mkdir(parents=True)
        (root / "index.html").write_text("observer")
        pairing = CompanionPairingStore(tmp_path / "pairing")
        bridge = FakeBridge()
        bridge.value.update(mode="MANUAL_WEB", armed=True)

        def forbidden_orchestrator(*args, **kwargs):
            raise AssertionError("read-only API reached the privileged orchestrator")

        monkeypatch.setattr(web_server, "orchestrator_request", forbidden_orchestrator)
        app = create_app(bridge, AuthStore("b" * 32, pairing),
                         MapRegistry(tmp_path / "maps"), pairing, root,
                         observation_only=True)
        app[ALLOW_INSECURE_HTTP] = True
        client = TestClient(TestServer(app))
        await client.start_server()
        try:
            assert (await client.get("/api/v1/status")).status == 401
            assert (await client.post("/api/v1/session", json={"token": "wrong"})).status == 401
            assert (await client.put("/api/v1/mode", json={})).status == 401
            login = await client.post("/api/v1/session", json={"token": "b" * 32})
            assert login.status == 200
            csrf = (await login.json())["csrf"]
            headers = {"X-CSRF-Token": csrf}
            for method, path in (
                ("PUT", "/api/v1/mode"), ("PUT", "/api/v1/sensors/camera"),
                ("PUT", "/api/v1/developer-mode"), ("POST", "/api/v1/maps"),
                ("POST", "/api/v1/maps/save"), ("POST", "/api/v1/navigation/goal"),
                ("GET", "/api/v1/bluetooth/devices"), ("POST", "/api/v1/bluetooth/pair"),
                ("GET", "/api/v1/network/wifi"), ("POST", "/api/v1/network/wifi"),
                ("GET", "/api/v1/updates"), ("POST", "/api/v1/updates/activate"),
                ("POST", "/api/v1/support-bundle"), ("POST", "/api/v1/companion/pair"),
                ("POST", "/api/v1/companion/pairing-code"), ("POST", "/api/v1/companion/heartbeat"),
            ):
                response = await client.request(method, path, headers=headers, json={})
                assert response.status == 403, (method, path, await response.text())
            try:
                await client.ws_connect("/api/v1/teleop", protocols=(f"studica-v1.{csrf}",))
                raise AssertionError("read-only API accepted teleop")
            except WSServerHandshakeError as error:
                assert error.status == 403
            assert (await client.get("/api/v1/status")).status == 200
            assert (await client.get("/api/v1/maps")).status == 200
            health = await (await client.get("/api/v1/health")).json()
            assert health["read_only"] is True and health["ros_ready"] is False
            spec = await (await client.get("/api/v1/openapi.json")).json()
            assert "/api/v1/mode" not in spec["paths"]
            assert "post" not in spec["paths"]["/api/v1/maps"]
            assert bridge.commands == [] and bridge.heartbeat is None
        finally:
            await client.close()

    asyncio.run(scenario())
