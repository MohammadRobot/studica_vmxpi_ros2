"use strict";

const state = { csrf: "", status: null, maps: [], driveSocket: null, sequence: 0, driveTimer: null };
const $ = (selector) => document.querySelector(selector);
const $$ = (selector) => Array.from(document.querySelectorAll(selector));

function log(message) {
  const activity = $("#activity");
  const timestamp = new Date().toLocaleTimeString();
  activity.textContent = `[${timestamp}] ${message}\n${activity.textContent}`.slice(0, 4000);
}

async function api(path, options = {}) {
  const headers = new Headers(options.headers || {});
  if (options.body && !(options.body instanceof Blob)) headers.set("Content-Type", "application/json");
  if (state.csrf && !["GET", "HEAD"].includes(options.method || "GET")) headers.set("X-CSRF-Token", state.csrf);
  const response = await fetch(path, { ...options, headers, credentials: "same-origin" });
  if (!response.ok) throw new Error((await response.text()) || `${response.status}`);
  const type = response.headers.get("content-type") || "";
  return type.includes("json") ? response.json() : response.text();
}

function renderStatus(value) {
  state.status = value;
  $("#connection").textContent = "Robot online";
  $("#connection").className = "pill online";
  $("#safety-state").textContent = value.safety_state;
  $("#safety-reason").textContent = value.safety_reason;
  $("#mode-state").textContent = value.mode;
  $("#mode-transition").textContent = value.transition;
  $("#companion-state").textContent = value.companion_connected ? "Connected" : "Unavailable";
  $("#control-source").textContent = value.active_control_source || "None";
  $("#platform-error").textContent = value.last_error || "No active error";
  $("#lidar-health").textContent = `${value.sensors.lidar.enabled ? "On" : "Off"} · ${value.sensors.lidar.healthy ? "healthy" : "not ready"}`;
  $("#camera-health").textContent = `${value.sensors.camera.enabled ? "On" : "Off"} · ${value.sensors.camera.healthy ? "healthy" : "not ready"}`;
  const diagnostic = value.diagnostics || {};
  const inputs = diagnostic.safety_inputs || {};
  const inputTrue = (item) => item === true || String(item).toLowerCase() === "true";
  $("#input-state").textContent = inputTrue(inputs.estop_ok) ? "E-stop released" : "E-stop/inputs blocked";
  $("#input-detail").textContent = `Start ${inputTrue(inputs.enable_active) ? "active" : "released"} · ${inputs.gate_state || "unknown"}`;
  const compute = diagnostic.compute || {};
  $("#compute-state").textContent = compute.cpu_load_1m_percent == null ? "Waiting" : `${compute.cpu_load_1m_percent.toFixed(0)}% load`;
  $("#compute-detail").textContent = compute.memory_used_percent == null ? diagnostic.summary || "—" : `${compute.memory_used_percent.toFixed(0)}% memory · ${compute.disk_used_percent == null ? "?" : compute.disk_used_percent.toFixed(0)}% disk`;
  const motors = diagnostic.motors || [];
  const unhealthy = motors.filter((motor) => motor.level > 0).length;
  $("#motor-state").textContent = motors.length ? `${motors.length - unhealthy}/${motors.length} healthy` : "Waiting";
  $("#motor-detail").textContent = diagnostic.summary || "—";
  $("#battery-state").textContent = diagnostic.battery_voltage == null ? "Not reported" : `${diagnostic.battery_voltage.toFixed(2)} V`;
  $$(".companion-required").forEach((button) => { button.disabled = !value.companion_connected; });
  $("#drive-panel").hidden = value.mode !== "MANUAL_WEB";
  $("#navigation-panel").hidden = value.mode !== "NAVIGATION";
  $("#observation-notice").hidden = !value.read_only;
  if (value.read_only) {
    $("#connection").textContent = "Sensor observer online";
    $("#input-state").textContent = "Not monitored";
    $("#input-detail").textContent = "No physical safety input connection";
    $("#motor-state").textContent = "Not monitored";
    for (const name of ["lidar", "camera"]) {
      const sensor = value.sensors[name];
      $(`#${name}-health`).textContent = sensor.healthy
        ? `Receiving · ${sensor.rate_hz} Hz` : "No recent data · off or unavailable";
    }
    $$("#dashboard button, #dashboard input, #dashboard select").forEach((item) => { item.disabled = true; });
  }
}

async function refreshStatus() {
  try { renderStatus(await api("/api/v1/status")); }
  catch (error) { $("#connection").textContent = "Robot unavailable"; $("#connection").className = "pill offline"; log(error.message); }
}

async function refreshMaps() {
  const result = await api("/api/v1/maps");
  state.maps = result.maps;
  const select = $("#map-select");
  select.replaceChildren(...result.maps.map((item) => new Option(item.map_id, item.map_id)));
}

async function setMode(mode, mapId = "") {
  stopDrive();
  const result = await api("/api/v1/mode", { method: "PUT", body: JSON.stringify({ mode, map_id: mapId }) });
  log(result.message);
  await refreshStatus();
}

async function setSensor(sensor) {
  const enabled = !state.status.sensors[sensor].enabled;
  const result = await api(`/api/v1/sensors/${sensor}`, { method: "PUT", body: JSON.stringify({ enabled }) });
  log(result.message);
}

function driveValues(direction) {
  return {
    forward: [0.20, 0], reverse: [-0.16, 0], left: [0, 0.55], right: [0, -0.55], stop: [0, 0],
  }[direction];
}

function openDriveSocket() {
  if (state.driveSocket && state.driveSocket.readyState <= WebSocket.OPEN) return;
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  state.driveSocket = new WebSocket(
    `${protocol}://${location.host}/api/v1/teleop`,
    [`studica-v1.${state.csrf}`],
  );
}

function sendDrive(direction, deadman = true) {
  openDriveSocket();
  const values = driveValues(direction);
  if (!values || state.driveSocket.readyState !== WebSocket.OPEN) return;
  state.driveSocket.send(JSON.stringify({ sequence: ++state.sequence, linear_x: values[0], angular_z: values[1], deadman }));
}

function startDrive(direction) {
  stopDrive();
  sendDrive(direction, true);
  state.driveTimer = setInterval(() => sendDrive(direction, true), 50);
}

function stopDrive() {
  if (state.driveTimer) clearInterval(state.driveTimer);
  state.driveTimer = null;
  if (state.driveSocket && state.driveSocket.readyState === WebSocket.OPEN) {
    const values = driveValues("stop");
    state.driveSocket.send(JSON.stringify({ sequence: ++state.sequence, linear_x: values[0], angular_z: values[1], deadman: false }));
  }
}

$("#login-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  try {
    const result = await api("/api/v1/session", { method: "POST", body: JSON.stringify({ token: $("#token").value }) });
    state.csrf = result.csrf;
    $("#token").value = "";
    $("#login-card").hidden = true;
    $("#dashboard").hidden = false;
    await Promise.all([refreshStatus(), refreshMaps()]);
    setInterval(refreshStatus, 1000);
  } catch (error) { $("#login-error").textContent = error.message; }
});

$$('[data-mode]').forEach((button) => button.addEventListener("click", () => setMode(button.dataset.mode).catch((error) => log(error.message))));
$("#navigation-button").addEventListener("click", () => {
  const mapId = $("#map-select").value;
  if (!mapId) return log("Import or select a map first");
  setMode("NAVIGATION", mapId).catch((error) => log(error.message));
});
$("#lidar-toggle").addEventListener("click", () => setSensor("lidar").catch((error) => log(error.message)));
$("#camera-toggle").addEventListener("click", () => setSensor("camera").catch((error) => log(error.message)));
$("#developer-toggle").addEventListener("click", async () => {
  try { const result = await api("/api/v1/developer-mode", { method: "PUT", body: JSON.stringify({ enabled: !state.status.developer_mode }) }); log(result.message); }
  catch (error) { log(error.message); }
});
$("#refresh").addEventListener("click", () => Promise.all([refreshStatus(), refreshMaps()]));
$("#bluetooth-scan").addEventListener("click", async () => {
  try {
    log("Scanning for Bluetooth controllers…");
    const result = await api("/api/v1/bluetooth/devices?scan=true");
    const select = $("#bluetooth-devices");
    select.replaceChildren(...result.devices.map((item) => new Option(`${item.name} · ${item.address}`, item.address)));
    log(`Found ${result.devices.length} Bluetooth device(s)`);
  } catch (error) { log(error.message); }
});
$("#bluetooth-pair").addEventListener("click", async () => {
  const address = $("#bluetooth-devices").value;
  if (!address) return log("Select a controller first");
  try { const result = await api("/api/v1/bluetooth/pair", { method: "POST", body: JSON.stringify({ address }) }); log(result.message); }
  catch (error) { log(error.message); }
});
$("#wifi-scan").addEventListener("click", async () => {
  try {
    const result = await api("/api/v1/network/wifi");
    $("#wifi-networks").replaceChildren(...result.networks.map((item) => new Option(`${item.ssid} · ${item.signal}% · ${item.security}`, item.ssid)));
    log(`Found ${result.networks.length} Wi-Fi network(s)`);
  } catch (error) { log(error.message); }
});
$("#wifi-connect").addEventListener("click", async () => {
  const ssid = $("#wifi-networks").value;
  if (!ssid) return log("Select a Wi-Fi network first");
  try {
    const result = await api("/api/v1/network/wifi", { method: "POST", body: JSON.stringify({ ssid, password: $("#wifi-password").value }) });
    $("#wifi-password").value = "";
    log(result.message);
  } catch (error) { log(error.message); }
});
$("#companion-pair").addEventListener("click", async () => {
  try {
    const result = await api("/api/v1/companion/pairing-code", { method: "POST", body: "{}" });
    $("#companion-code").textContent = result.code;
    $("#companion-expiry").textContent = `Expires ${new Date(result.expires_at * 1000).toLocaleTimeString()}`;
    log("One-time companion pairing code created");
  } catch (error) { log(error.message); }
});
$("#updates-refresh").addEventListener("click", async () => {
  try {
    const result = await api("/api/v1/updates");
    const pending = result.updates.filter((item) => item.state === "VERIFIED_PENDING_APPROVAL");
    $("#update-select").replaceChildren(...pending.map((item) => new Option(item.version, item.version)));
    $("#update-activate").disabled = pending.length === 0;
    log(`${pending.length} verified update(s) pending approval`);
  } catch (error) { log(error.message); }
});
$("#update-activate").addEventListener("click", async () => {
  const version = $("#update-select").value;
  if (!version) return;
  try { const result = await api("/api/v1/updates/activate", { method: "POST", body: JSON.stringify({ version }) }); log(result.message); }
  catch (error) { log(error.message); }
});
$("#support-bundle").addEventListener("click", async () => {
  try {
    const response = await fetch("/api/v1/support-bundle", { method: "POST", headers: { "X-CSRF-Token": state.csrf }, credentials: "same-origin" });
    if (!response.ok) throw new Error(await response.text());
    const blob = await response.blob();
    const link = document.createElement("a");
    link.href = URL.createObjectURL(blob);
    link.download = "studica-support.tar.gz";
    link.click();
    URL.revokeObjectURL(link.href);
    log("Support bundle downloaded");
  } catch (error) { log(error.message); }
});
$("#map-upload-form").addEventListener("submit", async (event) => {
  event.preventDefault();
  const file = $("#map-bundle").files[0];
  try { const result = await api(`/api/v1/maps?map_id=${encodeURIComponent($("#map-id").value)}`, { method: "POST", body: file, headers: { "Content-Type": "application/zip" } }); log(`Imported ${result.map_id}`); await refreshMaps(); }
  catch (error) { log(error.message); }
});
$("#save-map").addEventListener("click", async () => {
  try { const result = await api("/api/v1/maps/save", { method: "POST", body: JSON.stringify({ map_id: $("#save-map-id").value }) }); log(result.message); await refreshMaps(); }
  catch (error) { log(error.message); }
});
$("#send-goal").addEventListener("click", async () => {
  try {
    const result = await api("/api/v1/navigation/goal", { method: "POST", body: JSON.stringify({ x: $("#goal-x").value, y: $("#goal-y").value, yaw: $("#goal-yaw").value }) });
    log(result.message);
  } catch (error) { log(error.message); }
});
$$('[data-drive]').forEach((button) => {
  button.addEventListener("pointerdown", (event) => { event.preventDefault(); startDrive(button.dataset.drive); });
  button.addEventListener("pointerup", stopDrive);
  button.addEventListener("pointercancel", stopDrive);
  button.addEventListener("pointerleave", stopDrive);
});
window.addEventListener("blur", stopDrive);
document.addEventListener("visibilitychange", () => { if (document.hidden) stopDrive(); });
