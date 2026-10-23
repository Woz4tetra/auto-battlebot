"use strict";

const $ = (id) => document.getElementById(id);

// ---- leader rows -------------------------------------------------------------------------

// Builds "Label ..... value" rows into a container and returns {key: value span}.
function leaderRows(containerId, rows) {
  const spans = {};
  $(containerId).innerHTML = "";
  for (const [key, label] of rows) {
    const row = document.createElement("div");
    row.className = "leader";
    row.innerHTML = '<span class="k"></span><span class="dots"></span><span class="v">--</span>';
    row.firstChild.textContent = label;
    $(containerId).appendChild(row);
    spans[key] = row.lastChild;
  }
  return spans;
}

function setValue(span, text, bad) {
  span.textContent = text;
  span.classList.toggle("bad", !!bad);
}

// tone: "" plain, "warn" amber, "bad" red.
function setTone(span, text, tone) {
  setValue(span, text, tone === "bad");
  span.classList.toggle("warn", tone === "warn");
}

// Rows rebuilt on every status poll: [label, value, bad].
function fillRows(containerId, rows) {
  const spans = leaderRows(
    containerId,
    rows.map((r, i) => [i, r[0]]),
  );
  rows.forEach((r, i) => setValue(spans[i], r[1], r[2]));
}

// ---- theme -------------------------------------------------------------------------------

const SUN =
  '<svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="square"><circle cx="12" cy="12" r="4.5"/><path d="M12 1.5v3M12 19.5v3M1.5 12h3M19.5 12h3M4.6 4.6l2.1 2.1M17.3 17.3l2.1 2.1M4.6 19.4l2.1-2.1M17.3 6.7l2.1-2.1"/></svg>';
const MOON =
  '<svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M20.5 14.5A8.5 8.5 0 1 1 9.5 3.5a7 7 0 0 0 11 11z"/></svg>';

function renderTheme() {
  const dark = document.documentElement.dataset.theme === "dark";
  $("darkBtn").innerHTML = dark ? SUN : MOON;
  $("darkBtn").setAttribute("aria-pressed", dark);
}
$("darkBtn").onclick = () => {
  const dark = document.documentElement.dataset.theme !== "dark";
  document.documentElement.dataset.theme = dark ? "dark" : "light";
  try {
    localStorage.setItem("theme", dark ? "dark" : "light");
  } catch (e) {}
  renderTheme();
};
renderTheme();

// ---- live stream -------------------------------------------------------------------------

// Column order of the /events CSV rows. The Jetson's Esp32WifiDiagnostics and
// scripts/log_drive_twitch.py parse the same order. New columns go just before seq.
const HEADER =
  "timestamp_ms,radio_connected,armed,a_percent,b_percent,button_state,flip_switch,left_cmd,right_cmd,accel_x,accel_y,accel_z,is_upside_down,loop_us,wifi_clients,orientation_x,orientation_y,orientation_z,pid_setpoint,pid_output,vbat,ibat,yaw_rate,yaw_rate_cmd,motor_rise_left,motor_rise_right,motor_speed_left,motor_speed_right,motor_amps_left,motor_amps_right,guard_scale,guard_gain,guard_state,guard_sensors,stall_flags,erpm_crc_errors,seq";
const COL = Object.fromEntries(HEADER.split(",").map((name, i) => [name, i]));
const FLIP = ["DOWN", "MIDDLE", "UP"];

const drive = leaderRows("driveRows", [
  ["radio", "Radio link"],
  ["armed", "Armed"],
  ["a", "Throttle (a_percent)"],
  ["b", "Steer (b_percent)"],
  ["flip", "Flip switch"],
  ["button", "Button"],
  ["left", "Left command"],
  ["right", "Right command"],
]);
const imuLive = leaderRows("imuLiveRows", [
  ["usd", "Upside down"],
  ["heading", "Heading / roll / pitch"],
  ["accel", "Accel x / y / z"],
  ["yr", "Yaw rate, deg/s CW"],
  ["yrc", "Yaw rate command"],
  ["sp", "Heading setpoint"],
  ["po", "Steer output"],
]);
const power = leaderRows("powerRows", [
  ["vbat", "Pack voltage"],
  ["ibat", "Pack current"],
  ["loop", "Loop time"],
  ["clients", "AP clients"],
  ["ts", "Robot clock"],
  ["seq", "Row seq"],
]);

// Motor stall guard. Mirrors lib/motor_guard Config: rise_soft_c and rise_max_c.
const RISE_SOFT_C = 35;
const RISE_MAX_C = 50;
const GUARD_STATE = ["OFF", "NOMINAL", "HOT", "LIMITING"];
const GUARD_SENSORS = ["FUSED", "SHUNT ONLY", "ERPM ONLY", "NONE"];
const heat = leaderRows("heatRows", [
  ["state", "Guard state"],
  ["sensors", "Sensors"],
  ["scale", "Drive scale"],
  ["rise", `Rise L / R, of ${RISE_MAX_C} C`],
  ["amps", "Motor current L / R"],
  ["speed", "Motor speed L / R, rad/s"],
  ["stall", "Stalled wheel"],
  ["gain", "Shunt gain g"],
  ["crc", "eRPM decode errors"],
]);

function riseTone(rise) {
  if (!(rise < RISE_MAX_C)) return "bad";
  return rise > RISE_SOFT_C ? "warn" : "";
}

let latest = null;
let rows = [];
let recording = false;
let lastMessageMs = 0;

function yesNo(v) {
  return v === "1" ? "YES" : "NO";
}

function render() {
  requestAnimationFrame(render);
  const live = latest && performance.now() - lastMessageMs < 2000;
  const pill = $("linkPill");
  pill.firstChild.style.background = live ? "var(--green)" : "var(--red)";
  pill.lastChild.textContent = live ? "LIVE" : "OFFLINE";
  if (!latest) return;
  const f = latest;
  latest = null;
  const g = (name) => f[COL[name]];
  setValue(drive.radio, g("radio_connected") === "1" ? "UP" : "DOWN", g("radio_connected") !== "1");
  setValue(drive.armed, yesNo(g("armed")));
  setValue(drive.a, g("a_percent") + " %");
  setValue(drive.b, g("b_percent") + " %");
  setValue(drive.flip, FLIP[+g("flip_switch")] || g("flip_switch"));
  setValue(drive.button, g("button_state") === "1" ? "ON" : "OFF");
  setValue(drive.left, g("left_cmd") + " %");
  setValue(drive.right, g("right_cmd") + " %");
  setValue(imuLive.usd, yesNo(g("is_upside_down")));
  setValue(
    imuLive.heading,
    [g("orientation_x"), g("orientation_y"), g("orientation_z")].join(" / "),
  );
  setValue(imuLive.accel, [g("accel_x"), g("accel_y"), g("accel_z")].join(" / "));
  const state = GUARD_STATE[+g("guard_state")] || g("guard_state");
  setTone(heat.state, state, state === "OFF" ? "bad" : state === "NOMINAL" ? "" : "warn");
  const sensors = GUARD_SENSORS[+g("guard_sensors")] || g("guard_sensors");
  setTone(heat.sensors, sensors, sensors === "NONE" ? "bad" : sensors === "FUSED" ? "" : "warn");
  const scale = +g("guard_scale");
  setTone(heat.scale, (100 * scale).toFixed(0) + " %", scale < 1 ? "warn" : "");
  const riseL = +g("motor_rise_left");
  const riseR = +g("motor_rise_right");
  setTone(
    heat.rise,
    g("motor_rise_left") + " / " + g("motor_rise_right") + " C",
    riseTone(Math.max(riseL, riseR)),
  );
  setValue(heat.amps, g("motor_amps_left") + " / " + g("motor_amps_right") + " A");
  setValue(heat.speed, g("motor_speed_left") + " / " + g("motor_speed_right"));
  const stall = +g("stall_flags");
  setTone(heat.stall, ["none", "STALL L", "STALL R", "STALL L + R"][stall & 3], stall ? "bad" : "");
  const gain = +g("guard_gain");
  setTone(heat.gain, g("guard_gain"), gain < 0.9 || gain > 1.25 ? "warn" : "");
  setValue(heat.crc, g("erpm_crc_errors"));
  setValue(imuLive.yr, g("yaw_rate"));
  setValue(imuLive.yrc, g("yaw_rate_cmd") === "nan" ? "LOOP OFF" : g("yaw_rate_cmd"));
  setValue(imuLive.sp, g("pid_setpoint"));
  setValue(imuLive.po, g("pid_output"));
  setValue(power.vbat, g("vbat") === "nan" ? "NO SENSOR" : g("vbat") + " V", g("vbat") === "nan");
  setValue(power.ibat, g("ibat") === "nan" ? "NO SENSOR" : g("ibat") + " A", g("ibat") === "nan");
  setValue(power.loop, g("loop_us") + " us");
  setValue(power.clients, g("wifi_clients"));
  setValue(power.ts, (g("timestamp_ms") / 1000).toFixed(1) + " s");
  setValue(power.seq, g("seq"));
}

function connect() {
  const es = new EventSource("/events");
  es.onmessage = (e) => {
    lastMessageMs = performance.now();
    latest = e.data.split(",");
    if (recording) {
      rows.push(e.data);
      $("recCount").textContent = rows.length + " rows";
    }
  };
}

// While recording the robot sends every loop tick, so the DOM updates once per frame instead
// of once per message.
requestAnimationFrame(render);
connect();

// ---- recording ---------------------------------------------------------------------------

$("recBtn").onclick = () => {
  recording = !recording;
  $("recBtn").setAttribute("aria-pressed", recording);
  $("recBtn").textContent = recording ? "STOP" : "RECORD";
  if (recording) {
    rows = [];
    $("recCount").textContent = "";
    fetch("/record/start");
  } else {
    fetch("/record/stop");
    $("recCount").textContent = rows.length + " rows (stopped)";
  }
};

$("dlBtn").onclick = () => {
  if (rows.length === 0) {
    $("recCount").textContent = "nothing recorded";
    return;
  }
  const blob = new Blob([HEADER + "\n" + rows.join("\n") + "\n"], {
    type: "text/csv",
  });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = "mr_stabs_" + new Date().toISOString().slice(0, 19).replace(/[:-]/g, "") + ".csv";
  a.click();
  URL.revokeObjectURL(a.href);
};

// ---- ESC deadzone ------------------------------------------------------------------------

function flash(id, text) {
  $(id).textContent = text;
  setTimeout(() => {
    if ($(id).textContent === text) $(id).textContent = "";
  }, 4000);
}

for (const btn of document.querySelectorAll("[data-tune]")) {
  const input = btn.dataset.input;
  fetch("/tune/" + btn.dataset.tune)
    .then((r) => r.text())
    .then((v) => ($(input).value = v));
  btn.onclick = () =>
    fetch("/tune/" + btn.dataset.tune + "?value=" + encodeURIComponent($(input).value))
      .then((r) => r.text())
      .then((t) => flash(input + "S", "set to " + t));
}

// ---- WiFi --------------------------------------------------------------------------------

let wifi = null;
let pendingMode = null; // the operator's unapplied choice; null follows the robot
let pendingSelected = null;

function post(url, fields) {
  return fetch(url, { method: "POST", body: new URLSearchParams(fields) }).then((r) =>
    r.text().then((t) => {
      if (!r.ok) throw new Error(t);
      return t;
    }),
  );
}

const STATE_TEXT = {
  ap: "ACCESS POINT",
  connecting: "CONNECTING",
  connected: "CONNECTED",
  reconnecting: "RECONNECTING",
  fallback_ap: "FALLBACK ACCESS POINT",
};

function renderWifi() {
  if (!wifi) return;
  const mode = pendingMode ?? wifi.mode;
  const selected = pendingSelected ?? wifi.selected;

  // ESP-NOW pins the robot to its access point, so client mode cannot be chosen.
  const locked = wifi.forced_ap;
  $("espnowLockout").hidden = !locked;
  $("lockoutSsid").textContent = wifi.ap_ssid;
  $("lockoutChannel").textContent = wifi.ap_channel ? "channel " + wifi.ap_channel : "pinned";
  for (const b of $("modeSeg").children) {
    b.setAttribute("aria-pressed", b.dataset.mode === (locked ? "ap" : mode));
    b.disabled = locked;
  }
  $("netSelect").disabled = locked;

  // Rebuilt only when the list changes, so a poll does not close a dropdown the operator has open.
  const select = $("netSelect");
  const key = wifi.networks.join("\n");
  if (select.dataset.key !== key) {
    select.dataset.key = key;
    select.innerHTML = "";
    for (const ssid of wifi.networks) {
      const opt = document.createElement("option");
      opt.value = opt.textContent = ssid;
      select.appendChild(opt);
    }
    if (wifi.networks.length === 0) {
      const opt = document.createElement("option");
      opt.textContent = "no saved networks";
      opt.value = "";
      select.appendChild(opt);
    }
  }
  select.value = wifi.networks.includes(selected) ? selected : wifi.networks[0] || "";
  $("selectField").hidden = mode !== "client";

  // The lockout box above explains forced AP mode; the note would only repeat it.
  $("modeNote").hidden = locked;
  $("modeNote").textContent =
    mode === "ap"
      ? `Boots as access point ${wifi.ap_ssid} at 192.168.4.1 and never joins a router.`
      : `Joins the selected network on boot. With no connection after 30 s it falls back to access point ${wifi.ap_ssid}. Once joined, find it at http://${wifi.hostname}.local or by its router address.`;
  $("applyBtn").disabled = locked || (mode === "client" && wifi.networks.length === 0);

  const pill = $("wifiPill");
  const isAp = wifi.state === "ap" || wifi.state === "fallback_ap";
  pill.lastChild.textContent = isAp ? (locked ? "AP · ESP-NOW" : "AP") : "CLIENT";
  pill.firstChild.style.background =
    wifi.state === "connected" || wifi.state === "ap"
      ? "var(--green)"
      : wifi.state === "fallback_ap"
        ? "var(--red)"
        : "var(--amber)";

  let state = STATE_TEXT[wifi.state] || wifi.state;
  if (wifi.state === "connecting") state += ` (${wifi.connect_s_left} s left)`;
  const rows = [["State", state, wifi.state === "fallback_ap"]];
  if (isAp) {
    rows.push(["Network", wifi.ap_ssid], ["Address", wifi.ip], ["Clients", String(wifi.stations)]);
  } else {
    rows.push(
      ["Network", wifi.ssid || "--"],
      ["Address", wifi.ip || "--"],
      ["Signal", wifi.state === "connected" ? wifi.rssi + " dBm" : "--"],
    );
  }
  rows.push(["Hostname", wifi.hostname + ".local"]);
  fillRows("wifiRows", rows);

  const list = $("netList");
  list.innerHTML = "";
  if (wifi.networks.length === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "None saved yet.";
    list.appendChild(li);
  }
  for (const ssid of wifi.networks) {
    const li = document.createElement("li");
    li.innerHTML = '<span class="ssid"></span><button class="btn small">DELETE</button>';
    li.firstChild.textContent = ssid;
    li.lastChild.onclick = () =>
      post("/wifi/network/delete", { ssid })
        .then(pollWifi)
        .catch((e) => flash("addStatus", e.message));
    list.appendChild(li);
  }
}

function pollWifi() {
  return fetch("/wifi")
    .then((r) => r.json())
    .then((w) => {
      wifi = w;
      renderWifi();
    })
    .catch(() => {});
}

for (const b of $("modeSeg").children) {
  b.onclick = () => {
    pendingMode = b.dataset.mode;
    renderWifi();
  };
}
$("netSelect").onchange = () => {
  pendingSelected = $("netSelect").value;
};

$("applyBtn").onclick = () => {
  if (!wifi) return;
  const mode = pendingMode ?? wifi.mode;
  const selected = $("netSelect").value;
  post("/wifi/config", { mode, selected })
    .then(() => {
      pendingMode = pendingSelected = null;
      const leaving = mode === "client" || wifi.state !== "ap";
      $("applyStatus").textContent = leaving
        ? "saved; the robot is switching networks and this page will lose its connection"
        : "saved";
      pollWifi();
    })
    .catch((e) => flash("applyStatus", e.message));
};

$("addForm").onsubmit = (e) => {
  e.preventDefault();
  const ssid = $("addSsid").value.trim();
  post("/wifi/network", { ssid, password: $("addPass").value })
    .then(() => {
      $("addSsid").value = $("addPass").value = "";
      flash("addStatus", "saved " + ssid);
      pollWifi();
    })
    .catch((err) => flash("addStatus", err.message));
};

setInterval(pollWifi, 2000);
pollWifi();

// ---- sensor status -----------------------------------------------------------------------

const I2C_ERR = {
  0: "ok",
  1: "data too long",
  2: "address NACK: device not answering",
  3: "data NACK",
  4: "bus error",
  5: "timeout: bus stuck?",
  6: "short read",
  255: "never tried",
};
const KNOWN = {
  0x28: "BNO055",
  0x29: "BNO055 (ADR high)",
  0x45: "INA238",
};
const SYS_STATUS = [
  "idle",
  "system error",
  "initializing peripherals",
  "system init",
  "running self-test",
  "fusion running",
  "running without fusion",
];
const SYS_ERR = [
  "none",
  "peripheral init error",
  "system init error",
  "self-test failed",
  "register value out of range",
  "register address out of range",
  "register write error",
  "low power mode not available",
  "accel power mode not available",
  "fusion config error",
  "sensor config error",
];
const MODES = { 0: "CONFIG", 8: "IMUPLUS", 12: "NDOF" };

const hex = (v, n) =>
  "0x" +
  v
    .toString(16)
    .toUpperCase()
    .padStart(n || 2, "0");
const age = (ms) => (ms == null ? "never" : (ms / 1000).toFixed(1) + " s ago");
const err = (c) => [I2C_ERR[c] || "error " + c, c !== 0 && c !== 255];

function selfTest(v) {
  const failed = ["accel", "mag", "gyro", "MCU"].filter((_, i) => !((v >> i) & 1));
  return [
    failed.length ? "failed: " + failed.join(", ") : "all passed (" + hex(v) + ")",
    failed.length > 0,
  ];
}
function calib(v) {
  return `sys ${(v >> 6) & 3}, gyro ${(v >> 4) & 3}, accel ${(v >> 2) & 3}, mag ${v & 3} (of 3)`;
}

let prevImu = null;
function sampleRate(s) {
  const p = prevImu;
  prevImu = { t: s.snapshot_ms, n: s.imu.samples };
  if (!p || s.snapshot_ms <= p.t || s.imu.samples < p.n) return "--";
  return (((s.imu.samples - p.n) * 1000) / (s.snapshot_ms - p.t)).toFixed(0) + " Hz";
}

function renderStatus(s) {
  $("busPins").textContent = `Wire1, SDA ${s.bus.sda_pin}, SCL ${s.bus.scl_pin}`;
  const found =
    s.bus.found.map((a) => hex(a) + (KNOWN[a] ? " " + KNOWN[a] : "")).join(", ") || "nothing";
  fillRows("busRows", [
    ["SDA idle level", s.bus.sda_high ? "high" : "LOW: held by a device", !s.bus.sda_high],
    ["SCL idle level", s.bus.scl_high ? "high" : "LOW: held by a device", !s.bus.scl_high],
    [
      "Devices found",
      s.bus.scanned ? found : "not scanned",
      s.bus.scanned && !s.bus.found.includes(0x28),
    ],
    ["Last scan", s.bus.scanned ? age(s.bus.scan_age_ms) : "never"],
  ]);
  $("scanStatus").textContent = s.bus.scan_pending ? "scan queued (runs while disarmed)" : "";

  const m = s.imu;
  const imuRows = [
    ["Initialized", m.initialized ? "yes" : "NO", !m.initialized],
    [
      "Begin attempts / failures",
      m.begin_attempts + " / " + m.begin_failures,
      m.begin_failures > 0,
    ],
    [
      "Chip id",
      m.last_error === 255
        ? "not read"
        : hex(m.chip_id) + (m.chip_id === 0xa0 ? " (BNO055)" : " (expected 0xA0)"),
      m.last_error === 0 && m.chip_id !== 0xa0,
    ],
    ["Last chip id read", ...err(m.last_error)],
    ["Dropouts after init", String(m.lost_count), m.lost_count > 0],
    ["Samples", String(m.samples)],
    ["Sample rate (expect ~100 Hz)", sampleRate(s)],
    [
      "Last sample",
      m.sample_age_ms == null ? "never" : m.sample_age_ms + " ms before snapshot",
      m.initialized && (m.sample_age_ms == null || m.sample_age_ms > 100),
    ],
  ];
  if (m.details_age_ms != null) {
    if (m.details_error !== 0) imuRows.push(["Status registers", ...err(m.details_error)]);
    else
      imuRows.push(
        [
          "Operation mode",
          (MODES[m.operation_mode] || "mode") + " (" + hex(m.operation_mode) + ")",
          m.operation_mode !== 8,
        ],
        ["System status", SYS_STATUS[m.sys_status] || hex(m.sys_status), m.sys_status !== 5],
        ["System error", SYS_ERR[m.sys_error] || hex(m.sys_error), m.sys_error !== 0],
        ["Self-test", ...selfTest(m.self_test)],
        ["Calibration", calib(m.calibration)],
        ["Status read", age(m.details_age_ms)],
      );
  }
  imuRows.push(
    [
      "Yaw rate: gyro / heading",
      m.yaw_rate.toFixed(0) + " / " + m.heading_rate.toFixed(0) + " deg/s CW",
    ],
    [
      "Gyro sign check (spin it)",
      m.gyro_sign_suspect
        ? "FAILED: yaw loop off, flip YAW_RATE_SIGN"
        : m.gyro_agree + m.gyro_disagree === 0
          ? "no votes yet"
          : "agree " + m.gyro_agree + ", disagree " + m.gyro_disagree,
      m.gyro_sign_suspect,
    ],
  );
  fillRows("imuRows", imuRows);

  const n = s.ina;
  fillRows("inaRows", [
    ["Present", n.present ? "yes" : "NO", !n.present],
    [
      "Device id",
      hex(n.device_id, 4) + (n.device_id >> 4 === 0x238 ? " (INA238)" : " (expected 0x238x)"),
      n.device_id >> 4 !== 0x238,
    ],
    ["Last read", ...err(n.last_error)],
    ["Reads / failures", n.reads + " / " + n.read_failures, n.read_failures > 0],
    ["Last good read", age(n.last_ok_age_ms)],
  ]);

  if (s.espnow) renderEspNow(s.espnow, s.snapshot_ms);
  if (s.boot) renderBoot(s.boot);
}

// ---- last reboot -------------------------------------------------------------------------

// Reset reasons that mean the firmware failed rather than someone rebooting it.
const BAD_RESETS = ["PANIC", "TASK_WDT", "INT_WDT", "OTHER_WDT", "BROWNOUT"];

function renderBoot(b) {
  const rows = [["Reset reason", b.reset_reason, BAD_RESETS.includes(b.reset_reason)]];
  if (!b.has_dump) {
    rows.push(["Core dump", "none"]);
  } else if (!b.dump_valid) {
    rows.push(["Core dump", b.dump_size + " B, checksum failed (cut short)", true]);
  } else {
    rows.push(
      ["Core dump", b.dump_size + " B", true],
      // A task watchdog aborts from its interrupt on core 0, so the summary names whatever ran
      // there. The stuck task is in the full decode (scripts/coredump).
      [
        b.reset_reason === "TASK_WDT" ? "Task interrupted (not the stuck one)" : "Task",
        b.task || "--",
      ],
      ["PC", b.pc],
      ["Exception cause / addr", b.exc_cause + " / " + b.exc_vaddr],
      ["Backtrace", b.backtrace.join(" ") + (b.backtrace_corrupted ? " (corrupted)" : "")],
      ["Build ELF SHA-256", b.elf_sha256],
    );
  }
  fillRows("bootRows", rows);
  $("eraseDumpBtn").disabled = !b.has_dump;
}

$("eraseDumpBtn").onclick = () =>
  post("/crash/erase", {})
    .then(() => flash("eraseDumpStatus", "erasing..."))
    .catch((e) => flash("eraseDumpStatus", e.message));

// ---- ESP-NOW -----------------------------------------------------------------------------

const OTA_STATES = ["idle", "receiving", "done", "validated", "error"];
const PAIR_RESULTS = [
  "",
  "saved",
  "disarm first",
  "list full (8 paired)",
  "already allowed",
  "",
  "not paired",
  "not a device MAC",
];

function pairRequest(url, mac) {
  return post(url, { mac: mac.trim().toLowerCase() })
    .then(() => flash("pairStatus", "sent..."))
    .catch((err) => flash("pairStatus", err.message));
}

$("pairBtn").onclick = () => pairRequest("/espnow/pair", $("pairOfferMac").textContent);
$("pairManualBtn").onclick = () => pairRequest("/espnow/pair", $("pairMac").value);

let lastPairResult = 0;
let lastDongleList = "";

function renderDongles(e) {
  // A refused dongle is the one to offer, unless it was paired since.
  const offer = e.rx_rejected > 0 && !e.allowed.includes(e.rejected_mac) ? e.rejected_mac : null;
  $("pairOffer").hidden = !offer;
  if (offer) $("pairOfferMac").textContent = offer;

  if (e.pair_result !== lastPairResult) {
    lastPairResult = e.pair_result;
    if (PAIR_RESULTS[e.pair_result]) flash("pairStatus", PAIR_RESULTS[e.pair_result]);
  }

  // Rebuilt only when the list changes, so a button is never replaced under a click.
  const key = e.allowed.join(",") + "|" + e.dongle_mac;
  if (key === lastDongleList) return;
  lastDongleList = key;
  const list = $("dongleList");
  list.innerHTML = "";
  if (e.allowed.length === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "None paired. The robot accepts no dongle until one is.";
    list.appendChild(li);
  }
  for (const mac of e.allowed) {
    // MAC on its own line and the state under it, so neither is cut off in a narrow column.
    const li = document.createElement("li");
    li.innerHTML =
      '<span class="ssid dongle"><span></span><span class="muted"></span></span>' +
      '<button class="btn small">REMOVE</button>';
    const label = li.firstChild;
    label.firstChild.textContent = mac;
    label.lastChild.textContent =
      e.heard_dongle && mac === e.dongle_mac ? "paired, talking now" : "paired";
    li.lastChild.onclick = () => pairRequest("/espnow/unpair", mac);
    list.appendChild(li);
  }
}
let prevEspNow = null;

// Per-second rates from the change since the previous status poll.
function espnowRates(e, snapshotMs) {
  const p = prevEspNow;
  prevEspNow = { t: snapshotMs, e };
  if (!p || snapshotMs <= p.t || e.rx_frames < p.e.rx_frames) return null;
  const dt = (snapshotMs - p.t) / 1000;
  const rate = (k) => (e[k] - p.e[k]) / dt;
  const sent = e.tx_ok - p.e.tx_ok + (e.tx_fail - p.e.tx_fail);
  return {
    rx: rate("rx_frames"),
    cmdNew: rate("rx_commands_new"),
    cmdRepeat: rate("rx_commands_repeat"),
    ping: rate("rx_pings"),
    tlm: rate("tx_telemetry"),
    ack: sent > 0 ? (100 * (e.tx_ok - p.e.tx_ok)) / sent : null,
  };
}

function renderEspNow(e, snapshotMs) {
  const r = espnowRates(e, snapshotMs);
  const perS = (v) => (r ? v.toFixed(0) + " /s" : "--");
  if (!e.enabled) {
    $("espnowSub").textContent = "off in this build";
    fillRows("espnowLinkRows", [
      ["ESPNOW_ENABLED", "false: no ESP-NOW init, telemetry, or OTA", true],
    ]);
    fillRows("espnowTrafficRows", []);
    return;
  }
  $("espnowSub").textContent = e.substitution
    ? "drives from ESP-NOW"
    : "shadow mode: receives, never drives";
  const live = e.frame_age_ms != null && e.frame_age_ms < 500;
  fillRows("espnowLinkRows", [
    ["Started", e.begin_ok ? "yes" : "esp_now_init FAILED", !e.begin_ok],
    [
      "Channel",
      e.sweeping
        ? "sweeping (last good " + (e.last_good_channel || "none") + ")"
        : String(e.channel),
      e.sweeping,
    ],
    ["Dongle", e.heard_dongle ? e.dongle_mac : "never heard", !e.heard_dongle],
    ["Last dongle frame", e.frame_age_ms == null ? "never" : e.frame_age_ms + " ms ago", !live],
    [
      "Refused, unpaired dongle",
      String(e.rx_rejected),
      e.rx_rejected > 0 && !e.allowed.includes(e.rejected_mac),
    ],
    [
      "Last refused",
      e.rx_rejected === 0
        ? "none"
        : e.rejected_mac + (e.allowed.includes(e.rejected_mac) ? " (paired since)" : ""),
      e.rx_rejected > 0 && !e.allowed.includes(e.rejected_mac),
    ],
    [
      "Telemetry ACK (LQ, last 32)",
      e.heard_dongle ? e.lq + " %" : "--",
      e.heard_dongle && e.lq < 90,
    ],
    [
      "Command",
      e.command_age_ms == null
        ? "none yet"
        : `seq ${e.command_seq}, ${e.command_age_ms} ms ago` + (e.command_fresh ? ", fresh" : ""),
    ],
    [
      "Command a / b",
      e.command_age_ms == null ? "--" : e.command_a.toFixed(2) + " / " + e.command_b.toFixed(2),
    ],
    ["Driving from ESP-NOW", e.driving ? "YES" : "no"],
    [
      "OTA",
      (OTA_STATES[e.ota_state] || "state " + e.ota_state) +
        (e.validated && e.ota_state !== 3 ? ", this boot validated" : ""),
      e.ota_state === 4,
    ],
  ]);
  fillRows("espnowTrafficRows", [
    ["Received", e.rx_frames + " (" + perS(r ? r.rx : 0) + ")"],
    [
      "Commands new / repeat",
      r
        ? r.cmdNew.toFixed(0) + " / " + r.cmdRepeat.toFixed(0) + " /s"
        : e.rx_commands_new + " / " + e.rx_commands_repeat,
    ],
    ["Commands dropped late", String(e.rx_commands_old), e.rx_commands_old > 0],
    ["Pings", e.rx_pings + " (" + perS(r ? r.ping : 0) + ")"],
    ["Tunables / OTA frames", e.rx_tunables + " / " + e.rx_ota],
    [
      "Wrong-size frames",
      String(e.rx_size_errors) +
        (e.rx_size_errors ? ": robot and dongle built from different headers" : ""),
      e.rx_size_errors > 0,
    ],
    ["Receive queue overflows", String(e.rx_queue_full), e.rx_queue_full > 0],
    ["Telemetry sent", e.tx_telemetry + " (" + perS(r ? r.tlm : 0) + ")"],
    [
      "Send ACK, last poll",
      r && r.ack != null ? r.ack.toFixed(1) + " %" : "--",
      r && r.ack != null && r.ack < 90,
    ],
    ["Sends ACKed / failed", e.tx_ok + " / " + e.tx_fail, e.tx_fail > 0],
    ["esp_now_send refused", String(e.tx_errors), e.tx_errors > 0],
  ]);
  renderDongles(e);
}

function pollStatus() {
  fetch("/status")
    .then((r) => r.json())
    .then(renderStatus)
    .catch(() => {});
}
$("scanBtn").onclick = () =>
  fetch("/i2c/scan").then(() => ($("scanStatus").textContent = "scan queued"));
setInterval(pollStatus, 1000);
pollStatus();
