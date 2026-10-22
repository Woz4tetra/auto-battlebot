"use strict";

const $ = (id) => document.getElementById(id);
const NONE = "--";

// ---- formatting --------------------------------------------------------------------------

const num = (v, digits = 1, unit = "") =>
  v === null || v === undefined
    ? NONE
    : v.toFixed(digits) + (unit ? " " + unit : "");
const int = (v, unit = "") =>
  v === null || v === undefined
    ? NONE
    : Math.round(v).toLocaleString() + (unit ? " " + unit : "");
const yesNo = (v) => (v === null || v === undefined ? NONE : v ? "YES" : "NO");
const ago = (s) =>
  s === null || s === undefined
    ? NONE
    : s < 1
      ? `${Math.round(s * 1000)} ms ago`
      : `${s.toFixed(1)} s ago`;
const duration = (s) => {
  if (s === null || s === undefined) return NONE;
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = Math.floor(s % 60);
  return h ? `${h}h ${m}m` : m ? `${m}m ${sec}s` : `${sec}s`;
};
const clock = (t) => new Date(t * 1000).toTimeString().slice(0, 8);
const esc = (s) =>
  String(s).replace(
    /[&<>"]/g,
    (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c],
  );

// ---- leader rows -------------------------------------------------------------------------

// Rows: [label, value, tone] where tone is "bad", "warn", "good", or falsy.
function fillRows(id, rows) {
  const el = $(id);
  if (el.childElementCount !== rows.length) {
    el.innerHTML = rows
      .map(
        () =>
          '<div class="leader"><span class="k"></span><span class="dots"></span><span class="v"></span></div>',
      )
      .join("");
  }
  rows.forEach(([label, value, tone], i) => {
    const row = el.children[i];
    row.children[0].textContent = label;
    const v = row.children[2];
    v.textContent = value;
    v.className = "v" + (tone ? " " + tone : "");
  });
}

const pctTone = (v, good, bad) =>
  v === null || v === undefined
    ? ""
    : v >= good
      ? "good"
      : v < bad
        ? "bad"
        : "warn";

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

// ---- header ------------------------------------------------------------------------------

function setPill(id, text, color) {
  const pill = $(id);
  pill.firstChild.style.background = color;
  pill.lastChild.textContent = text;
}

function renderHeader(s, live) {
  $("portSub").textContent = s ? s.port.path : "dongle diagnostics";
  if (!live) {
    setPill("portPill", "PAGE OFFLINE", "var(--red)");
    setPill("robotPill", NONE, "var(--faint)");
    return;
  }
  const portColor = s.port.state === "OPEN" ? "var(--green)" : "var(--red)";
  setPill("portPill", s.port.state, portColor);
  const tlmLive =
    s.telemetry_metrics.age_s !== null && s.telemetry_metrics.age_s < 0.5;
  if (!s.link) setPill("robotPill", "NO DONGLE", "var(--red)");
  else if (!s.link.robot_known)
    setPill("robotPill", "BROADCAST", "var(--amber)");
  else
    setPill(
      "robotPill",
      tlmLive ? "ROBOT" : "ROBOT STALE",
      tlmLive ? "var(--green)" : "var(--red)",
    );

  const lm = s.link_metrics;
  const tm = s.telemetry_metrics;
  const cells = [
    ["CH", s.link ? String(s.link.channel) : NONE, ""],
    ["ACK %", num(lm.ack_pct_1s, 0), pctTone(lm.ack_pct_1s, 95, 80)],
    [
      "TLM HZ",
      s.telemetry ? String(tm.hz) : NONE,
      s.telemetry && tm.hz < 80 ? "bad" : "",
    ],
    [
      "DELIV %",
      num(tm.delivery_pct_5s, 0),
      pctTone(tm.delivery_pct_5s, 98, 90),
    ],
    [
      "LQ %",
      s.telemetry ? String(s.telemetry.espnow_lq) : NONE,
      s.telemetry ? pctTone(s.telemetry.espnow_lq, 95, 80) : "",
    ],
    ["RTT MS", num(s.command.rtt_ms.min, 1), ""],
  ];
  $("metrics").innerHTML = cells
    .map(
      ([k, v, tone]) =>
        `<div class="cell"><span class="k">${k}</span><span class="v ${tone === "good" ? "" : tone}">${esc(v)}</span></div>`,
    )
    .join("");
}

// ---- sections ----------------------------------------------------------------------------

function renderChecks(s) {
  // Checks after the first error are not shown by the server; list them greyed so the shape of
  // the checklist stays put.
  const ALL = [
    "Serial port",
    "Dongle link stats",
    "Robot heard",
    "Robot telemetry",
    "Channel match",
    "Dongle to robot ACK",
    "Robot to dongle delivery",
    "Robot telemetry LQ",
    "Channel to ourselves",
    "Serial CRC",
    "Robot accepts commands",
  ];
  const byName = Object.fromEntries(s.checks.map((c) => [c.name, c]));
  $("checks").innerHTML = ALL.map((name) => {
    const c = byName[name];
    if (!c)
      return `<div class="check skipped"><span class="badge">--</span><span class="name">${name}</span><span class="val">--</span></div>`;
    const badge = { ok: "OK", warn: "WARN", error: "FAIL", info: "INFO" }[
      c.level
    ];
    return (
      `<div class="check"><span class="badge ${c.level}">${badge}</span>` +
      `<span class="name">${esc(c.name)}</span><span class="val">${esc(c.value)}</span>` +
      (c.hint ? `<span class="hint">${esc(c.hint)}</span>` : "") +
      "</div>"
    );
  }).join("");
}

function renderPort(s) {
  const p = s.port;
  fillRows("portRows", [
    ["State", p.state, p.state === "OPEN" ? "good" : "bad"],
    ["Error", p.error || "none", p.error ? "bad" : ""],
    ["Open for", duration(p.open_s)],
    ["Opens", String(p.opens), p.opens > 1 ? "warn" : ""],
    ["Received", int(p.rx_bytes, "B")],
    ["Rate", int(p.rx_bytes_per_s, "B/s")],
    ["Frames", int(p.frames)],
    ["CRC errors", int(p.crc_errors), p.crc_errors ? "warn" : ""],
    ["Bytes outside frames", int(p.junk_bytes)],
    ["Dongle MAC (banner)", s.banner ? s.banner.mac : "not seen"],
    ["Tool uptime", duration(s.uptime_s)],
  ]);
}

function renderDongle(s) {
  const l = s.link;
  const m = s.link_metrics;
  fillRows("dongleRows", [
    ["Channel", l ? String(l.channel) : NONE],
    [
      "Mode",
      l ? (l.robot_known ? "UNICAST" : "BROADCAST, no ACKs") : NONE,
      l && !l.robot_known ? "warn" : "",
    ],
    ["Sent", l ? int(l.sent) : NONE],
    ["Acked", l ? int(l.acked) : NONE],
    ["Failed", l ? int(l.failed) : NONE, l && l.failed ? "warn" : ""],
    ["Sent / s", num(m.sent_hz, 0)],
    ["Failed / s", num(m.failed_hz, 1), m.failed_hz > 0 ? "warn" : ""],
    ["ACK, last 1 s", num(m.ack_pct_1s, 1, "%"), pctTone(m.ack_pct_1s, 95, 80)],
    [
      "ACK, since robot heard",
      num(m.ack_pct_session, 1, "%"),
      pctTone(m.ack_pct_session, 95, 80),
    ],
    ["Send callback (last)", l ? int(l.last_rtt_us, "us") : NONE],
    ["Stats rate", `${m.hz} Hz`, l && m.hz < 8 ? "warn" : ""],
    ["Stats age", ago(m.age_s), m.age_s > 0.5 ? "bad" : ""],
    [
      "Longest stats gap",
      num(m.gap_max_ms, 0, "ms"),
      m.gap_max_ms > 300 ? "warn" : "",
    ],
    [
      "Dongle reboots seen",
      String(m.dongle_reboots),
      m.dongle_reboots ? "warn" : "",
    ],
  ]);
}

function renderRobotLink(s) {
  const t = s.telemetry;
  const m = s.telemetry_metrics;
  const dongleCh = s.link ? s.link.channel : null;
  fillRows("robotLinkRows", [
    [
      "Telemetry",
      ago(m.age_s),
      m.age_s === null || m.age_s > 0.5 ? "bad" : "good",
    ],
    ["Rate", t ? `${m.hz} Hz` : NONE, t && m.hz < 80 ? "warn" : ""],
    ["Received", int(m.received)],
    ["Lost (seq gaps)", int(m.dropped), m.dropped ? "warn" : ""],
    [
      "Delivery, last 5 s",
      num(m.delivery_pct_5s, 1, "%"),
      pctTone(m.delivery_pct_5s, 98, 90),
    ],
    [
      "Delivery, session",
      num(m.delivery_pct_session, 2, "%"),
      pctTone(m.delivery_pct_session, 98, 90),
    ],
    [
      "Robot LQ (its ACKs)",
      t ? `${t.espnow_lq} %` : NONE,
      t ? pctTone(t.espnow_lq, 95, 80) : "",
    ],
    [
      "Robot channel",
      t ? (t.espnow_channel ? String(t.espnow_channel) : "SWEEPING") : NONE,
      t && dongleCh !== null && t.espnow_channel !== dongleCh ? "bad" : "",
    ],
    ["Autonomy switch (SA)", t ? (t.autonomy_enabled ? "ON" : "OFF") : NONE],
    [
      "ESP-NOW driving",
      yesNo(t && t.espnow_active),
      t && t.espnow_active ? "good" : "",
    ],
    [
      "Since last command",
      t
        ? t.espnow_age_ms === 4294967295
          ? "never"
          : int(t.espnow_age_ms, "ms")
        : NONE,
    ],
    ["telemetry_seq", t ? int(t.telemetry_seq) : NONE],
    [
      "Robot reboots seen",
      String(m.robot_reboots),
      m.robot_reboots ? "warn" : "",
    ],
    ["Robot uptime", t ? duration(t.timestamp_ms / 1000) : NONE],
  ]);
}

function renderTiming(s) {
  const m = s.telemetry_metrics;
  const h = m.host_gap_ms;
  fillRows("timingRows", [
    ["Host gap p50 / p90", `${num(h.p50, 1)} / ${num(h.p90, 1)} ms`],
    [
      "Host gap p99 / max",
      `${num(h.p99, 1)} / ${num(h.max, 1)} ms`,
      h.max > 100 ? "warn" : "",
    ],
    [
      "Robot interval p50 / max",
      `${num(m.robot_interval_ms.p50, 1)} / ${num(m.robot_interval_ms.max, 1)} ms`,
    ],
    ["Gaps over 100 ms, session", int(m.big_gaps), m.big_gaps ? "warn" : ""],
    ["Robot clock drift", num(m.clock_drift_ms, 1, "ms")],
  ]);
  const hist = m.gap_histogram;
  const max = Math.max(1, ...hist.map((b) => b.count));
  $("hist").innerHTML = hist
    .map((b, i) => {
      const w = (100 * b.count) / max;
      return (
        `<span>${b.label} ms</span><span><span class="bar ${i >= 5 ? "late" : ""}" style="display:block;width:${w}%"></span></span>` +
        `<span class="n">${b.count}</span>`
      );
    })
    .join("");
}

function renderCommand(s) {
  const c = s.command;
  $("cmdHz").textContent = c.hz.toFixed(0);
  const btn = $("sendBtn");
  btn.setAttribute("aria-pressed", c.sending);
  btn.textContent = c.sending ? "STOP" : "SEND COMMANDS";
  btn.disabled = s.port.state !== "OPEN";
  sending = c.sending;
  for (const id of ["sliderA", "sliderB"]) $(id).disabled = !c.sending;
  if (!c.sending && (Number($("sliderA").value) || Number($("sliderB").value)))
    setNeutral();
  const lag = c.robot_seq !== null && c.sending ? c.seq - c.robot_seq : null;
  // What the robot should report for what the tool is sending: the firmware writes
  // a_percent = -100 * channel_a and b_percent = 100 * channel_b, then negates a_percent upside
  // down. A mismatch while ESP-NOW is driving means a sign or mapping error.
  const t = s.telemetry;
  const driving = t && t.espnow_active;
  const expA = t ? -100 * c.a * (t.is_upside_down ? -1 : 1) : null;
  const expB = 100 * c.b;
  const mismatch =
    driving &&
    (Math.abs(t.a_percent - expA) > 1.5 || Math.abs(t.b_percent - expB) > 1.5);
  fillRows("cmdRows", [
    ["Sending a / b", c.sending ? `${num(c.a, 2)} / ${num(c.b, 2)}` : "off"],
    [
      "Robot driving from ESP-NOW",
      t
        ? driving
          ? "YES"
          : t.armed
            ? t.autonomy_enabled
              ? "NO (stale)"
              : "NO, SA off"
            : "NO, disarmed"
        : NONE,
      driving ? "good" : "",
    ],
    [
      "Robot drive a / b %",
      t ? `${num(t.a_percent, 1)} / ${num(t.b_percent, 1)}` : NONE,
      mismatch ? "bad" : "",
    ],
    [
      "Expected while driving",
      c.sending ? `${num(expA, 1)} / ${num(expB, 1)}` : NONE,
    ],
    [
      "Robot wheels left / right %",
      t ? `${num(t.left_cmd, 1)} / ${num(t.right_cmd, 1)}` : NONE,
    ],
    ["Our seq", int(c.seq)],
    ["Written to dongle", int(c.sent)],
    ["Robot accepted seq", c.robot_seq === null ? NONE : int(c.robot_seq)],
    ["Seq lag", lag === null ? NONE : String(lag), lag > 3 ? "warn" : ""],
    ["Seqs skipped by robot", int(c.seq_jumps), c.seq_jumps ? "warn" : ""],
    ["Echoes not from us", int(c.foreign_echo)],
    ["RTT min / p50", `${num(c.rtt_ms.min, 1)} / ${num(c.rtt_ms.p50, 1)} ms`],
    ["RTT p90 / max", `${num(c.rtt_ms.p90, 1)} / ${num(c.rtt_ms.max, 1)} ms`],
    ["RTT samples, 10 s", String(c.rtt_ms.n)],
  ]);
}

const FLIP = ["DOWN", "MIDDLE", "UP"];
const sensor = (v, unit, digits = 2) =>
  v === null ? "NO SENSOR" : num(v, digits, unit);
const triple = (a, b, c, digits = 2) =>
  [a, b, c].map((v) => num(v, digits)).join(" / ");

function renderRobotState(s) {
  const t = s.telemetry;
  if (!t) {
    fillRows("stateRows", [["Telemetry", "none yet"]]);
    fillRows("imuRows", [["Telemetry", "none yet"]]);
    return;
  }
  fillRows("stateRows", [
    ["Armed", yesNo(t.armed), t.armed ? "warn" : ""],
    [
      "RC radio",
      t.radio_connected ? "UP" : "DOWN",
      t.radio_connected ? "" : "bad",
    ],
    ["Drive a / b", `${num(t.a_percent, 1)} / ${num(t.b_percent, 1)} %`],
    ["Left / right", `${num(t.left_cmd, 1)} / ${num(t.right_cmd, 1)} %`],
    ["Flip switch", FLIP[t.flip_switch] || String(t.flip_switch)],
    ["Button", t.button_state ? "ON" : "OFF"],
    ["Upside down", yesNo(t.is_upside_down)],
    ["Pack voltage", sensor(t.vbat, "V"), t.vbat === null ? "bad" : ""],
    ["Pack current", sensor(t.ibat, "A"), t.ibat === null ? "bad" : ""],
    ["Loop time", int(t.loop_us, "us")],
  ]);
  const cal = t.imu_calibration;
  const calText = `sys ${(cal >> 6) & 3} gyro ${(cal >> 4) & 3} acc ${(cal >> 2) & 3} mag ${cal & 3}`;
  fillRows("imuRows", [
    ["Calibration", calText, ((cal >> 6) & 3) < 3 ? "warn" : ""],
    [
      "Heading / roll / pitch",
      triple(t.orientation_x, t.orientation_y, t.orientation_z, 1),
    ],
    ["Gravity x / y / z", triple(t.accel_x, t.accel_y, t.accel_z)],
    [
      "Linear accel x / y / z",
      triple(t.linear_accel_x, t.linear_accel_y, t.linear_accel_z),
    ],
    ["Gyro x / y", `${num(t.gyro_x, 1)} / ${num(t.gyro_y, 1)} deg/s`],
    ["Yaw rate", num(t.yaw_rate, 1, "deg/s CW")],
    [
      "Yaw rate command",
      t.yaw_rate_cmd === null ? "LOOP OFF" : num(t.yaw_rate_cmd, 1, "deg/s"),
    ],
    ["Heading setpoint", num(t.pid_setpoint, 1)],
    ["Steer output", num(t.pid_output, 2)],
  ]);
}

// What the dongle's LED should be showing, by the rules in espnow_dongle/src/main.cpp.
function renderLed(s) {
  let color = "off";
  let swatch = "transparent";
  if (s.link && s.port.state === "OPEN") {
    const stale =
      s.telemetry_metrics.age_s === null || s.telemetry_metrics.age_s > 0.5;
    if (!s.link.robot_known) [color, swatch] = ["BLUE, blinking", "#2f6bff"];
    else if (stale) [color, swatch] = ["RED", "var(--red)"];
    else if (
      s.link_metrics.ack_pct_1s !== null &&
      s.link_metrics.ack_pct_1s < 90
    )
      [color, swatch] = ["AMBER", "var(--amber)"];
    else [color, swatch] = ["GREEN", "var(--green)"];
    color += s.command.sending ? ", bright" : ", dim";
  }
  fillRows("ledRows", [["Dongle LED should be", color]]);
  const v = $("ledRows").querySelector(".v");
  v.insertAdjacentHTML(
    "afterbegin",
    `<span class="ledswatch" style="background:${swatch}"></span>`,
  );
}

function renderTypes(s) {
  const head =
    "<tr><th>TYPE</th><th>NAME</th><th>FROM</th><th>COUNT</th><th>HZ</th><th>BYTES</th><th>SIZE ERR</th><th>LAST</th></tr>";
  const rows = s.types.map((t) => {
    const from = t.foreign ? `air (${t.sender})` : t.sender;
    return (
      `<tr class="${t.foreign ? "foreign" : ""}"><td>0x${t.type.toString(16).padStart(2, "0").toUpperCase()}</td>` +
      `<td>${esc(t.name)}</td><td>${esc(from)}</td><td>${t.count.toLocaleString()}</td><td>${t.hz}</td>` +
      `<td>${t.bytes.toLocaleString()}</td><td class="${t.size_errors ? "bad" : ""}">${t.size_errors}</td>` +
      `<td>${ago(t.age_s)}</td></tr>`
    );
  });
  $("types").innerHTML =
    head +
    (rows.length
      ? rows.join("")
      : '<tr><td colspan="8" class="muted">no frames yet</td></tr>');
}

function renderOta(s) {
  $("otaSection").hidden = !s.ota;
  if (!s.ota) return;
  const o = s.ota;
  fillRows("otaRows", [
    ["State", o.state_name, o.state === 4 ? "bad" : ""],
    ["Error", o.error_name, o.error ? "bad" : ""],
    ["Next offset", int(o.next_offset, "B")],
    ["Running image MD5", o.running_md5],
    ["Last status", ago(s.ota_age_s)],
  ]);
}

function renderLogs(s) {
  $("events").innerHTML = s.events.length
    ? s.events
        .map(
          (e) =>
            `<li><span class="ts">${clock(e.t)}</span><span class="lvl ${e.level}">${e.level.toUpperCase()}</span><span class="msg">${esc(e.text)}</span></li>`,
        )
        .join("")
    : '<li class="empty">No events yet.</li>';
  $("text").innerHTML = s.text_lines.length
    ? s.text_lines
        .map(
          (l) =>
            `<li class="text"><span class="ts">${clock(l.t)}</span><span class="msg">${esc(l.text)}</span></li>`,
        )
        .join("")
    : '<li class="empty">Nothing yet. The dongle prints one banner line at boot: RESET DONGLE to see it.</li>';
}

function renderCapture(s) {
  const c = s.capture;
  $("capDir").textContent = c.dir;
  $("capBtn").setAttribute("aria-pressed", c.active);
  $("capBtn").textContent = c.active ? "STOP CAPTURE" : "CAPTURE";
  fillRows("capRows", [
    [
      "Capture",
      c.active ? `${int(c.records)} records` : "OFF",
      c.active ? "warn" : "",
    ],
    ["Capture file", c.path ? c.path.split("/").pop() : NONE],
    ["Ring", `${int(c.ring_records)} records`],
  ]);
}

async function refreshFiles() {
  try {
    const files = await (await fetch("/api/captures")).json();
    $("files").innerHTML = files
      .map(
        (f) =>
          `<li><a href="/captures/${encodeURIComponent(f.name)}">${esc(f.name)}</a><span class="muted">${(f.bytes / 1e6).toFixed(2)} MB</span></li>`,
      )
      .join("");
  } catch (e) {}
}

// ---- plots -------------------------------------------------------------------------------

const PLOTS = [
  ["ack_pct", "DONGLE ACK %"],
  ["delivery_pct", "TELEMETRY DELIVERY %"],
  ["lq", "ROBOT LQ %"],
  ["tlm_hz", "TELEMETRY HZ"],
  ["gap_max_ms", "MAX TELEMETRY GAP MS"],
  ["rtt_min_ms", "COMMAND RTT MIN MS"],
  ["sent_hz", "DONGLE SENT / S"],
  ["failed_hz", "DONGLE FAILED / S"],
  ["send_cb_us", "SEND CALLBACK US"],
  ["rx_kbps", "SERIAL KBIT/S"],
];
const W = 740;
const H = 128;

$("plots").innerHTML = PLOTS.map(
  ([key, title]) =>
    `<div class="plot"><span class="head label"><span>${title}</span><span class="range mono" id="r_${key}">no data</span></span>` +
    `<div class="frame"><svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none"><line x1="0" y1="32" x2="${W}" y2="32"/>` +
    `<line x1="0" y1="64" x2="${W}" y2="64"/><line x1="0" y1="96" x2="${W}" y2="96"/><polyline id="p_${key}"/></svg></div></div>`,
).join("");

const fmt = (v) => (Math.abs(v) >= 100 ? v.toFixed(0) : v.toPrecision(3));

async function refreshSeries() {
  let data;
  try {
    data = await (await fetch("/api/series")).json();
  } catch (e) {
    return;
  }
  for (const [key] of PLOTS) {
    const values = data.series[key] || [];
    const pts = [];
    values.forEach((v, i) => {
      if (v !== null) pts.push([data.t[i], v]);
    });
    const line = $("p_" + key);
    if (!pts.length) {
      line.setAttribute("points", "");
      $("r_" + key).textContent = "no data";
      continue;
    }
    let min = Math.min(...pts.map((p) => p[1]));
    let max = Math.max(...pts.map((p) => p[1]));
    $("r_" + key).textContent = `${fmt(min)} to ${fmt(max)}`;
    if (min === max) {
      const pad = Math.abs(min) * 0.1 || 1;
      min -= pad;
      max += pad;
    }
    const x = (t) => W + (t / data.window_s) * W;
    const y = (v) => H - 8 - ((v - min) / (max - min)) * (H - 16);
    line.setAttribute(
      "points",
      pts.map(([t, v]) => `${x(t).toFixed(1)},${y(v).toFixed(1)}`).join(" "),
    );
  }
}

// ---- controls ----------------------------------------------------------------------------

let latest = null;

const post = (url, body) =>
  fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });

$("sendBtn").onclick = () =>
  post("/api/command", { on: !(latest && latest.command.sending) });

// ---- command sliders ---------------------------------------------------------------------

let sending = false;

function sliderValues() {
  return { a: Number($("sliderA").value), b: Number($("sliderB").value) };
}

function showSliders() {
  const v = sliderValues();
  $("sliderAVal").textContent = v.a.toFixed(2);
  $("sliderBVal").textContent = v.b.toFixed(2);
}

function pushValues() {
  if (sending) post("/api/command/value", sliderValues());
}

function setNeutral() {
  $("sliderA").value = 0;
  $("sliderB").value = 0;
  showSliders();
  pushValues();
}

for (const id of ["sliderA", "sliderB"]) {
  $(id).addEventListener("input", () => {
    showSliders();
    pushValues();
  });
}
$("neutralBtn").onclick = setNeutral;

// Space is neutral wherever focus is. Capture phase, and both key events, so a focused button or
// slider never also acts on it.
window.addEventListener(
  "keydown",
  (e) => {
    if (e.code !== "Space") return;
    e.preventDefault();
    setNeutral();
  },
  true,
);
window.addEventListener(
  "keyup",
  (e) => e.code === "Space" && e.preventDefault(),
  true,
);
// A hidden tab may stop running timers; drop to neutral rather than rely on the server deadman.
document.addEventListener(
  "visibilitychange",
  () => document.hidden && setNeutral(),
);

// Heartbeat: the tool returns to neutral if it hears nothing for 0.3 s.
setInterval(pushValues, 100);
$("capBtn").onclick = async () => {
  await post("/api/capture", { on: !(latest && latest.capture.active) });
  refreshFiles();
};
$("ringBtn").onclick = async () => {
  const r = await (await post("/api/save_ring")).json();
  await refreshFiles();
  location.href = "/captures/" + encodeURIComponent(r.name);
};
$("resetBtn").onclick = () => post("/api/reset_dongle");
$("clearBtn").onclick = () => post("/api/clear");

// ---- polling -----------------------------------------------------------------------------

let lastOk = 0;

async function poll() {
  try {
    const r = await fetch("/api/state");
    latest = await r.json();
    lastOk = performance.now();
  } catch (e) {}
  const live = latest && performance.now() - lastOk < 2000;
  renderHeader(latest, live);
  if (latest) {
    const s = latest;
    renderChecks(s);
    renderPort(s);
    renderDongle(s);
    renderRobotLink(s);
    renderTiming(s);
    renderCommand(s);
    renderRobotState(s);
    renderLed(s);
    renderTypes(s);
    renderOta(s);
    renderLogs(s);
    renderCapture(s);
  }
  setTimeout(poll, 200);
}

poll();
refreshFiles();
setInterval(refreshSeries, 1000);
setInterval(refreshFiles, 10000);
