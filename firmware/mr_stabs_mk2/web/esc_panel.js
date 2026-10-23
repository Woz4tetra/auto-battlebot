"use strict";

// ESC SETTINGS panel: tuning mode, backup and restore, and the side-by-side settings editor
// (docs/plans/esc_settings_dashboard_plan.md). The codec and 4-way client are in esc.js; the
// robot only relays frames. Uses $, post and flash from app.js.
(() => {
  // Left to right as seen from behind the robot. ESC numbers follow the passthrough.
  const COLUMNS = [
    { esc: 1, side: "LEFT", pin: "A3" },
    { esc: 0, side: "RIGHT", pin: "A2" },
  ];
  const SIDE = { 0: "right", 1: "left" };
  // lib/erpm_decode and esc.h assume bidirectional 3D mode.
  const DIRECTIONS_3D = [3, 4];

  let layout = null;
  let tuning = null; // last GET /esc/status
  let busy = false;
  let review = null; // {title, items: [{esc, payload, changes, warnings}]}
  // Per ESC: snap read this tuning session, edit = the form's payload, bad = key -> message.
  const escs = {
    0: { snap: null, edit: null, bad: new Map(), saved: false, note: "" },
    1: { snap: null, edit: null, bad: new Map(), saved: false, note: "" },
  };
  const rows = {}; // key -> {row, cells: {esc: {input|span}}}

  const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
  const ACRONYMS = {
    pwm: "PWM",
    hw: "HW",
    esc: "ESC",
    led: "LED",
    rpm: "RPM",
    mcu: "MCU",
    fw: "FW",
  };
  const humanize = (key) =>
    key
      .split("_")
      .map((w, i) => ACRONYMS[w] || (i === 0 ? w.charAt(0).toUpperCase() + w.slice(1) : w))
      .join(" ")
      .replace("Sbus", "SBUS")
      .replace("Sport", "S.Port");

  function download(name, doc) {
    const blob = new Blob([JSON.stringify(doc, null, 1)], {
      type: "application/json",
    });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  }

  // ---- transport ---------------------------------------------------------------------------

  async function transact(frame) {
    const sent = JSON.parse(await post("/esc/4way", { frame: Blheli.hex(frame) }));
    const deadline = performance.now() + 15000;
    while (performance.now() < deadline) {
      await sleep(30);
      const r = await (await fetch("/esc/4way", { cache: "no-store" })).json();
      if (r.id !== sent.id) throw new Error("another client replaced this frame");
      if (r.state === "done") return Blheli.unhex(r.frame);
      if (r.state === "none") throw new Error("frame dropped: tuning mode left?");
    }
    throw new Error("no response from the robot in 15 s");
  }
  const client = new Blheli.Client(transact);

  // ---- tuning mode -------------------------------------------------------------------------

  function clearSession() {
    for (const e of Object.values(escs)) {
      e.snap = e.edit = null;
      e.bad.clear();
      e.saved = false;
      e.note = "";
    }
    review = null;
  }

  function renderTuning() {
    const btn = $("escTuneBtn");
    const t = tuning;
    if (!t) {
      btn.disabled = true;
      btn.textContent = "TUNING MODE";
      $("escTuneStatus").textContent = "no answer from /esc/status";
      $("escBody").hidden = true;
      return;
    }
    const inTuning = t.state === "tuning";
    if (!inTuning && (escs[0].snap || escs[1].snap)) {
      // A new tuning session starts from fresh reads.
      clearSession();
      renderAll();
    }
    $("escBody").hidden = !inTuning;
    btn.setAttribute("aria-pressed", inTuning);
    let status = "";
    if (t.state === "combat") {
      btn.textContent = "ENTER TUNING MODE";
      btn.disabled = t.armed || t.driving || busy;
      status = t.armed ? "disarm first" : t.driving ? "ESP-NOW is driving" : "motors run normally";
    } else if (inTuning) {
      btn.textContent = "EXIT TUNING MODE";
      btn.disabled = busy;
      status = `${t.frames} frames relayed, idle ${t.idle_s} s`;
      if (t.released) status += ", ESCs released after " + t.idle_release_s + " s idle";
      if (t.guard_refusals) status += `, ${t.guard_refusals} writes refused by the guard`;
    } else {
      btn.textContent = "EXITING";
      btn.disabled = true;
      status = "ESCs waking up";
    }
    if (t.error) status += " · " + t.error;
    $("escTuneStatus").textContent = status;
    $("escArmedWarn").hidden = !(inTuning && t.armed);
  }

  async function pollTuning() {
    try {
      tuning = await (await fetch("/esc/status", { cache: "no-store" })).json();
    } catch (e) {
      tuning = null;
    }
    renderTuning();
  }

  $("escTuneBtn").onclick = () => {
    const enter = !tuning || tuning.state !== "tuning";
    post("/esc/tuning", { enter: enter ? "1" : "0" })
      .then(() => setTimeout(pollTuning, 200))
      .catch((e) => flash("escTuneStatus", e.message));
  };

  // ---- reading -----------------------------------------------------------------------------

  function setNote(esc, text) {
    escs[esc].note = text;
    renderColumn(esc);
  }

  async function run(label, fn) {
    if (busy) return;
    busy = true;
    renderAll();
    $("escBusy").textContent = label;
    try {
      await fn();
    } finally {
      busy = false;
      $("escBusy").textContent = "";
      renderAll();
      renderTuning();
    }
  }

  async function readEsc(esc) {
    const e = escs[esc];
    try {
      const snap = await client.readEsc(esc);
      e.snap = snap;
      e.edit = Uint8Array.from(snap.payload);
      e.bad.clear();
      try {
        layout.checkLayout(snap.payload);
        e.note = "read " + new Date().toLocaleTimeString();
      } catch (err) {
        e.note = err.message + ": editor read-only, backup still works";
      }
      // The first read of a session saves a backup, like `blheli32.py write` does before it
      // writes anything.
      if (!e.saved) {
        download(Blheli.backupName("esc" + esc), Blheli.backupJson(snap, layoutOrRaw()));
        e.saved = true;
      }
    } catch (err) {
      e.note = "read failed: " + err.message;
    }
  }

  // backupJson decodes the settings; a page from another layout gets an empty decode instead.
  function layoutOrRaw() {
    return {
      decode: (p) => {
        try {
          return layout.decode(p);
        } catch (err) {
          return {};
        }
      },
    };
  }

  const layoutOk = (esc) => {
    const snap = escs[esc].snap;
    if (!snap) return false;
    try {
      layout.checkLayout(snap.payload);
      return true;
    } catch (err) {
      return false;
    }
  };

  // ---- editor table ------------------------------------------------------------------------

  function hwPwmBounds(esc) {
    const e = escs[esc];
    if (!e.snap) return [16, 128];
    return [
      layout.read(layout.byKey.hw_pwm_freq_min, e.snap.payload),
      layout.read(layout.byKey.hw_pwm_freq_max, e.snap.payload),
    ];
  }

  function makeControl(f, esc) {
    let el;
    if (f.kind === "enum") {
      el = document.createElement("select");
      for (const [raw, lab] of f.options) {
        const opt = document.createElement("option");
        opt.value = raw;
        opt.textContent = lab;
        el.appendChild(opt);
      }
    } else if (f.kind === "int") {
      el = document.createElement("input");
      el.type = "number";
      el.min = f.lo;
      el.max = f.hi;
      el.step = 1;
    } else if (f.key === "name" || f.kind === "music") {
      el = document.createElement("input");
      el.autocomplete = "off";
      el.spellcheck = false;
      if (f.key === "name") el.maxLength = f.width;
      else el.placeholder = "A5 1/8 D5 1/4";
    } else {
      el = document.createElement("span");
      el.className = "mono esc-ro";
      return el;
    }
    el.className = "esc-in";
    el.onchange = () => onInput(f, esc, el);
    return el;
  }

  function buildTable() {
    const table = $("escTable");
    table.innerHTML = "";
    const head = document.createElement("div");
    head.className = "esc-row esc-head";
    head.innerHTML =
      '<div class="label">SETTING</div><div class="label"></div><div></div><div class="label"></div>';
    head.children[1].textContent = `${COLUMNS[0].side} (ESC ${COLUMNS[0].esc})`;
    head.children[3].textContent = `${COLUMNS[1].side} (ESC ${COLUMNS[1].esc})`;
    table.appendChild(head);
    for (const f of layout.fields) {
      const row = document.createElement("div");
      row.className = "esc-row";
      const editable = layout.isEditable(f) || f.key === "name" || f.kind === "music";
      if (!editable) row.classList.add("ro");
      const k = document.createElement("div");
      k.className = "esc-k";
      k.textContent = humanize(f.key);
      if (!f.verified) {
        const tag = document.createElement("span");
        tag.className = "esc-tag";
        tag.textContent = "UNVERIFIED";
        k.appendChild(tag);
      }
      k.title = f.doc || "";
      row.appendChild(k);
      const cells = {};
      const copy = document.createElement("div");
      copy.className = "esc-copy";
      for (const [i, col] of COLUMNS.entries()) {
        const cell = document.createElement("div");
        cell.className = "esc-cell";
        const control = makeControl(f, col.esc);
        cell.appendChild(control);
        cells[col.esc] = control;
        row.appendChild(cell);
        if (i === 0) row.appendChild(copy);
      }
      if (editable) {
        // → copies the left column's value to the right, ← the reverse.
        const toRight = document.createElement("button");
        toRight.className = "btn small";
        toRight.textContent = "→";
        toRight.title = "Copy LEFT to RIGHT";
        toRight.onclick = () => copyField(f, COLUMNS[0].esc, COLUMNS[1].esc);
        const toLeft = document.createElement("button");
        toLeft.className = "btn small";
        toLeft.textContent = "←";
        toLeft.title = "Copy RIGHT to LEFT";
        toLeft.onclick = () => copyField(f, COLUMNS[1].esc, COLUMNS[0].esc);
        copy.append(toRight, toLeft);
      }
      rows[f.key] = { row, cells, copy };
      table.appendChild(row);
    }
  }

  // The payload bytes a change writes, e.g. "byte 20: fe → c8". Wide fields (name, music) list
  // only the bytes that change.
  function rawChange(f, before, after) {
    const h = (b) => b.toString(16).padStart(2, "0");
    if (f.width <= 2) {
      const span = (p) => Array.from(p.subarray(f.offset, f.offset + f.width), h).join(" ");
      const where = f.width === 1 ? `byte ${f.offset}` : `bytes ${f.offset}-${f.offset + 1}`;
      return `${where}: ${span(before)} → ${span(after)}`;
    }
    const changed = [];
    for (let i = f.offset; i < f.offset + f.width; i++) {
      if (before[i] !== after[i]) changed.push(`${i}: ${h(before[i])} → ${h(after[i])}`);
    }
    const more = changed.length > 6 ? `, and ${changed.length - 6} more` : "";
    return "bytes " + changed.slice(0, 6).join(", ") + more;
  }

  function fieldText(f, payload) {
    if (f.kind === "music") return layout.decodeMusic(payload);
    if (f.kind === "text") return layout.text(f, payload).replace(/[ \xff]+$/, "");
    return layout.label(f, layout.read(f, payload));
  }

  function fillValues(esc) {
    const e = escs[esc];
    const [hwMin, hwMax] = hwPwmBounds(esc);
    for (const f of layout.fields) {
      const el = rows[f.key].cells[esc];
      const editable = layoutOk(esc) && !busy;
      if (!e.edit) {
        if (el.tagName === "SPAN") el.textContent = "--";
        else {
          el.value = "";
          el.disabled = true;
        }
        continue;
      }
      if (el.tagName === "SPAN") {
        el.textContent = fieldText(f, e.edit);
        continue;
      }
      el.disabled = !editable;
      if (e.bad.has(f.key)) continue; // keep what the operator typed
      if (f.kind === "music") el.value = layout.decodeMusic(e.edit);
      else if (f.key === "name") el.value = layout.text(f, e.edit).replace(/ +$/, "");
      else el.value = layout.read(f, e.edit);
      if (f.key === "pwm_frequency_low" || f.key === "pwm_frequency_high") {
        // Frequencies this ESC's hardware cannot run are disabled. 129 is By RPM.
        for (const opt of el.options || []) {
          const v = Number(opt.value);
          opt.disabled = v !== 129 && (v < hwMin || v > hwMax);
        }
        if (f.kind === "int") {
          el.min = hwMin;
          el.max = hwMax;
        }
      }
    }
  }

  function onInput(f, esc, el) {
    const e = escs[esc];
    if (!e.edit) return;
    try {
      if (f.kind === "music") e.edit = layout.encodeMusic(e.edit, el.value);
      else if (f.key === "name") e.edit = layout.encodeName(e.edit, el.value);
      else {
        const v = Number(el.value);
        if (f.key === "pwm_frequency_low" || f.key === "pwm_frequency_high") {
          const [hwMin, hwMax] = hwPwmBounds(esc);
          if (v !== 129 && (v < hwMin || v > hwMax)) {
            throw new Error(`this ESC runs ${hwMin}-${hwMax} kHz`);
          }
        }
        e.edit = layout.setRaw(e.edit, f.key, v);
      }
      e.bad.delete(f.key);
    } catch (err) {
      e.bad.set(f.key, err.message);
    }
    renderAll();
  }

  function copyField(f, from, to) {
    const src = escs[from];
    const dst = escs[to];
    if (!src.edit || !dst.edit || src.bad.has(f.key)) return;
    const buf = Uint8Array.from(dst.edit);
    buf.set(src.edit.subarray(f.offset, f.offset + f.width), f.offset);
    dst.edit = buf;
    dst.bad.delete(f.key);
    renderAll();
  }

  // Two separate highlights: a row whose left and right differ, and a cell edited away from what
  // its ESC holds.
  function renderHighlights() {
    const both = escs[0].edit && escs[1].edit;
    const diffOnly = $("escDiffOnly").checked;
    let differ = 0;
    for (const f of layout.fields) {
      const r = rows[f.key];
      const diff = both && !layout.sameField(f, escs[0].edit, escs[1].edit);
      if (diff) differ++;
      r.row.classList.toggle("diff", !!diff);
      r.row.hidden = !!(diffOnly && both && !diff);
      for (const btn of r.copy.children)
        btn.disabled = !both || busy || !layoutOk(0) || !layoutOk(1);
      for (const col of COLUMNS) {
        const e = escs[col.esc];
        const el = r.cells[col.esc];
        const edited = e.snap && e.edit && !layout.sameField(f, e.edit, e.snap.payload);
        el.parentElement.classList.toggle("edited", !!edited);
        el.parentElement.classList.toggle("invalid", e.bad.has(f.key));
        el.title = e.bad.get(f.key) || "";
      }
    }
    $("escDiffCount").textContent = both
      ? differ === 1
        ? "1 field differs between left and right"
        : `${differ} fields differ between left and right`
      : "read both ESCs to compare";
  }

  // ---- per-ESC columns ---------------------------------------------------------------------

  function changesFor(esc) {
    const e = escs[esc];
    if (!e.snap || !e.edit) return [];
    return layout.fields.filter((f) => !layout.sameField(f, e.edit, e.snap.payload));
  }

  function problemsFor(esc) {
    const e = escs[esc];
    if (!e.edit) return [];
    return [...e.bad.values(), ...layout.crossErrors(e.edit)];
  }

  function renderColumn(esc) {
    const e = escs[esc];
    const box = $("escCol" + esc);
    const snap = e.snap;
    const info = box.querySelector(".esc-info");
    if (snap) {
      const p = snap.payload;
      fillRows(info.id, [
        ["Firmware / layout", `${p[0]}.${p[1]} / ${p[2]}`, !layoutOk(esc)],
        ["Signature", Blheli.hex(snap.signature)],
        ["Input frames good / bad", snap.goodFrames + " / " + snap.badFrames, snap.badFrames > 0],
      ]);
    } else {
      fillRows(info.id, [["Settings", "not read this session"]]);
    }
    const changes = changesFor(esc);
    const problems = problemsFor(esc);
    box.querySelector(".esc-read").disabled = busy;
    box.querySelector(".esc-backup").disabled = busy || !snap;
    box.querySelector(".esc-restore").disabled = busy || !layoutOk(esc);
    box.querySelector(".esc-apply").disabled =
      busy || !layoutOk(esc) || changes.length === 0 || problems.length > 0;
    box.querySelector(".esc-apply").textContent = changes.length
      ? `APPLY ${changes.length} CHANGE${changes.length > 1 ? "S" : ""}`
      : "APPLY";
    const note = box.querySelector(".esc-note");
    note.textContent = problems.length ? problems.join("; ") : e.note;
    note.classList.toggle("bad", problems.length > 0);
  }

  function renderAll() {
    if (!layout) return;
    fillValues(0);
    fillValues(1);
    renderHighlights();
    renderColumn(0);
    renderColumn(1);
    renderReview();
    $("escReadBoth").disabled = busy;
    $("escApplyBoth").disabled =
      busy ||
      ![0, 1].some((esc) => changesFor(esc).length > 0) ||
      [0, 1].some(
        (esc) => problemsFor(esc).length > 0 || (changesFor(esc).length && !layoutOk(esc)),
      );
    $("escRevert").disabled = busy;
  }

  // ---- review and write --------------------------------------------------------------------

  function warningsFor(esc, payload, changes) {
    const warnings = [];
    const dir = layout.byKey.direction;
    if (changes.includes(dir)) {
      const v = layout.read(dir, payload);
      if (!DIRECTIONS_3D.includes(v)) {
        warnings.push(
          `Direction ${layout.label(dir, v)}: the robot drives these ESCs in bidirectional 3D ` +
            "mode (esc.h, lib/erpm_decode), so its throttle mapping breaks.",
        );
      }
      const other = escs[1 - esc].snap;
      if (other && layout.read(dir, other.payload) === v) {
        warnings.push(
          "Direction now matches the other ESC; on this robot they differ, so one wheel will " +
            "turn the wrong way.",
        );
      }
    }
    for (const f of changes) {
      if (!f.verified) warnings.push(`${humanize(f.key)} is not confirmed on hardware.`);
    }
    return warnings;
  }

  function openReview(title, list) {
    const items = list
      .map((esc) => {
        const changes = changesFor(esc);
        const payload = escs[esc].edit;
        return { esc, payload, changes, warnings: warningsFor(esc, payload, changes) };
      })
      .filter((item) => item.changes.length > 0);
    if (!items.length) return;
    review = { title, items };
    renderReview();
    $("escReview").scrollIntoView({ block: "nearest" });
  }

  function renderReview() {
    const box = $("escReview");
    box.hidden = !review;
    if (!review) return;
    $("escReviewTitle").textContent = review.title;
    const body = $("escReviewBody");
    body.innerHTML = "";
    for (const item of review.items) {
      const h = document.createElement("h3");
      h.className = "label";
      h.textContent = `${SIDE[item.esc].toUpperCase()} (ESC ${item.esc})`;
      body.appendChild(h);
      const held = escs[item.esc].snap.payload;
      for (const f of item.changes) {
        const row = document.createElement("div");
        row.className = "leader";
        row.innerHTML = '<span class="k"></span><span class="dots"></span><span class="v"></span>';
        row.children[0].textContent = humanize(f.key);
        row.children[2].textContent = `${fieldText(f, held)} → ${fieldText(f, item.payload)}`;
        body.appendChild(row);
        const raw = document.createElement("p");
        raw.className = "mono muted esc-raw";
        raw.textContent = rawChange(f, held, item.payload);
        body.appendChild(raw);
      }
      for (const w of item.warnings) {
        const p = document.createElement("p");
        p.className = "note esc-warn";
        p.textContent = w;
        body.appendChild(p);
      }
    }
    $("escWriteBtn").disabled = busy;
  }

  async function writeReviewed() {
    const items = review.items;
    review = null;
    for (const item of items) {
      const e = escs[item.esc];
      const held = e.snap;
      // Saved before every write, as `blheli32.py write` does.
      download(Blheli.backupName(`esc${item.esc}_before_write`), Blheli.backupJson(held, layout));
      try {
        const snap = await client.writeEsc(item.esc, layout, item.payload, held.payload);
        e.snap = snap;
        e.edit = Uint8Array.from(snap.payload);
        e.bad.clear();
        e.note = `written and verified ${new Date().toLocaleTimeString()}`;
      } catch (err) {
        e.note = "write failed: " + err.message;
        // Show what the ESC holds now; the edits stay in the form.
        try {
          const now = await client.readEsc(item.esc);
          e.snap = now;
        } catch (err2) {
          e.note += "; read-back failed too: " + err2.message;
        }
      }
    }
  }

  // ---- wiring ------------------------------------------------------------------------------

  function wireColumn(col) {
    const esc = col.esc;
    const box = $("escCol" + esc);
    box.querySelector(".esc-title").textContent = `${col.side} (ESC ${esc}, ${col.pin})`;
    box.querySelector(".esc-read").onclick = () =>
      run(`reading ${SIDE[esc]} ESC...`, () => readEsc(esc));
    box.querySelector(".esc-backup").onclick = () =>
      download(Blheli.backupName("esc" + esc), Blheli.backupJson(escs[esc].snap, layoutOrRaw()));
    const file = box.querySelector(".esc-file");
    box.querySelector(".esc-restore").onclick = () => file.click();
    file.onchange = async () => {
      const chosen = file.files[0];
      file.value = "";
      if (!chosen) return;
      try {
        const parsed = Blheli.parseBackup(JSON.parse(await chosen.text()), layout);
        if (parsed.esc !== esc) {
          throw new Error(
            `${chosen.name} was read from ESC ${parsed.esc}, not ESC ${esc}. Direction ` +
              "differs per ESC: restore each ESC from its own backup.",
          );
        }
        escs[esc].edit = parsed.payload;
        escs[esc].bad.clear();
        renderAll();
        openReview(`Restore ${chosen.name}`, [esc]);
      } catch (err) {
        setNote(esc, "restore refused: " + err.message);
      }
    };
    box.querySelector(".esc-apply").onclick = () => openReview(`Write ${SIDE[esc]} ESC`, [esc]);
  }

  $("escReadBoth").onclick = () =>
    run("reading both ESCs...", async () => {
      await readEsc(COLUMNS[0].esc);
      await readEsc(COLUMNS[1].esc);
    });
  $("escApplyBoth").onclick = () => openReview("Write both ESCs", [1, 0]);
  $("escRevert").onclick = () => {
    for (const e of Object.values(escs)) {
      if (e.snap) e.edit = Uint8Array.from(e.snap.payload);
      e.bad.clear();
    }
    review = null;
    renderAll();
  };
  $("escDiffOnly").onchange = renderHighlights;
  $("escCancelBtn").onclick = () => {
    review = null;
    renderReview();
  };
  $("escWriteBtn").onclick = () => run("writing...", writeReviewed);

  fetch("/esc_layout.json")
    .then((r) => r.json())
    .then((table) => {
      layout = Blheli.makeLayout(table);
      buildTable();
      COLUMNS.forEach(wireColumn);
      renderAll();
    })
    .catch((e) => ($("escTuneStatus").textContent = "layout table failed to load: " + e.message));

  setInterval(pollTuning, 1000);
  pollTuning();
})();
