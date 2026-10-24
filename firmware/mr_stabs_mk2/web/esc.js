"use strict";

// BLHeli_32 settings codec and 4-way client for the ESC panel (esc_panel.js). A port of
// auto_battlebot/blheli32/ (fourway.py, cipher.py, layout.py); firmware/blheli/README.md is the
// spec, and esc_layout.json is the field table generated from layout.py. No DOM here, so
// esc.test.mjs runs it under node.
const Blheli = (() => {
  // ---- 4-way framing ------------------------------------------------------------------------

  const CMD = {
    DEVICE_RESET: 0x35,
    DEVICE_INIT_FLASH: 0x37,
    DEVICE_READ: 0x3a,
    DEVICE_WRITE: 0x3b,
  };
  const PAGE_ADDRESS = 0xf800;
  const PAGE_SIZE = 256;
  const STATS_ADDRESS = 0xfa00;
  const INFO_ADDRESS = 0xfd90;

  class FourWayError extends Error {}
  class LayoutError extends Error {}
  class VerifyError extends Error {}

  function crc16xmodem(bytes, length = bytes.length) {
    let crc = 0;
    for (let i = 0; i < length; i++) {
      crc ^= bytes[i] << 8;
      for (let bit = 0; bit < 8; bit++) {
        crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
      }
    }
    return crc;
  }

  function buildRequest(cmd, address, params) {
    if (params.length < 1 || params.length > 256) {
      throw new FourWayError(`4-way params must be 1..256 bytes, got ${params.length}`);
    }
    const frame = new Uint8Array(5 + params.length + 2);
    frame.set([0x2f, cmd, address >> 8, address & 0xff, params.length & 0xff]);
    frame.set(params, 5);
    const crc = crc16xmodem(frame, 5 + params.length);
    frame[5 + params.length] = crc >> 8;
    frame[6 + params.length] = crc & 0xff;
    return frame;
  }

  function parseResponse(frame) {
    if (frame.length < 8 || frame[0] !== 0x2e) {
      throw new FourWayError("not a 4-way response: " + hex(frame.slice(0, 8)));
    }
    const n = frame[4] || 256;
    if (frame.length !== 5 + n + 3) {
      throw new FourWayError(`response length ${frame.length} does not match n=${n}`);
    }
    const crc = (frame[frame.length - 2] << 8) | frame[frame.length - 1];
    if (crc16xmodem(frame, frame.length - 2) !== crc) {
      throw new FourWayError("response CRC mismatch");
    }
    return {
      cmd: frame[1],
      address: (frame[2] << 8) | frame[3],
      params: frame.slice(5, 5 + n),
      ack: frame[5 + n],
    };
  }

  const hex = (bytes) => Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join("");

  function unhex(text) {
    if (text.length % 2 || /[^0-9a-f]/i.test(text)) throw new Error("not hex: " + text);
    const out = new Uint8Array(text.length / 2);
    for (let i = 0; i < out.length; i++) out[i] = parseInt(text.substr(2 * i, 2), 16);
    return out;
  }

  const toBase64 = (bytes) => btoa(String.fromCharCode(...bytes));
  const fromBase64 = (text) => Uint8Array.from(atob(text), (c) => c.charCodeAt(0));

  // ---- page encryption (cipher.py) ----------------------------------------------------------

  // XTEA, 32 rounds, with a tweak added into each round key term: 0xF800 + 8 per block. Each
  // decrypted block is a 2-byte nonce and 6 payload bytes.
  const KEY = [0x81fbc617, 0x9e81c901, 0x29a1fa54, 0x318234b4];
  const DELTA = 0x9e3779b9;
  const TWEAK = 0xf800;
  const PAYLOAD_SIZE = 192;

  const mix = (v) => ((((v << 4) ^ (v >>> 5)) >>> 0) + v) >>> 0;
  const u32 = (b, i) => (b[i] | (b[i + 1] << 8) | (b[i + 2] << 16) | (b[i + 3] << 24)) >>> 0;
  function putU32(b, i, v) {
    b[i] = v & 0xff;
    b[i + 1] = (v >>> 8) & 0xff;
    b[i + 2] = (v >>> 16) & 0xff;
    b[i + 3] = (v >>> 24) & 0xff;
  }

  function decryptBlock(block, tweak) {
    let v0 = u32(block, 0);
    let v1 = u32(block, 4);
    let s = Math.imul(32, DELTA) >>> 0;
    for (let i = 0; i < 32; i++) {
      v1 = (v1 - ((mix(v0) ^ ((s + KEY[(s >>> 11) & 3] + tweak) >>> 0)) >>> 0)) >>> 0;
      s = (s - DELTA) >>> 0;
      v0 = (v0 - ((mix(v1) ^ ((s + KEY[s & 3] + tweak) >>> 0)) >>> 0)) >>> 0;
    }
    const out = new Uint8Array(8);
    putU32(out, 0, v0);
    putU32(out, 4, v1);
    return out;
  }

  function encryptBlock(block, tweak) {
    let v0 = u32(block, 0);
    let v1 = u32(block, 4);
    let s = 0;
    for (let i = 0; i < 32; i++) {
      v0 = (v0 + ((mix(v1) ^ ((s + KEY[s & 3] + tweak) >>> 0)) >>> 0)) >>> 0;
      s = (s + DELTA) >>> 0;
      v1 = (v1 + ((mix(v0) ^ ((s + KEY[(s >>> 11) & 3] + tweak) >>> 0)) >>> 0)) >>> 0;
    }
    const out = new Uint8Array(8);
    putU32(out, 0, v0);
    putU32(out, 4, v1);
    return out;
  }

  function decodePage(page) {
    if (page.length !== PAGE_SIZE) {
      throw new LayoutError(`page is ${page.length} bytes, expected ${PAGE_SIZE}`);
    }
    const payload = new Uint8Array(PAYLOAD_SIZE);
    for (let n = 0; n < 32; n++) {
      payload.set(decryptBlock(page.subarray(8 * n, 8 * n + 8), TWEAK + 8 * n).subarray(2), 6 * n);
    }
    return payload;
  }

  function pageNonces(page) {
    const out = [];
    for (let n = 0; n < 32; n++) {
      out.push(decryptBlock(page.subarray(8 * n, 8 * n + 8), TWEAK + 8 * n).slice(0, 2));
    }
    return out;
  }

  // Random nonces per block, as BLHeliSuite's Write Setup sends.
  const randomNonce = () => crypto.getRandomValues(new Uint8Array(2));

  function encodePage(payload, nonce = randomNonce) {
    if (payload.length !== PAYLOAD_SIZE) {
      throw new LayoutError(`payload is ${payload.length} bytes, expected ${PAYLOAD_SIZE}`);
    }
    const page = new Uint8Array(PAGE_SIZE);
    for (let n = 0; n < 32; n++) {
      const block = new Uint8Array(8);
      block.set(nonce(n));
      block.set(payload.subarray(6 * n, 6 * n + 6), 2);
      page.set(encryptBlock(block, TWEAK + 8 * n), 8 * n);
    }
    return page;
  }

  // ---- settings record (layout.py) ----------------------------------------------------------

  const NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"];
  const LENGTHS = { 8: 0, 4: 1, 2: 2, 1: 3 };

  function makeLayout(table) {
    const fields = table.fields.map((f) => ({ ...f, options: new Map(f.options) }));
    const byKey = Object.fromEntries(fields.map((f) => [f.key, f]));
    const music = byKey.music_notes;
    const nameField = byKey.name;

    const isEditable = (f) => f.kind === "int" || f.kind === "enum";

    function read(f, payload) {
      let v = 0;
      for (let i = f.width - 1; i >= 0; i--) v = v * 256 + payload[f.offset + i];
      return v;
    }

    function label(f, raw) {
      if (f.options.has(raw)) return f.options.get(raw);
      if (f.off_value !== null && raw === f.off_value) return "Off";
      return raw + (f.unit ? " " + f.unit : "");
    }

    function sameField(f, a, b) {
      for (let i = f.offset; i < f.offset + f.width; i++) if (a[i] !== b[i]) return false;
      return true;
    }

    function checkLayout(payload) {
      if (payload[0] !== table.fw_main || payload[2] !== table.layout_revision) {
        throw new LayoutError(
          `firmware ${payload[0]}.${payload[1]} layout ${payload[2]}: this table is for ` +
            `${table.fw_main}.x layout ${table.layout_revision}`,
        );
      }
    }

    function text(f, payload) {
      return String.fromCharCode(...payload.subarray(f.offset, f.offset + f.width));
    }

    function decodeMusic(payload) {
      const notes = [];
      for (const b of payload.subarray(music.offset, music.offset + music.width)) {
        if (b === 0xff) break;
        const pitch = b & 0x3f;
        const octave = 4 + (pitch >> 4);
        const semitone = pitch % 16;
        const name = semitone < 12 ? NOTE_NAMES[semitone] : "?" + semitone;
        notes.push(`${name}${octave} 1/${[8, 4, 2, 1][b >> 6]}`);
      }
      return notes.join(" ");
    }

    function encodeMusic(payload, script) {
      const tokens = script.trim() ? script.trim().split(/\s+/) : [];
      if (tokens.length % 2) {
        throw new LayoutError("music needs NOTE+OCTAVE and 1/N pairs, e.g. 'A5 1/8'");
      }
      const out = [];
      for (let i = 0; i < tokens.length; i += 2) {
        const note = tokens[i];
        const dur = tokens[i + 1];
        const name = note.slice(0, -1).toUpperCase();
        const octave = Number(note.slice(-1));
        if (!NOTE_NAMES.includes(name) || !/^[4-7]$/.test(note.slice(-1))) {
          throw new LayoutError(`bad note ${note}: use C..B with optional #, octave 4-7`);
        }
        const m = /^1\/(\d+)$/.exec(dur);
        if (!m || !(m[1] in LENGTHS)) {
          throw new LayoutError(`bad length ${dur}: use 1/1, 1/2, 1/4 or 1/8`);
        }
        out.push((LENGTHS[m[1]] << 6) | ((octave - 4) * 16 + NOTE_NAMES.indexOf(name)));
      }
      if (out.length > music.width) {
        throw new LayoutError(`music has ${out.length} notes, the ESC holds ${music.width}`);
      }
      checkLayout(payload);
      const buf = Uint8Array.from(payload);
      buf.fill(0xff, music.offset, music.offset + music.width);
      buf.set(out, music.offset);
      return buf;
    }

    function encodeName(payload, name) {
      if (name.length > nameField.width || /[^\x20-\x7e]/.test(name)) {
        throw new LayoutError(`name must be up to ${nameField.width} printable ASCII characters`);
      }
      checkLayout(payload);
      const buf = Uint8Array.from(payload);
      const padded = name.padEnd(nameField.width, " ");
      for (let i = 0; i < nameField.width; i++) buf[nameField.offset + i] = padded.charCodeAt(i);
      return buf;
    }

    // Range and option membership for one raw value, or an error message.
    function rangeError(f, value) {
      if (!Number.isInteger(value) || value < f.lo || value > f.hi) {
        return `${f.key}=${value} outside ${f.lo}..${f.hi}`;
      }
      if (f.kind === "enum" && f.options.size && !f.options.has(value)) {
        return `${f.key}=${value} is not one of its options`;
      }
      return null;
    }

    // Rules across fields, the same ones layout.encode enforces.
    function crossErrors(payload) {
      const errors = [];
      const min = read(byKey.min_throttle, payload);
      const center = read(byKey.center_throttle, payload);
      const max = read(byKey.max_throttle, payload);
      if (!(min < center && center < max)) {
        errors.push(`throttle needs min < center < max, got ${min} / ${center} / ${max}`);
      }
      if (read(byKey.pwm_frequency_low, payload) > read(byKey.pwm_frequency_high, payload)) {
        errors.push("PWM frequency low must not exceed PWM frequency high");
      }
      return errors;
    }

    function setRaw(payload, key, value) {
      const f = byKey[key];
      if (!f) throw new LayoutError(`unknown field ${key}`);
      if (!isEditable(f)) throw new LayoutError(`${key} is ${f.kind}, not editable`);
      const error = rangeError(f, value);
      if (error) throw new LayoutError(error);
      const buf = Uint8Array.from(payload);
      for (let i = 0; i < f.width; i++) buf[f.offset + i] = (value >> (8 * i)) & 0xff;
      return buf;
    }

    function encode(payload, changes) {
      checkLayout(payload);
      let buf = payload;
      for (const [key, value] of Object.entries(changes)) buf = setRaw(buf, key, value);
      const errors = crossErrors(buf);
      if (errors.length) throw new LayoutError(errors.join("; "));
      return Uint8Array.from(buf);
    }

    function decode(payload) {
      checkLayout(payload);
      const out = {};
      for (const f of fields) {
        if (f.kind === "text") out[f.key] = text(f, payload);
        else if (f.kind === "music") out[f.key] = decodeMusic(payload);
        else out[f.key] = read(f, payload);
      }
      return out;
    }

    return {
      fields,
      byKey,
      isEditable,
      read,
      label,
      sameField,
      checkLayout,
      text,
      decodeMusic,
      encodeMusic,
      encodeName,
      rangeError,
      crossErrors,
      setRaw,
      encode,
      decode,
    };
  }

  // ---- ESC snapshots and backups (esc.py, blheli32.py) --------------------------------------

  function snapshot(index, signature, page, stats, info) {
    const u32le = (b, i) => u32(b, i);
    return {
      index,
      signature,
      page,
      payload: decodePage(page),
      stats,
      info,
      inputProtocol: stats[3],
      goodFrames: u32le(stats, 4),
      badFrames: u32le(stats, 8),
    };
  }

  function localStamp(date, sep) {
    const p = (n) => String(n).padStart(2, "0");
    return (
      `${date.getFullYear()}-${p(date.getMonth() + 1)}-${p(date.getDate())}${sep}` +
      `${p(date.getHours())}${sep === "T" ? ":" : ""}${p(date.getMinutes())}` +
      `${sep === "T" ? ":" : ""}${p(date.getSeconds())}`
    );
  }

  // The document `blheli32.py backup` writes, so the page and the CLI read each other's files.
  function backupJson(snap, layout, date = new Date()) {
    return {
      esc: snap.index,
      read_at: localStamp(date, "T"),
      signature: hex(snap.signature),
      page_b64: toBase64(snap.page),
      payload_hex: hex(snap.payload),
      stats_hex: hex(snap.stats),
      info_hex: hex(snap.info),
      good_frames: snap.goodFrames,
      bad_frames: snap.badFrames,
      settings: layout.decode(snap.payload),
    };
  }

  function backupName(prefix, date = new Date()) {
    return `${prefix}_${localStamp(date, "_")}.json`;
  }

  // A backup document -> {esc, payload}, with blheli32.py's consistency checks.
  function parseBackup(doc, layout) {
    if (typeof doc !== "object" || doc === null || typeof doc.payload_hex !== "string") {
      throw new LayoutError("not a blheli32.py backup: no payload_hex");
    }
    const payload = unhex(doc.payload_hex);
    if (payload.length !== PAYLOAD_SIZE) {
      throw new LayoutError(`payload_hex is ${payload.length} bytes, expected ${PAYLOAD_SIZE}`);
    }
    if (typeof doc.page_b64 === "string") {
      const decoded = decodePage(fromBase64(doc.page_b64));
      if (hex(decoded) !== hex(payload)) {
        throw new LayoutError("backup is inconsistent: page_b64 does not decode to payload_hex");
      }
    }
    layout.checkLayout(payload);
    return { esc: doc.esc, payload };
  }

  // ---- zip archive, entries stored uncompressed ---------------------------------------------

  const CRC_TABLE = Uint32Array.from({ length: 256 }, (_, n) => {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    return c >>> 0;
  });

  function crc32(bytes) {
    let c = 0xffffffff;
    for (const b of bytes) c = CRC_TABLE[(c ^ b) & 0xff] ^ (c >>> 8);
    return (c ^ 0xffffffff) >>> 0;
  }

  // [[value, bytes], ...] -> little-endian Uint8Array, for the zip headers.
  function packLe(fields) {
    const out = new Uint8Array(fields.reduce((n, [, size]) => n + size, 0));
    let i = 0;
    for (const [value, size] of fields) {
      for (let k = 0; k < size; k++) out[i++] = (value >>> (8 * k)) & 0xff;
    }
    return out;
  }

  // [{name, data: Uint8Array}] -> one .zip, so a page can hand over many backups as one download.
  function zipStore(files, date = new Date()) {
    const time = (date.getHours() << 11) | (date.getMinutes() << 5) | (date.getSeconds() >> 1);
    const day = ((date.getFullYear() - 1980) << 9) | ((date.getMonth() + 1) << 5) | date.getDate();
    const parts = [];
    const central = [];
    let offset = 0;
    for (const f of files) {
      const name = new TextEncoder().encode(f.name);
      const crc = crc32(f.data);
      const n = f.data.length;
      // version 2.0, flag 0x0800 = UTF-8 name, method 0 = stored
      const common = [
        [20, 2],
        [0x0800, 2],
        [0, 2],
        [time, 2],
        [day, 2],
        [crc, 4],
        [n, 4],
        [n, 4],
      ];
      parts.push(packLe([[0x04034b50, 4], ...common, [name.length, 2], [0, 2]]), name, f.data);
      central.push(
        packLe([
          [0x02014b50, 4],
          [20, 2],
          ...common,
          [name.length, 2],
          [0, 2],
          [0, 2],
          [0, 2],
          [0, 2],
          [0, 4],
          [offset, 4],
        ]),
        name,
      );
      offset += 30 + name.length + n;
    }
    const centralSize = central.reduce((n, b) => n + b.length, 0);
    const end = packLe([
      [0x06054b50, 4],
      [0, 2],
      [0, 2],
      [files.length, 2],
      [files.length, 2],
      [centralSize, 4],
      [offset, 4],
      [0, 2],
    ]);
    const all = [...parts, ...central, end];
    const out = new Uint8Array(all.reduce((n, b) => n + b.length, 0));
    let i = 0;
    for (const b of all) {
      out.set(b, i);
      i += b.length;
    }
    return out;
  }

  // ---- 4-way client (fourway.py, esc.py) ----------------------------------------------------

  // transact(frame: Uint8Array) -> Promise<Uint8Array>: one request frame out, its response in.
  class Client {
    constructor(transact) {
      this.transact = transact;
    }

    async command(cmd, address = 0, params = [0]) {
      const resp = parseResponse(await this.transact(buildRequest(cmd, address, params)));
      if (resp.cmd !== cmd) {
        throw new FourWayError(
          `sent 0x${cmd.toString(16)}, got a reply to 0x${resp.cmd.toString(16)}`,
        );
      }
      if (resp.ack !== 0) {
        const at = address.toString(16).toUpperCase().padStart(4, "0");
        throw new FourWayError(
          `command 0x${cmd.toString(16).toUpperCase()} at 0x${at} failed, ack 0x${resp.ack.toString(16).padStart(2, "0")}`,
        );
      }
      return resp;
    }

    async initEsc(esc) {
      await this.command(CMD.DEVICE_RESET, 0, [esc]);
      return (await this.command(CMD.DEVICE_INIT_FLASH, 0, [esc])).params;
    }

    async releaseEsc(esc) {
      await this.command(CMD.DEVICE_RESET, 0, [esc]);
    }

    async read(address, length) {
      const data = (await this.command(CMD.DEVICE_READ, address, [length & 0xff])).params;
      if (data.length !== length) {
        throw new FourWayError(
          `read 0x${address.toString(16)}: asked ${length}, got ${data.length}`,
        );
      }
      return data;
    }

    // BLHeliSuite's Read Setup for one ESC.
    async readEsc(esc) {
      const signature = await this.initEsc(esc);
      let page;
      let stats;
      let info;
      try {
        page = await this.read(PAGE_ADDRESS, PAGE_SIZE);
        stats = await this.read(STATS_ADDRESS, 16);
        info = await this.read(INFO_ADDRESS, 16);
      } finally {
        await this.releaseEsc(esc);
      }
      return snapshot(esc, signature, page, stats, info);
    }

    // Session.write: check the ESC still holds expectCurrent, one 256-byte write at 0xF800,
    // read back and compare. Returns a fresh snapshot.
    async writeEsc(esc, layout, payload, expectCurrent) {
      layout.checkLayout(payload);
      const page = encodePage(payload);
      await this.initEsc(esc);
      let after;
      try {
        const current = decodePage(await this.read(PAGE_ADDRESS, PAGE_SIZE));
        layout.checkLayout(current);
        if (expectCurrent && hex(current) !== hex(expectCurrent)) {
          throw new VerifyError(`ESC ${esc} no longer matches what this page read: read it again`);
        }
        if (hex(current) !== hex(payload)) {
          await this.command(CMD.DEVICE_WRITE, PAGE_ADDRESS, page);
        }
        after = decodePage(await this.read(PAGE_ADDRESS, PAGE_SIZE));
      } finally {
        await this.releaseEsc(esc);
      }
      if (hex(after) !== hex(payload)) {
        const changed = [];
        for (let i = 0; i < payload.length; i++) if (after[i] !== payload[i]) changed.push(i);
        throw new VerifyError(
          `ESC ${esc}: read-back differs at payload bytes ${changed.join(", ")}`,
        );
      }
      return this.readEsc(esc);
    }
  }

  return {
    CMD,
    PAGE_ADDRESS,
    FourWayError,
    LayoutError,
    VerifyError,
    crc16xmodem,
    buildRequest,
    parseResponse,
    hex,
    unhex,
    toBase64,
    fromBase64,
    decodePage,
    pageNonces,
    encodePage,
    makeLayout,
    snapshot,
    backupJson,
    backupName,
    parseBackup,
    crc32,
    zipStore,
    Client,
  };
})();

if (typeof module !== "undefined") module.exports = Blheli;
