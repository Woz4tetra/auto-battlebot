// esc.js against the pages read from Mr Stabs Mk2's ESCs. Run with `node --test` (pytest runs it
// too, from tests/python/test_blheli32.py). The expected values are the ones the Python tests
// check, so the page and the CLI decode the same.
import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { createRequire } from "node:module";
import path from "node:path";
import { test } from "node:test";
import { fileURLToPath } from "node:url";

const require = createRequire(import.meta.url);
const Blheli = require("./esc.js");

const here = path.dirname(fileURLToPath(import.meta.url));
const table = JSON.parse(readFileSync(path.join(here, "esc_layout.json"), "utf8"));
const layout = Blheli.makeLayout(table);
const capture = (esc, addr = "f800") =>
  new Uint8Array(
    readFileSync(
      path.join(here, "../../blheli/captures", `mr_stabs_mk2_esc${esc}_${addr}_2026-10-23.bin`),
    ),
  );
// Reads a zip on stdin with Python's zipfile (which checks every CRC) and prints each entry.
const PY_UNZIP = `
import io, sys, zipfile
z = zipfile.ZipFile(io.BytesIO(sys.stdin.buffer.read()))
assert z.testzip() is None
for i in z.infolist():
    print(i.filename, "%04d-%02d-%02dT%02d:%02d:%02d" % i.date_time, z.read(i).decode())
`;
const diff = (a, b) => [...a.keys()].filter((i) => a[i] !== b[i]);

// What BLHeliSuite32xl showed for ESC 1 after Read Setup on 2026-10-23.
const BLHELISUITE_ESC1 = {
  direction: 3,
  rampup_power: 100,
  pwm_frequency_low: 24,
  comm_timing: 16,
  demag_compensation: 3,
  min_throttle: 1000,
  center_throttle: 1500,
  max_throttle: 2000,
  throttle_cal_enable: 1,
  temperature_protection: 140,
  low_voltage_protection: 0,
  low_rpm_power_protect: 0,
  brake_on_stop: 100,
  startup_beep_volume: 254,
  beacon_volume: 80,
  beacon_delay: 600,
  max_acceleration: 0,
  nondamped_mode: 0,
  sine_mode: 0,
  auto_telemetry: 1,
  stall_protection: 0,
  sbus_channel: 17,
  sport_physical_id: 0,
  pwm_frequency_high: 24,
};

test("decode matches BLHeliSuite", () => {
  const settings = layout.decode(Blheli.decodePage(capture(1)));
  for (const [key, value] of Object.entries(BLHELISUITE_ESC1)) {
    assert.equal(settings[key], value, key);
  }
  assert.ok(settings.esc_layout.startsWith("#Flycolor_Raptor_5#"));
  assert.ok(settings.esc_mcu.startsWith("#BLHeli_32*STM32G071x6#"));
});

test("page round trips with its own nonces", () => {
  for (const esc of [1, 2]) {
    const page = capture(esc);
    const nonces = Blheli.pageNonces(page);
    const again = Blheli.encodePage(Blheli.decodePage(page), (n) => nonces[n]);
    assert.deepEqual(again, page);
  }
});

test("fresh nonces change the ciphertext, not the payload", () => {
  const payload = Blheli.decodePage(capture(1));
  const a = Blheli.encodePage(payload);
  const b = Blheli.encodePage(payload);
  assert.notDeepEqual(a, b);
  assert.deepEqual(Blheli.decodePage(a), payload);
});

test("the ESCs differ in direction and one music byte", () => {
  const a = Blheli.decodePage(capture(1));
  const b = Blheli.decodePage(capture(2));
  assert.deepEqual(diff(a, b), [3, 144]);
  assert.deepEqual([a[3], b[3]], [3, 4]);
});

test("encode changes only the field bytes", () => {
  const before = Blheli.decodePage(capture(1));
  const after = layout.encode(before, { demag_compensation: 4, beacon_delay: 1200 });
  assert.deepEqual(diff(before, after), [7, 22, 23]);
  assert.equal(layout.decode(after).beacon_delay, 1200);
  // BLHeliSuite's own Write Setup with PWM low 16 / high 48 changed bytes 5 and 34 only.
  const pwm = layout.encode(before, { pwm_frequency_low: 16, pwm_frequency_high: 48 });
  assert.deepEqual(diff(before, pwm), [5, 34]);
});

test("encode rejects what layout.py rejects", () => {
  const payload = Blheli.decodePage(capture(1));
  for (const changes of [
    { rampup_power: 2 },
    { demag_compensation: 0 },
    { min_throttle: 1600 },
    { pwm_frequency_low: 40 },
    { fw_main_revision: 33 },
    { esc_layout: 1 },
    { led_control: 1 },
  ]) {
    assert.throws(
      () => layout.encode(payload, changes),
      Blheli.LayoutError,
      JSON.stringify(changes),
    );
  }
  const wrong = Uint8Array.from(payload);
  wrong[2] = 47;
  assert.throws(() => layout.decode(wrong), Blheli.LayoutError);
});

test("every editable field round trips at its limits and options", () => {
  const payload = Blheli.decodePage(capture(1));
  for (const f of layout.fields.filter(layout.isEditable)) {
    const values = f.options.size ? [...f.options.keys()] : [f.lo, f.hi];
    for (const v of values) {
      const buf = layout.setRaw(payload, f.key, v);
      assert.equal(layout.read(f, buf), v, `${f.key}=${v}`);
      assert.deepEqual(
        diff(payload, buf).filter((i) => i < f.offset || i >= f.offset + f.width),
        [],
      );
    }
  }
});

test("labels follow layout.py", () => {
  const f = layout.byKey;
  assert.equal(layout.label(f.demag_compensation, 4), "Very High");
  assert.equal(layout.label(f.pwm_frequency_high, 129), "By RPM");
  assert.equal(layout.label(f.brake_on_stop, 0), "Off");
  assert.equal(layout.label(f.rampup_power, 60), "60 %");
});

test("music matches BLHeliSuite's encoding", () => {
  const payload = Blheli.decodePage(capture(1));
  assert.equal(layout.decodeMusic(payload), "A5 1/8");
  assert.equal(layout.decodeMusic(Blheli.decodePage(capture(2))), "D5 1/8");
  const script = "C5 1/8 C#5 1/4 D6 1/2 C4 1/2 B5 1/1 A5 1/8";
  const music = layout.encodeMusic(payload, script);
  assert.equal(Blheli.hex(music.subarray(144, 151)), "1051a280db19ff");
  assert.equal(layout.decodeMusic(music), script);
  assert.throws(() => layout.encodeMusic(payload, "C4 1/16"), Blheli.LayoutError);
});

test("name is space padded", () => {
  const payload = layout.encodeName(Blheli.decodePage(capture(1)), "LEFT");
  assert.equal(String.fromCharCode(...payload.subarray(128, 144)), "LEFT".padEnd(16));
  assert.throws(() => layout.encodeName(payload, "x".repeat(17)), Blheli.LayoutError);
});

test("request framing matches fourway.py", () => {
  // build_request(0x3A, 0xF800, b"\x00")
  assert.equal(Blheli.hex(Blheli.buildRequest(0x3a, 0xf800, [0])), "2f3af800010093f4");
  assert.equal(Blheli.crc16xmodem(new TextEncoder().encode("123456789")), 0x31c3);
});

// Answers 4-way frames like the robot's relay and an ESC, from the captured ESC 1 pages. Every
// read re-encrypts with fresh nonces, as the ESC does.
class FakeEsc {
  constructor({ dropWrites = false } = {}) {
    this.settings = Blheli.decodePage(capture(1));
    this.memory = { 0xfa00: capture(1, "fa00"), 0xfd90: capture(1, "fd90") };
    this.writes = [];
    this.dropWrites = dropWrites;
  }

  respond(cmd, address, params, ack = 0) {
    const frame = new Uint8Array(5 + params.length + 3);
    frame.set([0x2e, cmd, address >> 8, address & 0xff, params.length & 0xff]);
    frame.set(params, 5);
    frame[5 + params.length] = ack;
    const crc = Blheli.crc16xmodem(frame, 6 + params.length);
    frame[6 + params.length] = crc >> 8;
    frame[7 + params.length] = crc & 0xff;
    return frame;
  }

  async transact(frame) {
    const cmd = frame[1];
    const address = (frame[2] << 8) | frame[3];
    const params = frame.slice(5, 5 + (frame[4] || 256));
    if (cmd === 0x3a && address === 0xf800) {
      return this.respond(cmd, address, Blheli.encodePage(this.settings));
    }
    if (cmd === 0x3a) return this.respond(cmd, address, this.memory[address].slice(0, params[0]));
    if (cmd === 0x3b) {
      this.writes.push([address, params.length]);
      if (!this.dropWrites) this.settings = Blheli.decodePage(params);
      return this.respond(cmd, address, [0]);
    }
    if (cmd === 0x37) return this.respond(cmd, 0, [0x06, 0x47, 0x6d, 0x04]);
    return this.respond(cmd, address, [0]);
  }
}

test("client reads an ESC", async () => {
  const esc = new FakeEsc();
  const snap = await new Blheli.Client((f) => esc.transact(f)).readEsc(0);
  assert.equal(snap.goodFrames, 338422);
  assert.equal(snap.inputProtocol, 5);
  assert.equal(Blheli.hex(snap.signature), "06476d04");
  assert.equal(layout.decode(snap.payload).rampup_power, 100);
});

test("client write is one page and verifies", async () => {
  const esc = new FakeEsc();
  const client = new Blheli.Client((f) => esc.transact(f));
  const before = (await client.readEsc(0)).payload;
  const edited = layout.encode(before, { startup_beep_volume: 200 });
  const after = await client.writeEsc(0, layout, edited, before);
  assert.deepEqual(after.payload, edited);
  assert.deepEqual(esc.writes, [[0xf800, 256]]);
});

test("client catches a dropped write and a stale read", async () => {
  const dropped = new FakeEsc({ dropWrites: true });
  const client = new Blheli.Client((f) => dropped.transact(f));
  const before = (await client.readEsc(0)).payload;
  await assert.rejects(
    client.writeEsc(0, layout, layout.encode(before, { pwm_frequency_low: 16 }), before),
    /payload bytes 5$/,
  );

  const esc = new FakeEsc();
  const fresh = new Blheli.Client((f) => esc.transact(f));
  const stale = layout.encode(before, { rampup_power: 50 });
  await assert.rejects(
    fresh.writeEsc(0, layout, layout.encode(before, { brake_on_stop: 0 }), stale),
    Blheli.VerifyError,
  );
  assert.deepEqual(esc.writes, []);
});

test("backups match blheli32.py's format both ways", () => {
  const snap = Blheli.snapshot(
    0,
    Blheli.unhex("06476d04"),
    capture(1),
    capture(1, "fa00"),
    capture(1, "fd90"),
  );
  const doc = Blheli.backupJson(snap, layout);
  assert.equal(doc.good_frames, 338422);
  const parsed = Blheli.parseBackup(JSON.parse(JSON.stringify(doc)), layout);
  assert.equal(parsed.esc, 0);
  assert.deepEqual(parsed.payload, snap.payload);
  assert.throws(
    () =>
      Blheli.parseBackup(
        { ...doc, payload_hex: Blheli.hex(layout.setRaw(snap.payload, "rampup_power", 50)) },
        layout,
      ),
    /inconsistent/,
  );
});

test("zipStore writes an archive unzip reads back", () => {
  assert.equal(Blheli.crc32(new TextEncoder().encode("123456789")), 0xcbf43926);
  const files = [
    { name: "esc1_2026-10-24_101500.json", data: new TextEncoder().encode('{"esc": 1}') },
    { name: "esc0_2026-10-24_101501.json", data: new Uint8Array(0) },
  ];
  const zip = Blheli.zipStore(files, new Date(2026, 9, 24, 10, 15, 0));
  const listing = execFileSync("python3", ["-c", PY_UNZIP], { input: zip }).toString();
  assert.equal(
    listing,
    'esc1_2026-10-24_101500.json 2026-10-24T10:15:00 {"esc": 1}\nesc0_2026-10-24_101501.json 2026-10-24T10:15:00 \n',
  );
});
