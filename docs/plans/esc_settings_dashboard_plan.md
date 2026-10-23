# Mr Stabs Mk2 ESC settings from the WiFi dashboard

## Goal

Read and write every BLHeli_32 setting on both drive ESCs from the robot's diagnostics page, so I
can tune them for performance without a USB cable or BLHeliSuite32. A **TUNING MODE** button on the
page stops the motors, hands the ESC signal wires to the BLHeli bootloader protocol, and exposes
two panels:

- **Backup / restore** (option B): read each ESC's settings, download them as a backup, write a
  saved backup back. Same JSON format as `blheli32.py backup`, so files move freely between the
  page and the CLI.
- **Settings editor** (option C): every setting as a named form field with its valid range or
  options, side by side for left and right, with the fields that differ between the two ESCs
  highlighted and a button per field to copy its value to the other ESC, and a diff against what
  the ESC holds before writing.

Tuning mode is bench only. It is refused while armed, and the motors cannot run inside it.

## Done: the protocol and the settings layout

The original steps 0 and 1 (capture BLHeliSuite32, map every field) are finished. The result is
`firmware/blheli/README.md` (the spec) and a working Python implementation over USB:

| Piece | Holds |
|---|---|
| `auto_battlebot/blheli32/fourway.py` | 4-way framing, CRC16-XMODEM, the `FourWay` client over any `SerialPort` (write, read, reset_input_buffer) |
| `auto_battlebot/blheli32/cipher.py` | XTEA page encryption with the per-block tweak; 256-byte page to 192-byte payload and back |
| `auto_battlebot/blheli32/layout.py` | 45 fields, 28 of them editable, with ranges, option labels, units; music and name encoders |
| `auto_battlebot/blheli32/esc.py` | `Session.read` / `Session.write`: layout check, stale-base check, one write, read-back verify |
| `firmware/blheli/blheli32.py` | `read`, `backup`, `show`, `set`, `write`, `fields` |
| `tests/python/test_blheli32.py` | tests against pages read from the robot |

What the four unknowns in the old plan turned out to be:

1. **Where the settings live.** One 256-byte page at `0xF800`, XTEA-encrypted in 8-byte blocks,
   each block 2 nonce bytes plus 6 payload bytes, so a 192-byte payload. BLHeliSuite also reads
   16 bytes at `0xFA00` (input protocol, good/bad frame counters) and 16 at `0xFD90` (unknown,
   read-only).
2. **Checksum.** None. The ESC draws a new nonce on every read, so the same settings never give the
   same ciphertext twice. A decode is validated by the firmware and layout bytes (32, 90, 48).
3. **What Write Setup touches.** One `DeviceWrite` of the whole encrypted page at `0xF800`. No
   `DevicePageErase`; the bootloader erases. Nothing else is written. Reserved bytes go back
   exactly as read.
4. **Field encodings.** Mapped by stepping every BLHeliSuite slider. Two fields stay unverified
   (`current_protection`, `current_sense_cal`): BLHeliSuite hides them on ESCs without current
   sense, so the tool carries them through and validates only 0-255.

Hardware round trip, 2026-10-23: startup beep volume 254 to 200 on ESC 0, written, verified in a
fresh bootloader session, restored. Only payload byte 20 changed; ESC 1 was untouched.

## Known quantities

| Quantity | Value | Source |
|---|---|---|
| ESC firmware | BLHeli_32 32.90, layout revision 48, STM32G071x6 | `firmware/blheli/README.md` |
| ESC layout name | `Flycolor_Raptor_5` | same |
| Signature | `06 47 6d 04` from `DeviceInitFlash` | same |
| ESC 0 | pin A2, the robot's **right** ESC (RMT TX 2, RX 5), direction 3 | passthrough `Global.cpp` `esc_pins = {A2, A3}`; `src/main.cpp` `RIGHT_ESC_PIN` |
| ESC 1 | pin A3, the robot's **left** ESC (RMT TX 3, RX 4), direction 4 | same, `LEFT_ESC_PIN` |
| Payload differences between ESCs | byte 3 (direction) and byte 144 (startup note) only | `firmware/blheli/README.md` |
| Signal path | each pad through 100 ohm to its GPIO | `docs/plans/motor_stall_protection_plan.md` |
| Bootloader link | half-duplex 19200 8N1 on the signal wire, CRC16 0xA001 | `firmware/BlHeli-Passthrough/ESP32/ESC_Serial.cpp` |
| Bootloader entry | send `RestartBootloader`, hold the line low 300 ms, release high | `4Way.cpp` `cmd_DeviceReset` |
| Read time | about 300 ms per 256 bytes | `4Way.cpp` `cmd_DeviceRead` |
| Loop watchdog | 5 s per `loop()` pass | `src/main.cpp` |
| Web server | ESPAsyncWebServer: handlers run on the async TCP task and must not block | `src/diagnostics_server.cpp` |
| Armed state | `radio_ok && radio_data->armed`; ESP-NOW driving is `diag.espnow.driving` | `src/main.cpp`, `diagnostics_server.h` |
| ESC wake-up | 2 s of zero-throttle frames before control | `start_setup_frames()` / `stop_setup_frames()` |

ESC indices stay as the passthrough numbers them (0 = A2 = right, 1 = A3 = left), because the
backup JSON records `"esc": 0|1` and `write` refuses a file from the other ESC. The page shows
**Right (ESC 0)** and **Left (ESC 1)**. The README's "ESC Pin Mapping" table says ESC 1 = left =
A2, which is wrong on both counts; step 5 fixes it.

## Remaining prerequisite

Power-cycle the pack after a beep-volume change and listen. The round trip above proved the
bootloader stores the page; it has not yet proved the running firmware picks it up. Do this with
the CLI over USB before building anything below.

## Design: the robot relays 4-way frames

The firmware does not decode settings. It speaks the same 4-way protocol the USB passthrough does,
with HTTP in place of the serial port, and adds the address guard. Everything that knows about
encryption and layout stays in one place per language:

- **Python** (`auto_battlebot/blheli32/`) drives the robot over WiFi through an HTTP transport that
  satisfies `fourway.SerialPort`. The CLI and its tests then validate the firmware before any page
  exists.
- **Browser** (`web/esc.js`) ports `cipher.py` and `fourway.py`, and reads the field table as JSON
  generated from `layout.py`, so the layout has one source.

This replaces the old plan's firmware-side job API (`read`/`write` of a whole block). It costs one
extra round trip per command, which is nothing next to the 300 ms ESC read, and it keeps the
firmware to a port of code that already works against these ESCs.

## Step 0 (done): everything BLHeli in `firmware/blheli/`

```
firmware/blheli/
  README.md              the spec and CLI usage (was docs/blheli32_settings.md)
  blheli32.py            the CLI (was scripts/blheli32.py)
  mr_stabs_mk2_esc.ixi   BLHeliSuite32 setup export (was firmware/config/blheli/)
  captures/              raw and decrypted pages (was firmware/config/blheli/captures/)
  backups/               CLI backups (was firmware/config/blheli/backups/)
```

- The library stays in `auto_battlebot/blheli32/`, since shared, importable, type-checked code
  lives there, and the tests stay in `tests/python/test_blheli32.py`. The CLI imports the library
  with no `sys.path` changes.
- The CLI has a shebang and the exec bit, like the other `firmware/` scripts, which keeps ruff's
  INP001 quiet. It is out of mypy's scope with the rest of `firmware/`, which is fine for a thin
  CLI over a type-checked library.
- New files from this plan that are not robot firmware go here too: the `--url` transport docs,
  and any BLHeli helper scripts. The served layout JSON stays in `firmware/mr_stabs_mk2/web/`,
  since the robot serves it from there.

## Step 1: 4-way relay and tuning mode in the Mk2 firmware

### Module layout

| Piece | Holds | Tested |
|---|---|---|
| `lib/blheli_boot/` | 4-way frame parse/build and CRC16-XMODEM, bootloader command builders and reply parsers, CRC16 0xA001, the address guard | host gtest, `env:native` |
| `src/esc_config.cpp`, `include/esc_config.h` | serial transport on the ESC pins, the tuning state machine, the one-slot frame queue | bench |
| `diagnostics_server.cpp` | HTTP routes, frame handoff | bench |

`lib/blheli_boot` is ported from `BlHeli-Passthrough/ESP32/4Way.cpp` and `ESC_Serial.cpp`, keeping
the 4-way dispatcher (the request frames are the same ones BLHeliSuite and `fourway.py` send) and
dropping MSP. It replaces the fixed `delay(param)` waits with reads that end on the expected length
plus CRC. Commands it handles: `TestAlive`, `ProtocolGetVersion`, `InterfaceGetName`,
`InterfaceGetVersion`, `InterfaceExit`, `InterfaceSetMode` (ARM BLB only), `DeviceReset`,
`DeviceInitFlash`, `DeviceRead`, `DeviceWrite`. `InterfaceGetName` returns a distinct name (for
example `MrStabsMk2`) so a log shows which relay answered.

The **address guard** is the bricking protection. `DeviceWrite` is accepted only at `0xF800` with
exactly 256 bytes. `DevicePageErase` is refused outright, since the write path never uses it.
Refusals return a 4-way NACK. Reads are unrestricted. A bad page can then corrupt settings
(fixable by restoring a backup) but cannot overwrite the ESC's application or bootloader.

### Transport

Start with the plerup SoftwareSerial the standalone passthrough uses (known to work with these
ESCs). It runs on core 1 inside `loop()`, and WiFi stays on core 0, so its bit timing should hold.
If the step 2 bench test shows CRC failures with WiFi busy, switch to a hardware UART (UART0 is
free, since `Serial` is the USB-Serial/JTAG port) with TX and RX routed to the same pin and the TX
open-drain, swapping pins between the two ESCs per command.

### State machine

```
COMBAT --enter request, disarmed, no ESP-NOW drive, no OTA--> ENTERING
ENTERING: stop_escs(); left/right deinit (RMT TX+RX released); pins handed to blheli_boot
          --> TUNING
TUNING:   runs at most one queued 4-way frame per loop() pass; LED blue breathing;
          control, motor_guard output, and DShot are all off
TUNING --exit request or radio button edge--> EXITING
EXITING:  DeviceReset on both ESCs; DShot begin() on both; start_setup_frames();
          2 s; stop_setup_frames() --> COMBAT
```

Bootloader entry per ESC stays where the client puts it (`DeviceInitFlash` / `DeviceReset`, as
`Session.read` does), so the firmware holds no per-ESC session state beyond what `4Way.cpp` keeps.

Rules:

- **Entry check** happens in `loop()`, which owns the armed state: `!(radio_ok && armed)`, not
  `espnow.driving`, not `espnow.ota_in_progress()`. The HTTP handler only sets a request flag,
  the same pattern as `take_erase_dump_request()`.
- **Arming inside TUNING does nothing.** DShot is off, so the ESCs cannot drive. The page shows
  "armed while tuning: exit to drive". There is no auto-exit on arm, because a write half done when
  the switch flips would leave a corrupt page.
- **The loop keeps reading** CRSF, the IMU and the INA238 in TUNING, so telemetry and the page stay
  live. `motor_guard` keeps cooling its thermal model.
- **ESP-NOW OTA and ArduinoOTA are refused** in TUNING. A reboot is a clean exit: the ESCs leave
  their bootloader on power-up and the normal 2 s DShot wake-up runs.
- **Watchdog.** One frame is one bootloader transaction. The longest is the 256-byte
  `DeviceWrite` (about 140 ms on the wire at 19200 plus the flash write), well under 5 s.
- **Idle timeout.** No frame for 60 s in TUNING sends `DeviceReset` to both ESCs, so a closed
  browser tab does not leave an ESC parked in its bootloader. TUNING itself stays until exit.
- **eRPM after exit.** The status panel's motor guard sensor state has to read `FUSED` again within
  3 s of COMBAT. If not, the exit sequence is wrong.

### HTTP API

All under `/esc/`, handled on the async TCP task, never blocking on the ESC:

| Route | Does |
|---|---|
| `POST /esc/tuning` `{"enter": true\|false}` | sets the request; 409 while armed |
| `GET /esc/status` | state, last error, guard refusals count, idle seconds |
| `POST /esc/4way` body: one request frame, hex | queues it; 409 if a frame is pending or not TUNING |
| `GET /esc/4way` | `{"state": "pending"}`, or `{"state": "done", "frame": "<hex response>"}` |

## Step 2: drive the relay from Python

Add `HttpPort` to `auto_battlebot/blheli32/` (`http_port.py`): `write()` posts the buffered request
frame, `read()` polls `GET /esc/4way` and serves the response bytes, `reset_input_buffer()` drops
any leftover. `FourWay.enter()` skips the MSP switch on this transport (the relay is 4-way only).
`firmware/blheli/blheli32.py` gets `--url http://mr-stabs.local` as an alternative to `--port`; every
subcommand works unchanged.

Bench validation, props off and wheels clear:

1. Enter tuning from the page or `curl`, `blheli32.py --url ... read`, exit, drive. Repeat 10
   times. No failed entry, no jingle loop, guard back to `FUSED`.
2. 50 reads of both ESCs with the diagnostics page open on two browsers and the telemetry recorder
   running. Zero CRC failures. If any, switch transport (step 1).
3. Write each ESC's own backup back with `write`. Read-back matches. The same ESC read over the USB
   passthrough (`--port`) gives the same payload.
4. `set` one field (startup beep volume), `write` over WiFi, confirm by ear after a power cycle,
   restore.
5. Guard: send a `DeviceWrite` at `0xF700` and a `DevicePageErase` by hand. Both NACK, the ESC
   still reads clean.
6. Pull the pack mid-write. On power-up the ESC either boots with old or new settings, or plays its
   "settings invalid" tone. Restore from the backup over WiFi and confirm it recovers. This is the
   worst case the address guard leaves, and it has to be recoverable before the page ships.

## Step 3: backup / restore panel (B)

A **TUNING MODE** button in a new ESC panel on `web/index.html`. Disabled with a reason while armed
or driven over ESP-NOW. `web/esc.js` holds the 4-way client and the cipher, ported from
`fourway.py` and `cipher.py`. Once in TUNING:

- **Read** per ESC: shows firmware/layout ids, good/bad frame counts, and downloads
  `esc<N>_<timestamp>.json`, the same document `snapshot_json()` writes. The page refuses Write on
  an ESC it has not read this session.
- **Restore** per ESC: pick a backup JSON. The page applies the CLI's rules: refuse a file whose
  `esc` is the other ESC, check `page_b64` decodes to `payload_hex`, check the layout bytes, show
  the changed fields, offer a download of the ESC's current settings, then write and verify.
- **Exit tuning**.

There is no whole-block copy from one ESC to the other. The two ESCs differ in direction and
startup note, so a straight copy is wrong. The editor copies field by field instead (step 4).

## Step 4: settings editor (C)

A second view in the ESC panel, built on B's read and write. The firmware does not change.

- **Field table.** `layout.py` gains `to_json()`, and a small generator writes
  `firmware/mr_stabs_mk2/web/esc_layout_32_90_r48.json` (key, offset, width, kind, range, options,
  unit, off value, verified, doc). A pytest asserts the committed file equals a fresh generation,
  so the page and the CLI cannot drift.
- After Read, decode both payloads and show every field: a number input with min/max/unit, or a
  dropdown for option fields. Right and left side by side in one row per field. Name and music
  notes are text inputs using the CLI's syntax (`A5 1/8 D5 1/4`).
- **Differences.** A row whose right and left values differ is highlighted, and the panel header
  counts them ("3 fields differ"). On the current setup that is `direction` and `music_notes`. A
  **Show differences only** toggle hides the matching rows. A second, separate highlight marks a
  cell edited in the form but not yet written, so "differs from the other ESC" and "differs from
  what this ESC holds" never look the same.
- **Copy per field.** Each row has a `→` and a `←` button that copy that one field's value from one
  ESC's column to the other. Copying only edits the form; nothing reaches an ESC until Apply.
  Copying `direction` triggers the firmware dependency confirm below, since both wheels would then
  spin the same way. Read-only rows have no copy buttons.
- Read-only fields shown greyed: firmware/layout ids, `hw_*` capability bytes, `esc_layout`,
  `esc_mcu`. Options the hardware cannot do (PWM frequencies outside `hw_pwm_freq_min/max`) are
  disabled. The two unverified fields are shown with a note and accept 0-255.
- **Apply** shows a table of field, old, new, then encodes the page and writes it, passing the
  payload it read as the expected current state (the `expect_current` check in `Session.write`).
  **Apply to both** writes the same field changes to each ESC's own payload, so direction and the
  startup note keep their per-ESC values.
- **Layout check.** If the ESC's firmware or layout bytes are not 32 / 90 / 48, the editor is
  read-only and says why. B's backup and restore still work.
- **Firmware dependency warning.** The robot firmware assumes bidirectional 3D mode
  (`esc.h`, `lib/erpm_decode`). Changing `direction` away from 3 or 4 gets a confirm dialog that
  says throttle mapping breaks. Swapping 3 and 4 on one ESC is allowed, with a note that it reverses
  that wheel.

Tests:

- `tests/python/test_blheli32.py`: the JSON table matches `layout.py`.
- `web/esc.test.mjs`, run with `node --test`: decode the captured pages in
  `firmware/blheli/captures/` and match the Python decode; encode then decode round-trips
  every field at its min, max and each option; 4-way CRC against frames from `test_blheli32.py`.
- `lib/blheli_boot` gtests: CRC against captured frames, address guard accepts `0xF800`/256 and
  rejects one byte either side, any other length, and `DevicePageErase`; reply parser rejects bad
  CRC and short replies.
- Bench: change rampup power, motor timing and brake on stop through the editor, verify each with
  `blheli32.py read` over USB, drive, restore.
- Bench: change startup beep volume on the right ESC only, confirm its row highlights, copy it to
  the left with `←`, Apply, read both back, and confirm the row no longer highlights.

## Step 5: docs and cleanup

- README: replace "Configuring ESCs via USB Passthrough" (stale since `4ab45d61`) and the "Tuning
  mode" line that says USB passthrough activates, with the tuning mode flow; fix the ESC pin
  mapping table; note that the standalone passthrough is still the way to flash ESC firmware.
- `firmware/blheli/README.md`: add the WiFi transport (`--url`) to Usage and remove its "next
  steps" item about the dashboard.
- A tuning log (README section or `docs/esc_tuning.md`): what each setting does, from the field
  `doc` strings and the BLHeli_32 manual, and each tuning change with the bench or match data
  behind it.

## Out of scope

- Flashing ESC firmware. The address guard blocks it on purpose; use BLHeliSuite32 with the
  standalone passthrough.
- Any ESC firmware other than 32.90 / layout 48. A new version needs new captures and a check of
  `layout.py` against them.
- Importing or exporting BLHeliSuite `.ini` / `.ixi` files. The backup JSON is the record, and
  `blheli32.py show` prints it. Revisit if I need to load a setup someone else made in BLHeliSuite.
- Changing settings while armed or between matches without a bench.

## Next steps

1. Power-cycle after a beep-volume change and confirm by ear (the remaining prerequisite).
2. Build `lib/blheli_boot` with its gtests, then tuning mode and the `/esc/` routes (step 1).
3. Add `HttpPort` and `--url`, and run the step 2 bench list including the pulled-pack test.
4. Then the backup/restore panel, then the editor.
