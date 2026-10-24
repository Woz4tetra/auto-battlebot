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

Status, 2026-10-23 evening: built, and run on the robot over the USB dashboard (below). The page
read both ESCs and wrote and verified a new startup tune on both. Before that the page failed every
read (`command 0x3A at 0xF800 failed, ack 0x0f`); "Bench findings" has what was wrong. "Left to do
on the robot" at the end says which checks are still open.

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
| Bootloader entry | stop DShot and hold the line high: the ESC drops into its bootloader within about 1 s. The passthrough's `RestartBootloader` plus 300 ms low restarts the ESC into its application instead | robot, 2026-10-23 ("Bench findings") |
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
the CLI over USB before the hardware checklist at the end.

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
  export_layout.py       writes the page's field table from layout.py
  mr_stabs_mk2_esc.ixi   BLHeliSuite32 setup export
  captures/              the four pages the tests read; other dumps are gitignored
  backups/               CLI and page backups, gitignored (local to each machine)
```

The library stays in `auto_battlebot/blheli32/` (shared, importable, type-checked) and the tests
in `tests/python/test_blheli32.py`. The served field table stays in `firmware/mr_stabs_mk2/web/`.

## Step 1 (built, untested on hardware): 4-way relay and tuning mode

| Piece | Holds | Tested |
|---|---|---|
| `lib/blheli_boot/` | 4-way parse/build, CRC16-XMODEM and 0xA001, the bootloader client, the address guard, `Server::handle()` | 24 gtests in `env:native` with a scripted fake link; robot |
| `src/esc_config.cpp`, `include/esc_config.h` | UART2 link on the ESC pins, the state machine, the one-slot frame queue | robot |
| `src/diagnostics_server.cpp` | the `/esc/` routes | robot |
| `lib/DShot` | `end()`: uninstalls both RMT channels and resets the pad, so the pins can change hands and `begin()` works again | robot: the ESCs counted 6400 good DShot frames and 0 bad after a tuning session |

`blheli_boot` is the passthrough's `4Way.cpp` and `ESC_Serial.cpp` minus MSP, with these changes:

- Replies are read until the line goes quiet and checked from their end, so a stray leading byte
  does not fail them.
- `DeviceInitFlash` requires the full `"471"` reply and retries the handshake. If the ESC never
  answers, it restarts the ESC the passthrough's way, waits 500 ms, and tries once more.
- `DeviceRead` retries 3 times, then redoes the handshake and tries 3 more.
- `DeviceReset` only selects the ESC (see "Bench findings").
- The write keeps the passthrough's sequence and retries (set address, set buffer, data, program),
  but stops before `ProgFlash` when the address or the data was not acknowledged.

`InterfaceGetName` answers `MrStabsMk2`.

The **address guard** accepts `DeviceWrite` only at `0xF800` with exactly 256 bytes and refuses
`DevicePageErase` and `DeviceEraseAll`, all with ack `0x09`, counted in `/esc/status`. Reads are
unrestricted. A bad page can corrupt settings (restored from a backup) but not the ESC's
application or bootloader.

### Transport

UART2 (UART1 is the CRSF receiver), routed through the GPIO matrix to the selected ESC's pin with
TX and RX on the same pin, open drain with the pull-up. The receiver hears every byte sent, and the
link drops that echo before reading the reply. The other ESC's line is driven high. This replaced
the passthrough's EspSoftwareSerial, which lost bits inside this firmware (see "Bench findings").

### State machine

```
COMBAT --enter request, disarmed, ESP-NOW not driving, 3 s since the last exit--> TUNING
         (both Esc::deinit(): a last stop frame, RMT released; both lines driven high)
TUNING:  after 1 s of high line, runs at most one queued 4-way frame per loop() pass;
         LED breathes blue
TUNING --exit request or radio button press--> EXITING
         (DeviceReset on both ESCs, serial released, Esc::begin() on both)
EXITING: zero-throttle DShot from the disarmed branch for 2 s --> COMBAT
```

- `loop()` owns the armed state, so the HTTP handler only queues the request. Entry refused while
  armed or while ESP-NOW drives leaves the reason in `/esc/status` `error`.
- Tuning mode runs through the disarmed branch whether or not the robot is armed, so the outputs
  stay stopped and the page shows "armed while tuning". No auto-exit on arm: a write half done
  when the switch flips would leave a corrupt page.
- CRSF, the IMU, the INA238 and the motor guard keep running, so telemetry and the page stay live.
- ArduinoOTA is not serviced and ESP-NOW OTA sees "armed" while tuning, so neither starts. A
  reboot is a clean exit.
- One frame blocks the loop for at most about 3 s (a write with every retry), under the 5 s
  watchdog. An exit request waits for the frame in flight.
- No frame for 60 s resets both ESCs out of their bootloader. Tuning mode stays.

### HTTP API

| Route | Does |
|---|---|
| `POST /esc/tuning` form `enter=1\|0` | queues the request; 409 while armed |
| `GET /esc/status` | `state` (`combat`, `tuning`, `exiting`), `error`, `frames`, `guard_refusals`, `idle_s`, `released`, `pending`, `armed`, `driving`, `failure` (why the latest bootloader exchange failed, with the bytes received) |
| `POST /esc/4way` form `frame=<hex>` | validates the frame (400 with the reason), queues it, returns `{"id": n}`; 409 when not tuning or a frame is pending |
| `GET /esc/4way` | `{"id", "state": "none"\|"pending"\|"done", "frame": <hex response>}` |

## Step 2 (built): drive the relay from Python

`auto_battlebot/blheli32/http_port.py`: `HttpPort` satisfies `fourway.SerialPort`. `write()` posts
each complete request frame and polls until it is done, `read()` serves the response. `Session`
and `FourWay` take `msp=False` to skip the MSP switch. `blheli32.py --url URL` enters tuning mode
for the command and leaves it after (only if it entered). Tested against a local fake of the
`/esc/` routes: a read and a verified write through `Session`, and the 409 refusal while armed.

## Step 3 and 4 (built): ESC SETTINGS panel

`web/esc.js` is the codec and 4-way client, ported from `fourway.py`, `cipher.py` and `layout.py`;
`web/esc_panel.js` is the panel. `layout.to_json()` writes `web/esc_layout.json` through
`firmware/blheli/export_layout.py`, and `scripts/embed_web.py` now serves `.json`.

- **TUNING MODE** button, disabled with the reason while armed or driven over ESP-NOW. The panel
  body shows only in tuning mode, and leaving tuning mode drops every read, so a new session
  starts from fresh reads.
- **Columns** Left (ESC 1, A3) and Right (ESC 0, A2), as seen from behind the robot. READ, BACKUP,
  RESTORE, APPLY per ESC; READ BOTH, APPLY BOTH, REVERT EDITS across both.
- **Backups.** The first read of each ESC in a session downloads `esc<N>_<time>.json`, and every
  write first downloads `esc<N>_before_write_<time>.json`, both in the CLI's format. RESTORE
  refuses a file from the other ESC, an inconsistent `page_b64`, or another layout, then loads it
  into the form for the normal review.
- **Differences.** Rows whose left and right differ are amber, the header counts them, and SHOW
  DIFFERENCES ONLY hides the rest. An outlined cell is edited and not written. Invalid input turns
  red and blocks APPLY with the reason, as do the cross-field rules (min < center < max throttle,
  PWM low <= high) and PWM frequencies outside the ESC's `hw_pwm_freq_min/max`.
- **Copy per field.** `→` copies a value left to right, `←` right to left. Copying edits the form
  only.
- **Review.** APPLY and APPLY BOTH list every change, old and new labels and the payload bytes it
  writes (`byte 20: fe → c8`), with warnings: direction
  outside bidirectional 3D (3, 4) breaks the robot's throttle mapping, direction equal to the
  other ESC's turns one wheel the wrong way, and a change to an unverified field. WRITE writes
  each ESC with the payload it read as the expected current state, and reads back to verify.
- **Layout check.** An ESC whose firmware or layout bytes do not match the table is read-only in
  the editor; BACKUP still works.

Tested: `web/esc.test.mjs` (15 node tests, also run by pytest) decodes the captured pages to
BLHeliSuite's values, round-trips every field at its limits and options, matches the music
encoding and the 4-way frames, and runs the client's read, verified write, dropped-write and
stale-read checks against a fake ESC. The whole page flow (enter, read both, edit, copy, invalid
input, review with the direction warning, write both, exit, the four downloads) passed in
headless Chrome against a mock robot, with no horizontal overflow at 390 px.

## Step 5 (done): docs

- `firmware/mr_stabs_mk2/README.md`: tuning mode, "Configuring ESCs" (page, CLI over WiFi,
  flashing through the passthrough), and the pin table fixed (ESC 0 = A2 = right).
- `firmware/blheli/README.md`: `--url`, the relay's routes and guard; "What each setting does", the
  field reference from the BLHeli_32 manual with what each field means on Mr Stabs; and an empty
  "Tuning log" table for each settings change that reaches the robot, with its evidence.

## Bench findings, 2026-10-23

The robot on a bench supply, wheels clear, driven from the laptop over the USB dashboard. Each
bug and its fix:

- Bug: `DeviceInitFlash` passed on junk. It accepted any reply of 6 or more bytes ending in `0x30`,
  so a garbled `d3 6a 1d 07 04 30` gave signature `30 04 07 04` and the session went on to fail.
  Fix: require the full 9-byte `34 37 31 6d 47 06 07 04 30` and retry the handshake.
- Bug: the settings read failed on a stray byte. The set-address ack arrived as `fe 30`, and the
  first-byte check refused it. That is the page's original `ack 0x0f`. Fix: check replies from
  their end, and retry reads.
- Bug: the software serial link lost bits inside this firmware. 40 of 72 reads needed a retry: bad
  CRCs, a missing byte, `0x30` read as `0xfe`. Moving the USB bridge between cores did not change
  it. Fix: UART2 in open-drain half duplex. 0 retries in the 144 reads, 48 handshakes and 6 CLI
  sessions logged after the switch.
- Bug: `DeviceReset` before every handshake made entry flaky. Its 300 ms low restarts the ESC into
  its application, which then has to fall back into the bootloader through its lost-signal check.
  A handshake landing on that check left the ESC unreachable in 1 CLI session in 10. Stopping
  DShot with the line held high puts the ESC in its bootloader by itself. Fix: `DeviceReset` only
  selects the ESC; the relay restarts it only when the handshake gets no answer. Leaving tuning
  mode and the 60 s idle timeout still restart both.
- Bug: entering again right after an exit failed 3 times in 8. The exit restarts both ESCs. Fix:
  entry waits until 3 s after the exit finished (0 failures in 14 after 1 s or more), and frames
  wait 1 s after entry with both lines driven high.

The page then read both ESCs in 2.4 to 3.8 s. It wrote a new startup tune to both ("Tuning log" in
`firmware/blheli/README.md`) and verified it, and a CLI read afterwards differed from the backup in
payload bytes 144 to 147 only.

## USB dashboard

`firmware/mr_stabs_mk2/scripts/usb_dashboard.py` serves the diagnostics page on
`http://localhost:8080` over the USB cable, so the page and `blheli32.py --url
http://localhost:8080` work without WiFi. The robot's log lines print in the same terminal. On
the robot, `src/usb_bridge.cpp` replays each request against its own web server on 127.0.0.1.
Details in `firmware/mr_stabs_mk2/README.md`.

## Left to do on the robot

Done on 2026-10-23: flashing over USB, repeated entry and exit with reads (more than 20 sessions),
reads of both ESCs (more than 150), and a page write of both ESCs verified by a CLI read. Still open:

1. Power-cycle the pack and listen: both ESCs should play the new F major tune together. This also
   closes the old prerequisite (a written setting takes effect after a power cycle).
2. Drive after a tuning session: motor guard sensors back to `FUSED` within 3 s, no jingle loop.
   The bench run never spun the motors.
3. Repeat the reads over WiFi with the page open on two browsers and the telemetry recorder
   running. Everything so far went over the USB bridge.
4. Write each ESC's own backup back with `write --url` (`firmware/blheli/backups/*_205816.json`
   hold the pre-tune state). The same ESC read over the USB passthrough gives the same payload.
5. Guard: send a `DeviceWrite` at `0xF700` and a `DevicePageErase` by hand. Both ack `0x09`, the
   ESC still reads clean.
6. Pull the pack mid-write. The ESC boots with old or new settings or plays its "settings invalid"
   tone; restore over WiFi and confirm it recovers.
7. Page: change rampup power, motor timing and brake on stop, verify with `blheli32.py read`,
   drive, restore. Change startup beep volume on one ESC, copy it across, APPLY BOTH, confirm the
   row stops highlighting.

## Out of scope

- Flashing ESC firmware. The address guard blocks it on purpose; use BLHeliSuite32 with the
  standalone passthrough.
- Any ESC firmware other than 32.90 / layout 48. A new version needs new captures and a check of
  `layout.py` against them.
- Importing or exporting BLHeliSuite `.ini` / `.ixi` files. The backup JSON is the record, and
  `blheli32.py show` prints it.
- Changing settings while armed or between matches without a bench.
