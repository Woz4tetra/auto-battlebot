# Mr Stabs Mk2 ESC settings from the WiFi dashboard

## Goal

Read and write every BLHeli_32 setting on both drive ESCs from the robot's diagnostics page, so I
can tune them for performance without a USB cable or BLHeliSuite32. A **TUNING MODE** button on the
page stops the motors, hands the ESC signal wires to the BLHeli bootloader protocol, and exposes
two panels:

- **Backup / restore** (option B): read each ESC's raw settings block, download it as a `.bin`,
  write a saved block back, copy one ESC's settings to the other. Needs no knowledge of the layout.
- **Settings editor** (option C): every setting as a named form field with its valid range or
  options, side by side for left and right, with a diff against what the ESC holds before writing.

Tuning mode is bench only. It is refused while armed, and the motors cannot run inside it.

## Known quantities

| Quantity | Value | Source |
|---|---|---|
| ESC firmware | BLHeli_32 32.90, layout revision 48 | `firmware/config/blheli/mr_stabs_mk2_esc.ini` |
| ESC layout name | `Flycolor_Raptor_5` | same |
| Settings in the `.ini` | 36 editable `Eep_Pgm_*` / `Eep_Note_*` fields, plus read-only `Eep_Hw_*` capability fields | same |
| Left ESC pin | A3, RMT TX 3, RX 4 | `src/main.cpp:32`, `:393` |
| Right ESC pin | A2, RMT TX 2, RX 5 | `src/main.cpp:33`, `:394` |
| Signal path | each pad through 100 ohm to its GPIO | `docs/plans/motor_stall_protection_plan.md` |
| Bootloader link | half-duplex 19200 8N1 on the signal wire, CRC16 0xA001 | `firmware/BlHeli-Passthrough/ESP32/ESC_Serial.cpp` |
| Bootloader entry | send `RestartBootloader`, hold the line low 300 ms, release high | `4Way.cpp` `cmd_DeviceReset` |
| Read time | about 300 ms per 256 bytes (`delay(256)` plus inter-byte timeout) | `4Way.cpp` `cmd_DeviceRead` |
| Loop watchdog | 5 s per `loop()` pass | `src/main.cpp:456` |
| Armed state | `radio_ok && radio_data->armed`; ESP-NOW driving is `diag.espnow.driving` | `src/main.cpp`, `diagnostics_server.h` |
| ESC wake-up | 2 s of zero-throttle frames before control | `start_setup_frames()` / `stop_setup_frames()` |

The README's "ESC Pin Mapping" table says ESC 1 = left = A2, which disagrees with `main.cpp`
(left = A3). The page labels ESCs **Left** and **Right** from the `main.cpp` pins, never "ESC 1/2",
and the README table gets fixed in step 5.

## What is unknown

1. **Where the settings live.** BLHeli_32 never published its settings layout, and the open
   configurators (esc-configurator, blheli-configurator) do not support BLHeli_32. The flash
   address and length of the settings block come from step 0.
2. **Whether the block has a checksum.** If one setting change flips one or two bytes, there is
   none. If a trailing word changes every time, it has to be found and recomputed before C can
   write.
3. **Whether BLHeliSuite32 writes anything besides the settings block** during Write Setup (a
   second page, a flag byte). The capture in step 0 shows every write.
4. **Field encodings.** Some `.ini` values look raw (`Ppm_Min_Throttle=1000`), some look like
   indices or scaled values (`Comm_Timing=16`, `Temp_Prot_Enable=140`, `Direction=6`). The UI
   ranges and option labels come from BLHeliSuite32's own dropdowns, recorded by hand.

Steps 0 and 1 answer all four with no robot firmware changes. Nothing in B or C ships before they
are done.

## Step 0: capture BLHeliSuite32's traffic

Run BLHeliSuite32 against the standalone passthrough (`firmware/BlHeli-Passthrough`) through a
logging serial bridge, so every 4-way frame lands in a file:

```bash
# Flash the standalone passthrough to the robot's QT Py (USB), then:
socat -x -v \
  pty,link=$HOME/.wine/dosdevices/com5,raw,echo=0 \
  /dev/ttyACM0,raw,echo=0,b115200 \
  2> firmware/config/blheli/captures/read_setup.log
```

In BLHeliSuite32 pick `COM5`, Connect, Read Setup. Then capture a Write Setup with no changes.

Add `firmware/scripts/blheli_capture.py`: parses a socat hex log into 4-way frames
(`0x2F` PC to interface, `0x2E` back), checks each CRC, and prints one line per command with ESC
index, address, length, and the data. It reconstructs the bytes each ESC returned for each
address range and writes them to `<name>_esc<N>_<addr>.bin`.

Outputs, committed under `firmware/config/blheli/`:

- `captures/*.log`, the raw socat logs.
- `layout_32_90_r48.md`: the settings address and length, which other addresses BLHeliSuite32
  touches, and the exact write sequence (erase, set buffer, program, in what chunk sizes).
- `mr_stabs_mk2_left_<date>.bin`, `mr_stabs_mk2_right_<date>.bin`: the current settings, as the
  first backup.

If the write sequence touches anything outside the settings block, stop and record it before step 1.

## Step 1: map every field

The `.ini` gives the current values, so most offsets can be found in one dump before any edits:
search the block for each value as u8, u16 LE, and u16 BE (1000 = `E8 03`, 1500, 2000, 600 are
distinctive). Then confirm each field by changing it alone:

1. Change one setting in BLHeliSuite32, Write Setup, Read Setup, under the socat capture.
2. `blheli_capture.py --diff before.bin after.bin` prints the changed offsets.
3. Record offset, width, endianness, the raw value at each UI choice, and BLHeliSuite32's displayed
   label and range.

Repeat for every editable field in the `.ini`, both ESCs for the first two (to rule out per-ESC
differences). Then step through each dropdown of a few enum fields (`Direction`, `Comm_Timing`,
`Demag_Comp`, `LED_Control`) to get every raw value, not only the endpoints. `Eep_Note_Array` (the
startup tune) is a string: map its encoding from two different tunes.

Output: `firmware/mr_stabs_mk2/web/esc_layout_32_90_r48.json`, the one table the page reads.

```json
{
  "fw_main": 32, "fw_sub": 90, "layout_rev": 48,
  "address": "0x....", "length": 0,
  "id_fields": {"layout_rev": {"offset": 0, "type": "u8"}},
  "checksum": null,
  "fields": [
    {"key": "Pgm_Rampup_Pwr", "label": "Rampup Power", "offset": 0, "type": "u8",
     "min": 3, "max": 150, "unit": "%"},
    {"key": "Pgm_Direction", "label": "Motor Direction", "offset": 0, "type": "u8",
     "options": {"1": "Normal", "2": "Reversed", "...": "..."}, "firmware_dependency": "3d"}
  ]
}
```

(The numbers above are placeholders. Every value in the real file comes from a capture.)

Done when every editable `.ini` field is in the table, and decoding the step 0 dump with the table
reproduces every value in `mr_stabs_mk2_esc.ini`. That check becomes a test (step 4).

## Step 2: tuning mode in the Mk2 firmware (B's backend)

### Module layout

| Piece | Holds | Tested |
|---|---|---|
| `lib/blheli_boot/` | CRC16, bootloader command builders and reply parsers (set address, read, set buffer, erase, program, keep-alive, restart), the address guard | host gtest, `env:native` |
| `src/esc_config.cpp`, `include/esc_config.h` | serial transport on the ESC pins, the tuning state machine, the job queue | bench |
| `diagnostics_server.cpp` | HTTP routes, job handoff | bench |

`lib/blheli_boot` is ported from `BlHeli-Passthrough/ESP32/4Way.cpp` and `ESC_Serial.cpp`, minus
the 4-way framing (the page talks to the robot over HTTP, not as a 4-way host). It replaces the
fixed `delay(param)` waits with reads that end on the expected length plus CRC.

The **address guard** is the bricking protection: the firmware writes and erases only inside the
settings block found in step 0, and refuses any other address with an error. Reads are
unrestricted. A wrong layout table can then corrupt settings (fixable by restoring a backup) but
cannot overwrite the ESC's application or bootloader.

### Transport

Start with the plerup SoftwareSerial the standalone passthrough uses (known to work with these
ESCs). It runs on core 1 inside `loop()`, and WiFi stays on core 0, so its bit timing should hold.
If the step 3 bench test shows CRC failures with the page open, switch to a hardware UART
(UART0 is free, since `Serial` is the USB-Serial/JTAG port) with TX and RX routed to the same pin
and the TX open-drain, swapping pins between the two ESCs per job.

### State machine

```
COMBAT --enter request, disarmed, no ESP-NOW drive, no OTA--> ENTERING
ENTERING: stop_escs(); left/right deinit (RMT TX+RX released); bootloader entry on both pins;
          keep-alive each; read the id fields --> TUNING (or back to COMBAT with an error)
TUNING:   runs one queued job per loop() pass; keep-alive every 1 s when idle;
          LED blue breathing; control, motor_guard output, and DShot are all off
TUNING --exit request or radio button edge--> EXITING
EXITING:  restart each ESC out of its bootloader; DShot begin() on both; start_setup_frames();
          2 s; stop_setup_frames() --> COMBAT
```

Rules:

- **Entry check** happens in `loop()`, which owns the armed state: `!(radio_ok && armed)`, not
  `espnow.driving`, not `espnow.ota_in_progress()`. The HTTP handler only sets a request flag,
  the same pattern as `take_erase_dump_request()`.
- **Arming inside TUNING does nothing.** DShot is off, so the ESCs cannot drive. The page shows
  "armed while tuning: exit to drive". There is no auto-exit on arm, because a write half done when
  the switch flips would leave a corrupt block.
- **The loop keeps reading** CRSF, the IMU and the INA238 in TUNING, so telemetry and the page stay
  live. `motor_guard` keeps cooling its thermal model.
- **ESP-NOW OTA and ArduinoOTA are refused** in TUNING. A reboot is a clean exit: the ESCs time out
  of their bootloader on power-up and the normal 2 s DShot wake-up runs.
- **Watchdog.** One job is one bootloader transaction (one read or one chunk of a write), well under
  5 s. A whole-block write is queued as a sequence of jobs, so the loop returns between chunks.
- **eRPM after exit.** The status panel's motor guard sensor state has to read `FUSED` again within
  3 s of COMBAT. If not, the exit sequence is wrong.

### HTTP API

All under `/esc/`, JSON, handled on core 0, never blocking on the ESC:

| Route | Does |
|---|---|
| `POST /esc/tuning` `{"enter": true\|false}` | sets the request; refused with 409 while armed |
| `GET /esc/status` | state, last error, per-ESC: in bootloader, fw/layout id bytes, last keep-alive ok |
| `POST /esc/job` `{"esc": "left", "op": "read"\|"write", "data": "<base64>"}` | queues a job on the whole settings block; 409 if one is pending or not TUNING |
| `GET /esc/job` | `pending`, `done` with base64 data, or `error` with the step that failed |

A write job is: erase the block, program it in the chunk size BLHeliSuite32 used, read it back,
compare. It reports `done` only if the read-back matches byte for byte.

## Step 3: backup / restore panel (B's page)

A **TUNING MODE** button in a new ESC panel on `web/index.html`. Disabled with a reason while armed
or driven over ESP-NOW. Once in TUNING:

- **Read** per ESC: shows the fw/layout ids, a hex view, and auto-downloads
  `mr_stabs_mk2_<left|right>_<iso time>.bin`. The page refuses Write on an ESC it has not read this
  session.
- **Restore** per ESC: pick a `.bin`, the page checks its length and id bytes against the ESC, shows
  the changed offsets, Write.
- **Copy left to right / right to left**: write one ESC's block to the other, after a diff view.
- **Exit tuning**.

Bench validation for B, all with props off and wheels clear:

1. Enter, read both, exit, drive. Repeat 10 times. No failed entry, no jingle loop, guard back to
   `FUSED`.
2. Read with the page open on two browsers and WiFi busy (the telemetry recorder running). Zero CRC
   failures in 50 reads. If any, switch transport (step 2).
3. Write each ESC's own block back unchanged. Read-back matches. BLHeliSuite32 over the standalone
   passthrough still reads both and shows the same settings as `mr_stabs_mk2_esc.ini`.
4. Write a block from step 1's captures with one field changed (Beep Strength), confirm the change
   in BLHeliSuite32 and by ear, restore the original.
5. Pull the pack mid-write. On power-up the ESC either boots with old or new settings, or plays its
   "settings invalid" tone. Restore from the backup through the page and confirm it recovers. This
   is the worst case the address guard leaves, and it has to be recoverable before C ships.

## Step 4: settings editor (C)

A second view in the ESC panel, built on B's read and write jobs. The firmware does not change:
all decoding and encoding happens in `web/esc.js` with `esc_layout_32_90_r48.json`.

- After Read, decode both blocks and show every field: a number input with min/max/unit, or a
  dropdown for option fields. Left and right side by side, mismatches highlighted.
- Read-only `Eep_Hw_*` capability fields shown greyed. Options the hardware cannot do (for example
  PWM frequencies outside `Hw_Pwm_Freq_Min/Max`, or current sense, which reads 255 here) are
  disabled.
- **Apply** shows a table of field, old, new, raw bytes, then encodes the new block (and its checksum
  if step 1 finds one) and queues the write. **Apply to both** writes the same field changes to
  each ESC's own block, so per-ESC fields like direction keep their own value.
- **Layout check.** If the ESC's fw or layout id bytes do not match the table, the editor is
  read-only and says why. B's raw backup/restore still works.
- **Firmware dependency warnings.** The robot firmware assumes 3D mode and bidirectional DShot
  (`esc.h`, `lib/erpm_decode`). Fields tagged `firmware_dependency` (direction/3D, anything that
  turns off telemetry or changes the DShot protocol) get a confirm dialog that names what breaks.
- **Load `.ini`.** Import a BLHeliSuite32 `.ini` by key name, so existing setups and
  `mr_stabs_mk2_esc.ini` load into the form. **Save `.ini`** exports the same format, keeping
  `firmware/config/blheli/` as the human-readable record.

Tests:

- `firmware/scripts/test_blheli_layout.py` (host, venv): decode the step 0 `.bin` files with the
  JSON table and assert every `.ini` value; encode then decode round-trips every field at its min,
  max and each option; the checksum, if any, matches every captured block.
- `lib/blheli_boot` gtests: CRC against captured frames, address guard accepts the block and rejects
  one byte either side, reply parser rejects bad CRC and short replies.
- Bench: change Rampup Power, Timing and Brake On Stop through the editor, verify each in
  BLHeliSuite32, drive, restore.

## Step 5: docs and cleanup

- README: replace "Configuring ESCs via USB Passthrough" (stale since `4ab45d61`) with the tuning
  mode flow; fix the ESC pin mapping table; note that the standalone passthrough is still the way
  to flash ESC firmware, which this plan does not cover.
- `docs/esc_tuning.md` (or a README section): the field table with what each setting does, taken
  from the BLHeli_32 manual, and a log of tuning changes with the bench or match data behind them.

## Out of scope

- Flashing ESC firmware. The address guard blocks it on purpose; use BLHeliSuite32 with the
  standalone passthrough.
- Any ESC firmware other than 32.90 / layout 48. A new version needs a new capture and table.
- Changing settings while armed or between matches without a bench.

## Next steps

1. Flash `firmware/BlHeli-Passthrough` to the Mk2 board and run the step 0 capture (Read Setup, then
   an unchanged Write Setup).
2. Write `blheli_capture.py` and commit the captures, the first `.bin` backups and
   `layout_32_90_r48.md`.
3. Map the fields (step 1) and check the table against `mr_stabs_mk2_esc.ini`.
4. Then build B, bench-validate it including the pulled-pack test, then C.
