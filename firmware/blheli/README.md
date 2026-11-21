# BLHeli_32 settings over the 4-way bootloader

Spec for reading and writing BLHeli_32 ESC settings without BLHeliSuite32. Derived on 2026-10-23
from Mr Stabs Mk2's two Flycolor Raptor 5 ESCs (BLHeli_32 32.90, layout 48, STM32G071x6) and
BLHeliSuite32xl 1.0.4.4 for Linux, using usbmon captures, gdb watchpoints on BLHeliSuite's
settings model, disassembly, and a keyboard sweep of every slider in its UI.

- Library: `auto_battlebot/blheli32/` (`cipher.py`, `layout.py`, `fourway.py`, `esc.py`,
  `http_port.py`)
- CLI: `firmware/blheli/blheli32.py`
- Robot side: `firmware/mr_stabs_mk2` tuning mode (`lib/blheli_boot`, `src/esc_config.cpp`) and
  the diagnostics page's ESC SETTINGS panel (`web/esc.js`, `web/esc_panel.js`)
- Page field table: `firmware/mr_stabs_mk2/web/esc_layout.json`, written by
  `firmware/blheli/export_layout.py` from `layout.py` (a test fails while they differ)
- Tests: `auto_battlebot/blheli32/tests/test_blheli32.py`, against pages read from the robot
- Captured pages the tests read: `firmware/blheli/captures/` (other `.bin` dumps there are
  gitignored)
- `BLHELISUITE_ESC1` in `auto_battlebot/blheli32/tests/test_blheli32.py` is what BLHeliSuite32xl showed for ESC 1 after Read Setup on 2026-10-23.
- `FakePassthrough` in `auto_battlebot/blheli32/tests/test_blheli32.py` answers 4-way requests from the captured ESC 1 data. It re-encrypts every read with fresh nonces, as the real ESC does. `drop_writes` acknowledges writes without storing them, like the dry-run passthrough build.
- Backups made with the CLI: `firmware/blheli/backups/` (gitignored, local to each machine)

## Usage

Over USB the robot runs `firmware/BlHeli-Passthrough`. Over WiFi (`--url`) it runs the Mk2
firmware, and the CLI enters tuning mode for the command and leaves it after. Either way: powered,
transmitter disarmed.

```bash
venv/bin/python firmware/blheli/blheli32.py read                    # both ESCs
venv/bin/python firmware/blheli/blheli32.py backup firmware/blheli/backups
venv/bin/python firmware/blheli/blheli32.py fields                  # every field and range
venv/bin/python firmware/blheli/blheli32.py set BACKUP.json demag_compensation="Very High" \
    rampup_power=60 -o new.json                                     # offline edit, validated
venv/bin/python firmware/blheli/blheli32.py set BACKUP.json music_notes="A5 1/8 D5 1/4" name=LEFT -o new.json
venv/bin/python firmware/blheli/blheli32.py write new.json --esc 0  # diff, confirm, write, verify
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local read  # over WiFi
```

`write` refuses a file read from the other ESC (direction differs per ESC), refuses an edit whose
source no longer matches the ESC, saves the ESC's current settings next to the file, asks before
writing, and reads the page back to verify.

`set` takes raw numbers or the labels BLHeliSuite shows (`High`, `Auto`, `By RPM`, `Off`).
ESC 0 is pin A2 (right), ESC 1 is pin A3 (left), the passthrough's pin order. Both ESCs read
338422 / 338408 good input frames, which identifies them.

## Transport

The PC talks to the passthrough at 115200 baud. BLHeliSuite first sends a burst of MSP queries,
then `MSP_SET_4WAY_IF` (`$M< 00 F5 F5`), after which the link speaks 4-way:

```
request:  2F cmd addr_hi addr_lo n params[n] crc_hi crc_lo
response: 2E cmd addr_hi addr_lo n params[n] ack crc_hi crc_lo
```

`n = 0` means 256 bytes. CRC is CRC16-XMODEM (poly 0x1021, init 0) over everything before it.
`ack = 0` is success.

The Mk2 firmware's tuning mode relays the same frames over HTTP, with no MSP step: `POST
/esc/4way frame=<hex>` queues one request and returns an id, and `GET /esc/4way` reports it
`pending` or `done` with the response frame in hex. `POST /esc/tuning enter=1|0` enters and leaves
tuning mode, and `GET /esc/status` reports it. The relay answers `InterfaceGetName` with
`MrStabsMk2` and refuses (ack `0x09`) every `DeviceWrite` that is not 256 bytes at `0xF800`, and
every `DevicePageErase` and `DeviceEraseAll`.

The relay differs from the passthrough where the robot showed it had to (2026-10-23, details in
`docs/plans/esc_settings_dashboard_plan.md` (deleted 2026-11-18, in git history), "Bench findings"):

- Tuning mode stops DShot and holds both lines high, and that alone puts the ESCs in their
  bootloader. `DeviceReset` therefore only selects the ESC. A real restart (the passthrough's
  `RestartBootloader` and 300 ms low) boots the application, and the relay uses it only when a
  handshake gets no answer, and when tuning mode ends.
- The link is a hardware UART in open-drain half duplex, not EspSoftwareSerial, which lost bits
  inside the robot firmware.
- Replies are checked from their end, the handshake must return all 9 bytes
  (`34 37 31 6d 47 06 07 04 30`), reads retry, and a write stops before `ProgFlash` if the data
  was not acknowledged.
- `GET /esc/status` `failure` says why the latest exchange failed, with the bytes received.

| Cmd | Name | Params |
|---|---|---|
| 0x31 | ProtocolGetVersion | - (reply 0x6C) |
| 0x32 | InterfaceGetName | - (reply `m4wFCIntf`) |
| 0x33 | InterfaceGetVersion | - (reply 0xC805) |
| 0x34 | InterfaceExit | - |
| 0x35 | DeviceReset | ESC index |
| 0x37 | DeviceInitFlash | ESC index (reply: 4 signature bytes, `06 47 6d 04` here) |
| 0x39 | DevicePageErase | page number (passthrough maps it to address `page << 10`) |
| 0x3A | DeviceRead | length (0 = 256), address in header |
| 0x3B | DeviceWrite | data, address in header |
| 0x3F | InterfaceSetMode | 0x04 = ARM BLB (BLHeli_32 bootloader) |

BLHeliSuite's Read Setup, per ESC: `DeviceReset(n)`, `DeviceInitFlash(n)`, read 256 at `0xF800`,
16 at `0xFA00`, 16 at `0xFD90`, `DeviceReset(n)`. The bootloader link to the ESC is 19200 8N1
half duplex on the signal wire, CRC16 poly 0xA001 (`firmware/BlHeli-Passthrough/ESP32/ESC_Serial.cpp`).

## Flash regions read

| Address | Size | Encrypted | Contents |
|---|---|---|---|
| `0xF800` | 256 | yes | Settings page (below). 0x7C00 on 32 KB parts |
| `0xFA00` | 16 | no | Byte 3: input protocol (5 seen on DShot; other codes unmapped). Bytes 4-7: good input frames, u32 LE. Bytes 8-11: bad frames, u32 LE |
| `0xFD90` | 16 | no | Two u16 LE (88, 100 on ESC 0; 120, 90 on ESC 1), then ASCII `P10L52 `, then `FF`. Meaning unknown; read-only as far as BLHeliSuite goes |

## Settings page encryption

- 32 blocks of 8 bytes. Each block is XTEA, 32 rounds, delta `0x9E3779B9`.
- Key, four u32 LE: `81fbc617 9e81c901 29a1fa54 318234b4`. In BLHeliSuite32xl 1.0.4.4 for Linux at
  VA `0xdd4e2c`; the decrypt routine is at `0x822190`.
- Non-standard round: the tweak `t` is added inside each round key term:
  `v1 -= mix(v0) ^ (sum + k[(sum >> 11) & 3] + t)`, then
  `v0 -= mix(v1) ^ (sum + k[sum & 3] + t)`, with `mix(v) = ((v << 4) ^ (v >> 5)) + v`.
- `t = 0xF800 + 8 * block_index`. The base is a constant in BLHeliSuite (`0x7f0fd0`), not the
  address it reads.
- A decrypted block is 2 nonce bytes then 6 payload bytes. The ESC draws a new nonce on every
  read: byte 0 starts random and rises by 8 per block, byte 1 starts random and falls by 8.
  BLHeliSuite writes plain random nonces. The nonce carries no check, so the same settings never
  give the same ciphertext twice. Validate a decode by the firmware and layout bytes.

The binary holds five more keys (`0xdd4dfc`, `0xdd4e0c`, `0xdd4e1c`, `0xdd4e3c`, `0xdd4e4c`) and
a plain TEA routine at `0x822530`. BLHeliSuite selects among them by MCU type; only `0xdd4e2c`
decodes these ESCs.

## Settings payload (192 bytes, layout 48)

Offsets follow BLHeliSuite's `Eep_*` record order. Encodings come from stepping each slider in
BLHeliSuite and reading the model byte next to the label it displayed. "Unverified" means the
byte's role or encoding is not confirmed on hardware.

| Off | Width | Field | Raw encoding | Mr Stabs now |
|---|---|---|---|---|
| 0 | 1 | FW main revision | read-only | 32 |
| 1 | 1 | FW sub revision | read-only | 90 |
| 2 | 1 | Layout revision | read-only; refuse to edit if not 48 | 48 |
| 3 | 1 | Motor direction | 1 Normal, 2 Reversed, 3 Bidirectional 3D, 4 Bidirectional 3D Rev., 5 Bidirectional Soft, 6 Bidirectional Soft Rev. | 3 / 4 |
| 4 | 1 | Rampup power | 3-150 % | 100 |
| 5 | 1 | PWM frequency low | kHz, 16 to PWM high | 24 |
| 6 | 1 | Motor timing | 0 Auto, 1-31 deg (15/16 deg units per the manual) | 16 |
| 7 | 1 | Demag compensation | 1 Off, 2 Low, 3 High, 4 Very High | 3 |
| 8 | 2 | Minimum throttle | us, u16 LE; BLHeliSuite enforces min < center < max, 900-2100. No effect on DShot | 1000 |
| 10 | 2 | Center throttle | us; bidirectional PWM only | 1500 |
| 12 | 2 | Maximum throttle | us | 2000 |
| 14 | 1 | Throttle cal enable | 0 / 1 | 1 |
| 15 | 1 | Temperature protection | 0 Off, 1-255 C | 140 |
| 16 | 1 | Low voltage protection | 0 Off, n = 2.4 + 0.1n V per cell (1-16 = 2.5-4.0 V) | 0 |
| 17 | 1 | Current protection | 0xFF on ESCs without current sense. BLHeliSuite hides it there, so 0 Off / else A is from the manual, not observed | 255 |
| 18 | 1 | Low RPM power protect | 0 Off, 1 On, 2 On Adaptive | 0 |
| 19 | 1 | Brake on stop | 0 Off, 1-100 % | 100 |
| 20 | 1 | Startup beep volume | 0 Off, 1-254 | 254 |
| 21 | 1 | Locator beep volume (BLHeliSuite: ".../Signal Volume"); also DShot tone volume | 0 Off, 1-255 | 80 |
| 22 | 2 | Locator beep delay after zero throttle | seconds, 0 Off, 1-3600 | 600 |
| 24 | 1 | LED control | 0 Off, 3 On (one-LED ESC). Per-LED bits on 4-LED ESCs not observed | 3 |
| 25 | 1 | Maximum acceleration | 0 Maximum, n = 0.1n %/ms (1-255) | 0 |
| 26 | 1 | Non damped mode | 0 / 1 | 0 |
| 27 | 1 | Current sense calibration | hidden by BLHeliSuite without current sense; encoding not observed | 100 |
| 28 | 1 | Music note config | high nibble = general length, low nibble = interval (0x35 = length 3, interval 5) | 0x10 |
| 29 | 1 | Sine modulation mode | 0 / 1. Disables variable PWM frequency | 0 |
| 30 | 1 | Auto telemetry | 0 / 1 (every 32 ms) | 1 |
| 31 | 1 | Stall protection | 0 Relaxed, 1 Normal | 0 |
| 32 | 1 | SBUS channel | 0-16 = CH 0-16, 17 Off | 17 |
| 33 | 1 | S.PORT physical ID | 0 Off, 1-28 | 0 |
| 34 | 1 | PWM frequency high | kHz, PWM low to 128; 129 = By RPM | 24 |
| 35-47 | 13 | Filler, flash counter | reserved, `FF` here; carried through unchanged | |
| 48 | 1 | HW voltage sense capable | read-only | 0 |
| 49 | 1 | HW current sense capable | read-only; 255 = none | 255 |
| 50-53 | 4 | HW LED capable 0-3 | read-only | 1, 0, 0, 0 |
| 54 | 1 | HW PWM freq min | read-only, kHz | 16 |
| 55 | 1 | HW PWM freq max | read-only, kHz | 128 |
| 56-61 | 6 | Filler 2 | reserved, `FF` | |
| 62 | 1 | S.PORT capable | read-only | 1 |
| 63 | 1 | Non damped capable | read-only | 1 |
| 64 | 32 | ESC layout | ASCII, `#Flycolor_Raptor_5#` space padded | |
| 96 | 32 | ESC MCU | ASCII, `#BLHeli_32*STM32G071x6#` | |
| 128 | 16 | Name | ASCII user name, blank here | |
| 144 | 48 | Music notes | one byte per note, `FF` ends the list (below) | `A5 1/8`, `D5 1/8` |

The two ESCs' payloads differ at byte 3 (direction 3 vs 4) and byte 144 (startup note) only.

### Music notes

`byte = length << 6 | (octave - 4) * 16 + semitone`, with length 0 = 1/8, 1 = 1/4, 2 = 1/2,
3 = 1/1, semitone 0 = C to 11 = B, octave 4-7. BLHeliSuite rejects 1/16. Its script
`C5 1/8 C#5 1/4 D6 1/2 C4 1/2 B5 1/1 A5 1/8` wrote `10 51 A2 80 DB 19 FF...`. Semitone values
12-15 (pauses, if any) were not observed.

BLHeliSuite keeps its own in-memory copy with a different order past byte 34 (PWM low and high
sit at model bytes 34 and 35, the HW fields at 37-46), so its model is not a page image.

## What each setting does

From the BLHeli_32 manual and the field notes in `layout.py` (the page shows those as tooltips).
The last column is what the setting means on Mr Stabs, which drives over bidirectional DShot300 in
3D mode and reads eRPM back on the same wire.

| Field | What it does | On Mr Stabs |
|---|---|---|
| Direction | Normal or reversed rotation, or bidirectional, where the throttle range splits at center into reverse and forward. "Soft" variants soften the direction change | Must stay Bidirectional 3D (3) or 3D Rev. (4). The firmware maps commands as 3D DShot (`esc.h`), and the two ESCs differ so both wheels drive forward |
| Rampup power | Power limit at startup and low rpm, and on direction reversal. Higher gives more punch off the line and more risk of a desync | The main launch-feel setting for a pushing robot |
| PWM frequency low / high | Motor switching frequency at low and high throttle; high can follow rpm (By RPM). Higher is smoother and quieter and costs more switching loss in the ESC | Limited to this ESC's 16-128 kHz |
| Motor timing | Commutation advance. Higher gives more top speed and current at lower efficiency; Auto picks it from the motor | 16 now |
| Demag compensation | Cuts power before a commutation when the winding demagnetizes slowly, which protects against stalls and desyncs under hard load. Higher is safer and can cost some power | High now |
| Min / center / max throttle | Pulse widths for PWM input | No effect on DShot |
| Throttle cal enable | Stick calibration at power-up on PWM input | No effect on DShot |
| Temperature protection | ESC temperature where it starts limiting power; Off disables it | 140 C now |
| Low voltage protection | Per-cell voltage where it starts limiting power | Off; the pack is watched by the INA238 |
| Current protection | Current limit, on ESCs with a current sensor | These ESCs have none (reads 255); carried through unchanged |
| Low RPM power protect | Limits power at low rpm. Off gives full power at low rpm, which low-kV or low-voltage setups can need; Adaptive scales the limit with kV and voltage | Off now |
| Brake on stop | Braking force at zero throttle | 100 % now: the wheels hold at stop |
| Startup beep volume | Volume of the power-up tune | |
| Locator beep volume / delay (`beacon_volume`, `beacon_delay`) | Beeps once the throttle has sat at zero for the delay, to find a downed model; the volume also sets the DShot beep command's. Loud beeps heat the motor | 80, after 600 s |
| LED control | ESC LED on or off | |
| Maximum acceleration | Slew limit on power, in %/ms; Maximum means none | Unlimited now. A softer launch is possible here instead of in the robot firmware |
| Non damped mode | Turns off damped light (active freewheeling and regenerative braking) | Off now |
| Current sense cal | Current sensor gain trim | No sensor; carried through |
| Music note config / notes | The startup tune: note length and pause, and up to 48 notes | One note each, A5 on ESC 0 and D5 on ESC 1, which tells them apart by ear |
| Sine modulation mode | Sine power modulation, smoother at low speed. Disables variable PWM frequency | Off now |
| Auto telemetry | Serial telemetry every 32 ms on the telemetry wire | Not used; the firmware reads eRPM over bidirectional DShot |
| Stall protection | How the ESC reacts to a stalled motor, Normal or Relaxed (the more tolerant) | Relaxed now; the robot's own stall guard (`lib/motor_guard`) limits current from the firmware side |
| SBUS channel / S.Port physical ID | Direct receiver input and FrSky telemetry | Not used |

## Write path

BLHeliSuite's Write Setup, captured against a passthrough built to acknowledge writes without
forwarding them (so ESC flash was never touched):

1. `DeviceReset(n)`, `DeviceInitFlash(n)`
2. Read `0xF800` (256), `0xFA00` (16), `0xFD90` (16), with `InterfaceTestAlive` between rounds
3. One `DeviceWrite` at `0xF800` with the whole 256-byte encrypted page. No `DevicePageErase`;
   the bootloader handles the erase
4. Read the three regions back and compare
5. `DeviceReset(n)`

Every payload byte it does not edit, reserved bytes included, goes back exactly as read. With
PWM low 16 and high 48 set in the UI, the written page differed from the read one at payload
bytes 5 and 34 only. `Session.write` does the same: read and check the current page, one
256-byte `DeviceWrite`, read back and compare, raise `VerifyError` on any difference. Against the
dry-run passthrough it sent the page and caught the dropped write at byte 5.

Hardware round trip, 2026-10-23, ESC 0: startup beep volume 254 -> 200 written and verified. A
new session (fresh bootloader entry) read payload byte 20 changed and nothing else; the `0xFA00`
frame counter (338422) and the `0xFD90` block were intact and ESC 1 was untouched. Restoring the
backup wrote 254 back and verified. Not yet checked: a pack power cycle with the startup beep
heard at the new volume.

## Not observable on this hardware

- Current protection (17) and current sense calibration (27): BLHeliSuite hides both on ESCs
  without a current sensor. The tool carries them through and validates only 0-255.
- LED bits for ESCs with more than one LED (24).
- Music pauses, if the format has them (semitones 12-15).

## Gaps versus BLHeliSuite

What BLHeliSuite32xl 1.0.4.4 does that this tool does not, as of 2026-10-23.

Not supported:

- Flashing or verifying ESC firmware (Flash BLHeli, Verify BLHeli). Settings only; a bad
  firmware write can brick an ESC. Use BLHeliSuite through the USB passthrough for that.
- Spinning motors from the PC (the Motors tab). Left out on purpose.
- Any ESC other than BLHeli_32 32.x layout 48 with the `0xdd4e2c` key. `check_layout` refuses
  other firmware and layout revisions. See "Supporting other ESCs" below.
- Interfaces other than `firmware/BlHeli-Passthrough` over USB and the Mk2 tuning mode over WiFi.
  BLHeliSuite also connects through flight controllers, Arduino 4-way adapters and USB linkers.

Supported, less conveniently:

- The CLI works on one ESC at a time and refuses a file read from the other ESC. BLHeliSuite
  edits all ESCs together and copies master to slave. The dashboard panel copies field by field.
- No BLHeliSuite `.ini` import or export; backups are the tool's own JSON.
- The startup tune is text (`A5 1/8 D5 1/4`) with no playback preview. General length and
  interval are one raw byte, `music_note_config` (high nibble length, low nibble interval).
- Command line only: field names come from `fields`, not labelled sliders.

Fields the tool carries but does not fully understand are listed under "Not observable on this
hardware".

## Supporting other ESCs

Ordered from least to most work.

1. **Other BLHeli_32 ESCs on 32.x, layout 48.** Three things can differ:
   - Key and cipher. BLHeliSuite holds six keys (`0xdd4dfc`, `0xdd4e0c`, `0xdd4e1c`, `0xdd4e2c`,
     `0xdd4e3c`, `0xdd4e4c`) and two decrypt routines (XTEA at `0x822190`, TEA at `0x822530`).
     `0x821f10` picks one from the chip ID in the `DeviceInitFlash` reply. Reading that function
     is static analysis, a few hours, no hardware.
   - Page address. 32 KB parts keep settings at `0x7C00`; the tweak base follows (`0x7f0fd0` in
     BLHeliSuite). `cipher.py` already takes the tweak as an argument; `esc.py` does not choose it.
   - Fields. The `Eep_Hw_*` bytes (48-63) report current sense, LED count, PWM range and S.PORT.
     The tool could hide fields the hardware lacks, as BLHeliSuite does. Current protection,
     current sense calibration and multi-LED bits still need an ESC that has them to confirm.
2. **Other BLHeli_32 firmware or layout revisions.** Offsets may move. Each needs one ESC on that
   firmware and the procedure used here: Read Setup capture, slider sweep, dry-run Write Setup
   capture. About an hour per revision with the existing scripts.
3. **BLHeli_S, Bluejay, AM32.** Their settings are not encrypted, and the layouts are public in
   the esc-configurator project. BLHeli_S and Bluejay (SiLabs) use a different 4-way mode and
   bootloader command set: the passthrough has code for it, the Python client does not. AM32
   uses an ARM bootloader like BLHeli_32, so it is mostly a new field table. Roughly a day per
   family plus a test ESC.

Each case needs an ESC of that type on the bench to verify a real write. Analysis alone does not
cover it.

## Tuning log

One row per settings change that reached a robot, with the evidence behind it: a bench capture,
a match recording, or a driver note. Read-back-and-restore tests do not count.

| Date | ESC | Field | Old → new | Why | Evidence |
|---|---|---|---|---|---|
| 2026-10-23 | 0 (right) | `music_notes` | `A5 1/1 A5 1/1 A5 1/1` → `F5 1/8 A5 1/8 C6 1/8 F6 1/2` | A nicer startup tune: F major, played with ESC 1 as a chord | Written and verified through the page over the USB dashboard; a CLI read differed from `backups/esc0_2026-10-23_205816.json` in payload bytes 144-147 only. Not yet heard after a power cycle |
| 2026-10-23 | 1 (left) | `music_notes` | `D5 1/1 D5 1/1 D5 1/1` → `C5 1/8 F5 1/8 A5 1/8 C6 1/2` | Harmony under ESC 0's tune | Same, against `backups/esc1_2026-10-23_205816.json` |

## Python module notes

Notes on the modules under `auto_battlebot/blheli32/`.

### `blheli32/cipher.py`

BLHeli_32 settings page encryption.

The ESC stores its 256-byte settings page at 0xF800 and returns it encrypted over the bootloader.
Each 8-byte block is XTEA (32 rounds) with a tweak added into every round key: the page base
BLHeliSuite passes (0xF800 on 64 KB parts, 0x7C00 on 32 KB parts) plus 8 per block. The key is
BLHeliSuite32xl 1.0.4.4's, at VA 0xdd4e2c in the Linux binary.

A decrypted block is a 2-byte nonce and 6 payload bytes. The ESC draws a fresh nonce for every
block on every read, so two reads of the same settings never match as ciphertext. The nonce
carries no check: a wrong key shows up as a payload whose firmware and layout bytes are wrong
(layout.check_layout), and transport errors are caught by the 4-way CRC. 32 blocks carry 192
payload bytes, which is the settings record `layout.py` describes.

### `blheli32/esc.py`

Read and write BLHeli_32 settings through the 4-way passthrough.

The read sequence is BLHeliSuite32xl's, from a usbmon capture: per ESC, DeviceReset(n),
DeviceInitFlash(n), read 256 bytes at 0xF800 (settings), 16 at 0xFA00 (input frame counters),
16 at 0xFD90, then DeviceReset(n) to release it.

Write Setup, from a capture against a passthrough that acknowledged writes without forwarding
them: DeviceReset(n), DeviceInitFlash(n), read the three regions, one DeviceWrite of the
256-byte encrypted page at 0xF800 (no DevicePageErase; the bootloader erases), read the three
regions back to verify, DeviceReset(n). Every payload byte BLHeliSuite does not edit, reserved
ones included, is written back exactly as read.

#### `EscSnapshot`

0xFA00 stats: byte 3 is the detected input protocol (5 on DShot; other codes unmapped),
then good and bad input frame counters as little-endian u32.

### `blheli32/fourway.py`

BLHeli 4-way interface client over a serial passthrough (firmware/BlHeli-Passthrough).

Framing, from the passthrough's Check_4Way and a usbmon capture of BLHeliSuite32xl 1.0.4.4:

    request:  0x2F cmd addr_hi addr_lo n params[n or 256] crc_hi crc_lo
    response: 0x2E cmd addr_hi addr_lo n params[n or 256] ack crc_hi crc_lo

n = 0 means 256 bytes. CRC is CRC16-XMODEM over everything before it. The serial link first
switches from MSP to 4-way with MSP_SET_4WAY_IF (0xF5), as BLHeliSuite does. The Mk2 robot's
tuning-mode relay (http_port.HttpPort) speaks 4-way only, so it skips that step.

### `blheli32/http_port.py`

4-way transport over the Mk2 robot's ESC tuning mode (firmware/mr_stabs_mk2, /esc/ routes).

HttpPort satisfies fourway.SerialPort, so esc.Session runs over WiFi unchanged: write() posts one
4-way request frame and waits for the robot to relay it, and read() serves the response bytes.
Pass msp=False to Session; the relay speaks 4-way from the start.

    POST /esc/tuning  enter=1|0      enter or leave tuning mode (409 while armed)
    GET  /esc/status                 {"state": "combat"|"tuning"|"exiting", "error": ...}
    POST /esc/4way    frame=<hex>    queue one request frame -> {"id": n}
    GET  /esc/4way                   {"id": n, "state": "none"|"pending"|"done", "frame": <hex>}

### `blheli32/layout.py`

BLHeli_32 settings record: the 192-byte payload of the 0xF800 page, firmware 32.x layout 48.

Offsets follow BLHeliSuite32xl 1.0.4.4's `Eep_*` record order, checked against two decrypted
Flycolor Raptor 5 pages (firmware/blheli/captures/). Encodings come from stepping every
BLHeliSuite slider and reading back its model byte and label (firmware/blheli/README.md).
`verified=False` marks fields whose offset or encoding has not been confirmed on hardware.

#### `NOTE_NAMES`

Startup music. One byte per note: bits 7-6 length (0 = 1/8, 1 = 1/4, 2 = 1/2, 3 = 1/1),
bits 5-0 = (octave - 4) * 16 + semitone. 0xFF ends the list. Decoded from BLHeliSuite's
Write Setup for "C5 1/8 C#5 1/4 D6 1/2 C4 1/2 B5 1/1 A5 1/8" -> 10 51 A2 80 DB 19.

## Next steps

1. Power-cycle the pack and listen for the F major tune from both ESCs together. On 2026-10-23
   ESC 0 had played its earlier `A5 1/1 A5 1/1 A5 1/1`, so the running firmware does pick up
   written settings; ESC 1's tune has not been heard yet. Startup beep volume is 40 on ESC 0 and
   254 on ESC 1, so the right ESC's half of the chord will be quiet until those match.
2. Run the WiFi relay's bench checklist in `docs/plans/esc_settings_dashboard_plan.md` (deleted 2026-11-18, in git history) (step 2).

## Notes moved from code comments

### tuning mode over WiFi

- `mr_stabs_mk2/include/esc_config.h`: ESC tuning mode. Stops DShot, hands both ESC signal wires to the BLHeli bootloader link, and relays 4-way frames posted over HTTP. The robot never decodes settings: the diagnostics page and firmware/blheli/blheli32.py --url do. ESC indices follow the passthrough: 0 = A2, the right ESC; 1 = A3, the left ESC.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: Tuning mode owns the pins; leaving it is POST /esc/tuning.
- `mr_stabs_mk2/src/diagnostics_server.cpp`: The 4-way relay for tuning mode. The loop runs one frame at a time: POST queues it and returns an id, GET reports it pending or done.
- `mr_stabs_mk2/web/esc_panel.js`: backupJson decodes the settings; a page from another layout gets an empty decode instead.
- `mr_stabs_mk2/web/esc_panel.js`: Show what the ESC holds now; the edits stay in the form.

### tuning mode

- `mr_stabs_mk2/lib/DShot/DShot.h`: Uninstalls both RMT channels and hands the pad back as a plain GPIO with nothing routed to it, so tuning mode can talk to the ESC bootloader on the same wire. begin() works again.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: Tuning mode puts the ESCs in their bootloader by stopping DShot, so the handshake normally answers at once. If it does not, the ESC is restarted the passthrough's way and given REBOOT_SETTLE_MS of quiet high line before the handshake is tried again.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: Betaflight's sequence: line low 300 ms, then high. No reply; drain whatever comes.
- `mr_stabs_mk2/src/esc_config.cpp`: Drives both signal lines high. A BLHeli ESC that has lost its signal enters the bootloader only if the line reads high when it checks, so neither line is left floating on a pull-up.
- `mr_stabs_mk2/src/esc_config.cpp`: Frames wait out SETTLE_MS: the ESCs need a quiet high line to enter the bootloader.

### 4-way protocol

- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: The passthrough's DeviceInitFlash handshake. It carries its own CRC, so it goes out raw.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: The passthrough's InterfaceGetVersion, 20.0.05.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: The passthrough's retry count for the steps of a write.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: a read changes nothing, so it can repeat
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: Answered OK whatever the ESC says, as the passthrough does.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.h`: 4-way framing: request 2F cmd addr_hi addr_lo n params[n] crc_hi crc_lo, response 2E cmd addr_hi addr_lo n params[n] ack crc_hi crc_lo. n = 0 means 256. CRC16-XMODEM.

### bootloader link

- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.cpp`: Data, CRC low and high, result. 256 bytes take 135 ms on the wire at 19200.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.h`: Bootloader link: 19200 8N1 half duplex on the signal wire, CRC16 poly 0xA001 (reflected, init 0), sent low byte first. Replies end in a result byte.
- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.h`: Reads until expect bytes have arrived (0: no length known), a 10 ms gap after the first byte, max bytes, or nothing at all for first_ms. Returns the count read.
- `mr_stabs_mk2/src/esc_config.cpp`: The bootloader link on the ESC signal wires: 19200 8N1, half duplex, one wire per ESC. The passthrough (firmware/BlHeli-Passthrough/ESP32/ESC_Serial.cpp) bit-bangs this with EspSoftwareSerial. Inside this firmware that lost bits: 40 of 72 reads on 2026-10-23 needed a retry (bad CRCs, a 0x30 ack read as 0xfe), so the link uses UART2 instead. The GPIO matrix routes the UART's TX and RX to the selected ESC's pin, set open drain with the pull-up, so the ESC can pull the line low to answer. The receiver hears every byte sent, and receive() drops that echo.
- `mr_stabs_mk2/src/esc_config.cpp`: Our own bytes come back first. 256 of them take 135 ms on the wire.
- `mr_stabs_mk2/src/esc_config.cpp`: Park the receiver on a constant high input before the driver goes.
- `mr_stabs_mk2/test/test_blheli_boot/test_blheli_boot.cpp`: n = 0 means 256 parameter bytes.

### Transport

- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.h`: BLHeli 4-way interface server and BLHeli_32 bootloader client. Tuning mode relays 4-way request frames from HTTP (the diagnostics page, or firmware/blheli/blheli32.py --url) to the ESC signal wires. Ported from firmware/BlHeli-Passthrough/ESP32/4Way.cpp and ESC_Serial.cpp, minus MSP, with reads that end on the expected length instead of fixed delays, and with an address guard. Protocol and settings layout: firmware/blheli/README.md.
- `mr_stabs_mk2/src/esc_config.cpp`: Anything still in the receiver belongs to an earlier exchange.
- `mr_stabs_mk2/src/esc_config.cpp`: the next select() hands the pin back to the UART

### Write path

- `mr_stabs_mk2/lib/blheli_boot/blheli_boot.h`: The settings page is the only flash a write may touch. BLHeliSuite's Write Setup is one 256-byte DeviceWrite here and no page erase, so anything else is refused: a bad page can corrupt settings (restored from a backup) but never the ESC's application or bootloader.

### captured bootloader exchanges

- `mr_stabs_mk2/test/test_blheli_boot/test_blheli_boot.cpp`: Captured on the robot: a handshake answered while the ESC was booting, then a clean one.
- `mr_stabs_mk2/test/test_blheli_boot/test_blheli_boot.cpp`: Captured on the robot: the address ack arrived as fe 30.

### Stall protection row

- `mr_stabs_mk2/test/test_motor_guard/test_motor_guard.cpp`: BLHeli restarts a reversed motor at a fraction of the commanded duty. The model alone overestimates; the shunt bound must keep the guard from cutting power.

### General

- `mr_stabs_mk2/web/esc.js`: XTEA, 32 rounds, with a tweak added into each round key term: 0xF800 + 8 per block. Each decrypted block is a 2-byte nonce and 6 payload bytes.
- `mr_stabs_mk2/web/esc_panel.js`: Left to right as seen from behind the robot. ESC numbers follow the passthrough.

### encryption

- `mr_stabs_mk2/web/esc.js`: Random nonces per block, as BLHeliSuite's Write Setup sends.

### captured ESC pages

- `mr_stabs_mk2/web/esc.test.mjs`: What BLHeliSuite32xl showed for ESC 1 after Read Setup on 2026-10-23.
- `mr_stabs_mk2/web/esc.test.mjs`: BLHeliSuite's own Write Setup with PWM low 16 / high 48 changed bytes 5 and 34 only.

### dashboard ESC panel

- `mr_stabs_mk2/web/esc_panel.js`: Every read and write lands here, in this browser only: nothing is stored on the robot, and nothing downloads until DOWNLOAD CURRENT or DOWNLOAD HISTORY. localStorage is per origin, so mr-stabs.local and 192.168.4.1 keep separate histories.

### xlg log format section

- `parse_blheli_log.py`: Extract length-prefixed text records from xlg binary data. The xlg format embeds strings after 0xFF delimiters as: FF [00|80] [00|FF] FF <4-byte LE length> <string bytes>
