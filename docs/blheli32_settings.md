# BLHeli_32 settings over the 4-way bootloader

Spec for reading and writing BLHeli_32 ESC settings without BLHeliSuite32. Derived on 2026-10-23
from Mr Stabs Mk2's two Flycolor Raptor 5 ESCs (BLHeli_32 32.90, layout 48, STM32G071x6) and
BLHeliSuite32xl 1.0.4.4 for Linux, using usbmon captures, gdb watchpoints on BLHeliSuite's
settings model, disassembly, and a keyboard sweep of every slider in its UI.

- Library: `auto_battlebot/blheli32/` (`cipher.py`, `layout.py`, `fourway.py`, `esc.py`)
- CLI: `scripts/blheli32.py`
- Tests: `tests/python/test_blheli32.py`, against pages read from the robot
- Raw captures and decrypted pages: `firmware/config/blheli/captures/`
- Backups made with the CLI: `firmware/config/blheli/backups/`

## Usage

The robot must run `firmware/BlHeli-Passthrough`, powered, transmitter off.

```bash
venv/bin/python scripts/blheli32.py read                      # both ESCs
venv/bin/python scripts/blheli32.py backup firmware/config/blheli/backups
venv/bin/python scripts/blheli32.py fields                    # every field and range
venv/bin/python scripts/blheli32.py set BACKUP.json demag_compensation="Very High" \
    rampup_power=60 -o new.json                               # offline edit, validated
venv/bin/python scripts/blheli32.py set BACKUP.json music_notes="A5 1/8 D5 1/4" name=LEFT -o new.json
venv/bin/python scripts/blheli32.py write new.json --esc 0    # diff, confirm, write, verify
```

`write` refuses a file read from the other ESC (direction differs per ESC), refuses an edit whose
source no longer matches the ESC, saves the ESC's current settings next to the file, asks before
writing, and reads the page back to verify.

`set` takes raw numbers or the labels BLHeliSuite shows (`High`, `Auto`, `By RPM`, `Off`).
ESC 0 is pin A2, ESC 1 is pin A3 (the passthrough's pin order). Both ESCs read 338422 / 338408
good input frames, which identifies them.

## Transport

The PC talks to the passthrough at 115200 baud. BLHeliSuite first sends a burst of MSP queries,
then `MSP_SET_4WAY_IF` (`$M< 00 F5 F5`), after which the link speaks 4-way:

```
request:  2F cmd addr_hi addr_lo n params[n] crc_hi crc_lo
response: 2E cmd addr_hi addr_lo n params[n] ack crc_hi crc_lo
```

`n = 0` means 256 bytes. CRC is CRC16-XMODEM (poly 0x1021, init 0) over everything before it.
`ack = 0` is success.

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

## Next steps

1. Power-cycle after a beep-volume change and listen, to confirm the running firmware (not just
   the bootloader) picks up written settings.
2. Wire the library into the dashboard tuning mode in `docs/plans/esc_settings_dashboard_plan.md`:
   the ESP32 only needs to relay 4-way frames; decoding can stay in the browser or in Python.
