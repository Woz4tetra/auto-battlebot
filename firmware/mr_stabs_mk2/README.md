# MR STABS MK2 Firmware

ESP32-S3 firmware for the MR STABS battlebot. Runs on an Adafruit QT Py ESP32-S3.

## Modes

**Combat mode** (default): DShot300 motor control via the CRSF radio link. Differential drive
mixing, IMU-based flip detection, upside-down compensation. LED shows a rainbow cycle.

**Tuning mode**: motors stop and the ESC signal wires carry the BLHeli_32 bootloader protocol, so
the diagnostics page or `firmware/blheli/blheli32.py --url` can read and write ESC settings. LED
shows a blue breathing pulse. Entered from the page's ESC SETTINGS panel or the CLI, refused while
armed or while ESP-NOW drives. The page's EXIT button or the transmitter button leaves it, after
2 s of zero-throttle frames to wake the ESCs. See "Configuring ESCs" below.

### Flip switch and steering

| Flip switch | Upside down | Turn stick |
|---|---|---|
| UP | forced on | passes straight to the wheels, 1% deadband |
| MIDDLE | off | passes straight to the wheels, 1% deadband |
| DOWN (power-on default) | from the BNO055 | commands a yaw rate, closed on the gyro |

In DOWN, `lib/yaw_control` steers. The turn stick asks for a yaw rate (2000 deg/s at full
stick); with it centered, heading hold asks for the rate that returns to the held heading. A
loop on the BNO055 gyro sets the left/right differential to get that rate, and while driving
tail-first its feedback rises up to 4x. Mr Stabs' center of mass sits ahead of the axle, so
driving tail-first is unstable and spins out without it, also in MIDDLE. Stick back drives
tail-first right side up and upside down alike, and the loop runs in both. Standing still
(under 5% throttle) with the turn stick centered, it idles and outputs nothing.

The yaw rate is the gyro projected onto the fused gravity vector, so it stays the world yaw
rate when the robot is tilted or inverted. The turn stick passes straight through (outside a
1% deadband) while the IMU is not sampling and when the gyro sign check fails. That check
compares the gyro with the change in heading on every fast-turning sample. The sensor panel on
the dashboard shows it: spin the robot by hand before driving and confirm the "agree" count
rises. `YAW_RATE_SIGN` in `include/updown_sensor.h` flips it.

The gains come from the closed-loop sim in `auto_battlebot/mujoco_sim`, whose
`firmware.py` mirrors this code. Its plant is not fit to recordings yet, so expect to tune.

## Hardware

| Component | Connection |
|---|---|
| Board | Adafruit QT Py ESP32-S3 (4MB Flash, 2MB PSRAM) |
| Left ESC | Pin A3, bidirectional DShot300: RMT TX channel 3, eRPM reply on RX channel 4 |
| Right ESC | Pin A2, bidirectional DShot300: RMT TX channel 2, eRPM reply on RX channel 5 |
| Radio RX | Crossfire Nano, UART on pins 17 (TX) / 18 (RX). Failsafe must be set to cut (no pulses): the firmware's only link failsafe is the CRSF channel timeout |
| Accelerometer | ADXL375, I2C |
| Pack voltage and current | INA238 at 0x45 on the main board, on the BNO055's I2C bus (`Wire1`) |

### INA238 pack voltage and current

The main board's INA238 sits on the high side of a 0.5 mohm shunt (R1) between the switched pack + and the ESC bus. The firmware reads bus voltage and shunt voltage; current is shunt voltage over the nominal 0.5 mohm, at 10 mA per count.

- VBUS is on the pack side of the shunt, so it reads the shunt's drop high: at most 35 mV in a 70 A stall.
- A0 and A1 are tied to 3.3 V, so the address is fixed at 0x45.
- The shunt stays at the reset-default ADCRANGE=0 (+-163.84 mV). ADCRANGE=1 is 4x finer, but a brownout clears it and current would then read 4x high with nothing to flag it.

The INA238 bus voltage is +-0.1% out of the box, so there is no calibration constant. Current accuracy is the shunt tolerance plus the pour resistance inside the Kelvin taps; check it once against a clamp meter. If the chip does not answer at boot, or reports a device ID other than INA238, `vbat` and `ibat` read `nan` and the robot runs normally.

### CRSF telemetry

While the radio link is up, the firmware sends two CRSF sensor frames back to the transmitter, alternating every 50 ms so each arrives at 10 Hz:

| Frame | EdgeTX sensors | Source |
|---|---|---|
| Battery (0x08) | `RxBt` volts, `Curr` amps | INA238. `Capa` and `Bat%` are sent as 0: capacity is not tracked yet |
| Attitude (0x1E) | `Yaw`, `Roll`, `Ptch` | BNO055 Euler angles. Yaw is heading wrapped to +-180 degrees |

After flashing, run Telemetry > Discover new sensors on the radio. A NaN reading (no INA238) goes out as 0.

## Stall protection

`lib/motor_guard` keeps a jammed drive motor from cooking itself. On 2026-10-21 a 5 s stall at full stick pulled 70 A and smoked a motor; nothing in the ESCs limits motor current or temperature. The derivation is in `docs/plans/motor_stall_protection_plan.md`.

- **Current.** Each ESC replies to every DShot frame with its eRPM (bidirectional DShot, `lib/DShot`, decoded by `lib/erpm_decode`). Speed gives each motor's model current `(d V - Ke w) / R`. The INA238 pack current caps it (a motor drawing power cannot take more than `I_pack / |d|`), which covers BLHeli holding duty back while it restarts a motor, and raises it through a shared gain `g` (1 to 1.5) when the shunt reads more than the model.
- **eRPM.** Both ESCs get their frames at the same time, and each receiver arms before its own frame and skips past it to the reply. Sent one after the other, the second frame landed on the first ESC's reply. The decoder measures the reply's bit length, since these ESCs reply about 11% faster than nominal.
- **Heat.** A one-node thermal model per motor integrates the estimated current. The state survives a watchdog or software reset (RTC memory) and keeps cooling while disarmed, so re-arming does not reset it.
- **Limit.** The limiter acts on current low-passed at 0.1 s, so the 30 ms of stall or plugging current in a launch or full-stick reversal does not cut power. Below 35 C rise, each motor may draw 15 A, about 4.5x the 3.3 A a 1 g launch needs. From 35 C to 50 C the allowance falls to the continuous current, about 5.4 A, which holds the motor at 50 C. One scale multiplies both wheel commands, so the robot keeps its curvature while folded back. When the jam clears, power returns over about 1 s.

The constants are placeholders. Step 0 of the plan (bench measurements of resistance, duty curve, and thermal constants) was deferred, so they lean toward limiting early: `R_cold` is 0.40 ohm against the MuJoCo fit's 0.453, and `dT_max` is 50 C rather than 70 C. A long wall push may fold back sooner than it needs to.

The MOTOR HEAT panel on the diagnostics page shows:

| Row | Meaning |
|---|---|
| Guard state | `NOMINAL`; `HOT` past 35 C with full power; `LIMITING` while the scale is under 100%; `OFF` (red) with no sensor for a motor |
| Sensors | `FUSED` is normal. `SHUNT ONLY` (an ESC's eRPM is missing or failing its CRC) and `ERPM ONLY` (no INA238) still protect. `NONE` turns the guard off |
| Drive scale | What both commands are multiplied by |
| Rise L / R | Modelled winding rise above ambient. Amber past 35 C, red at 50 C |
| Motor current L / R | Estimated winding current. At partial throttle it exceeds the pack current |
| Stalled wheel | A wheel commanded above 30% with eRPM at 0 for 100 ms. Telemetry only; heat drives the limit |
| Shunt gain g | Should sit between 0.9 and 1.25. Outside that for a whole session, a constant is wrong |
| eRPM decode errors | Replies that failed GCR or CRC, both ESCs, from boot |

Before a match, check that the panel reads `FUSED`.

## Building and Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
# Compile
./scripts/compile

# Upload via USB
./scripts/upload

# Upload over the SoftAP with ArduinoOTA (connect to MR-STABS WiFi first)
./scripts/ota_wifi

# Upload over ESP-NOW through the dongle on the ZED Box (robot disarmed)
./scripts/ota
# ...or through a dongle plugged into this machine
./scripts/ota --local /dev/ttyACM0

# Serial monitor
./scripts/monitor

# Host-side unit tests (test/, GoogleTest, no board needed)
./scripts/test
```

Code with unit tests lives in `lib/` so the `native` environment can build it without the
Arduino core; `pio test` does not compile `src/`. The heading-hold PID is in `lib/pid/`.

## OTA Updates

ESP-NOW is on (`ESPNOW_ENABLED` in `src/main.cpp`). With it off, update over the WiFi fallback
below or USB.

Firmware updates go over ESP-NOW through the dongle (`firmware/espnow_dongle`,
`docs/plans/espnow_link_plan.md`). `./scripts/ota` builds the image, copies it to the ZED Box, and
runs `scripts/espnow_ota.py` there, which stops `auto-battlebot` to take the dongle and starts it
again afterward. A transfer takes about 20 s for an 887 KB image.

- The robot refuses a transfer while armed, and arming mid-transfer aborts it. Any abort, or 5 s
  without a frame, leaves the old firmware running.
- **No rollback.** The Arduino bootloader is built without it, so a bad image stays installed.
  After a transfer the flasher waits for the new image to report that it has run 10 s and heard a
  dongle, and checks its MD5. If that never comes, recover over USB.
- USB flashing (`./scripts/upload`) is the recovery path, and needs no button: the firmware runs
  `Serial` on the chip's USB-Serial/JTAG port (`ARDUINO_USB_MODE=1`), which stays up when the
  firmware hangs, and esptool resets the chip into its ROM bootloader through it.

### WiFi

The robot either runs its own access point or joins a router, chosen on the diagnostics page
(see [WiFi settings](#wifi-settings)). Out of the box it is an access point:

- **SSID:** `MR-STABS`
- **Password:** `havocbots`
- **Address:** `192.168.4.1`
- **Channel:** `espnow_protocol::CHANNEL_OVERRIDE` (1), the channel ESP-NOW shares

Run `./scripts/ota_wifi` on that network. In client mode, pass the robot's router address or
`mr-stabs.local`: `./scripts/ota_wifi 192.168.50.40`.

## Configuring ESCs

Both drive ESCs run BLHeli_32 32.90 (layout 48). Their settings are read and written over the
signal wires, with no USB cable and no BLHeliSuite32. Protocol, field table, and CLI:
`firmware/blheli/README.md`. Plan and bench checklist: `docs/plans/esc_settings_dashboard_plan.md`.

Bench only: props off, wheels clear, transmitter disarmed.

### From the diagnostics page

1. Open the page, find ESC SETTINGS, press ENTER TUNING MODE.
2. READ BOTH. The first read of each ESC downloads a backup (`esc<N>_<time>.json`, the CLI's
   format).
3. Edit in the table. Amber rows differ between left and right; → and ← copy one value across.
   An outlined cell is edited and not yet written.
4. APPLY on one ESC, or APPLY BOTH, shows each change and any warning. WRITE saves the ESC's
   current settings as `esc<N>_before_write_<time>.json`, writes, and reads back to verify.
5. RESTORE loads a backup into the form for the same ESC, then the same review and write.
6. EXIT TUNING MODE.

### From the CLI over WiFi

```bash
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local read
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local backup firmware/blheli/backups
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local write new.json --esc 0
```

The CLI enters tuning mode for the command and leaves it after. Use `192.168.4.1` on the robot's
access point if mDNS does not resolve.

### Flashing ESC firmware

Tuning mode only writes the 256-byte settings page at `0xF800` and refuses every other write and
erase. To flash ESC firmware, flash `firmware/BlHeli-Passthrough` to the board and use
BLHeliSuite32 over USB, then flash this firmware back.

## ESC Pin Mapping

ESC numbers follow the passthrough's pin order, in the page, the CLI, and backup files.

| ESC | Motor | GPIO pin | RMT TX / RX |
|---|---|---|---|
| ESC 0 | Right | A2 | 2 / 5 |
| ESC 1 | Left | A3 | 3 / 4 |

## Diagnostics Dashboard

A live diagnostics web page, styled after the main web dashboard. It needs only a browser.

1. Join the robot's network: **MR-STABS** in access point mode, or the same router in client mode.
2. Open `http://192.168.4.1` (access point) or `http://mr-stabs.local` (client mode).

The page lives in `web/` as plain HTML, CSS, and JS with no build step of its own.
`scripts/embed_web.py` runs before every PlatformIO build, gzips `web/` into
`.pio/build/<env>/web_assets/web_assets.h`, and the firmware serves it from flash. Edit the files
in `web/` and rebuild; nothing generated is checked in. `web/fonts/` holds the dashboard's
Archivo and IBM Plex Mono, embedded because the access point has no internet. The whole page
costs about 130 KB of flash, nearly all of it fonts.

To check layout changes without flashing, serve `web/` from any static server; the live panels
stay empty without the robot's endpoints.

The dashboard streams all diagnostic data at 10 Hz:

- Radio state (connected, armed, stick percentages, switches)
- Motor commands (left, right)
- Accelerometer (x, y, z)
- Orientation (upside down detection)
- Loop timing and WiFi client count
- Pack voltage and current (`vbat` and `ibat`, sampled every 10 ms and repeated in between; `ibat` is positive while discharging)
- Yaw rate from the gyro and the rate the yaw loop commands (`yaw_rate`, `yaw_rate_cmd`, deg/s clockwise; the command is `nan` when the loop is not running)
- Stall guard state (`motor_rise_left` through `erpm_crc_errors`, between `yaw_rate_cmd` and `seq`; see Stall protection)
- Row sequence number (`seq`, last CSV column, +1 per row sent; a gap in a recording means the browser dropped rows)
- Current mode (combat / tuning)

### Sensor and I2C panel

Below the live values, the page polls `/status` once a second and shows the health of `Wire1`
and both sensors. Red rows are the ones to look at.

- **I2C bus**: idle level of SDA and SCL (a line stuck low means a device is holding it), and
  which addresses answered the last scan. The boot scan runs before either sensor starts.
  **Rescan bus** queues a new scan, which runs once the robot is disarmed.
- **BNO055**: whether `begin()` succeeded, the chip ID it read (0xA0) and the I2C result of that
  read, dropouts after boot, sample count and rate (about 100 Hz), and the age of the last
  sample. Ages are measured to when the loop took the snapshot, once a second, not to when
  the page asked. Once running, it also shows the operation mode (IMUPLUS is 0x08), system
  status (5 is fusion running), system error, self-test bits, and calibration.
- **INA238**: present, device ID (0x238x), last I2C result, read and failure counts.

A sensor that fails at boot or drops out stops being read, so the robot keeps driving. The
BNO055 is only retried after a reboot.

### Watchdog and crash dumps

A `loop()` pass longer than 5 s trips the task watchdog (`enableLoopWDT()` at the end of
`setup()`). So does core 0's idle task starving. A trip panics, writes an ELF core dump to the
`coredump` partition (`partitions.csv`), and reboots, so a hang no longer leaves the robot dead
with no access point. ArduinoOTA uploads pause the loop watchdog while they run.

The LAST REBOOT panel shows the reset reason and the dump summary. After a `TASK_WDT` reset, the
summary's task is whatever ran on core 0 when the watchdog interrupt fired. The stuck task is in
the full decode:

```bash
scripts/coredump [PORT] [ELF]   # reads the partition over USB (resets the robot), decodes every task
```

Pass the ELF of the build that crashed; the panel shows its SHA-256 prefix to compare. ERASE DUMP
clears it once read. The partition table only changes with a USB flash: OTA uploads keep the old
one.

### ESP-NOW panel

The same `/status` poll carries the robot's side of the ESP-NOW link. The dongle's side, with its
ACK rate and the robot's telemetry delivery, is on `firmware/espnow_dongle/scripts/espnow_diag.py`.

- **Link**: whether `esp_now_init()` ran, the channel (or the sweep and the last good channel),
  the dongle MAC it hears and how long ago, the telemetry ACK rate the robot reports as `lq`, the
  latest command (seq, age, values, fresh), whether this tick drove from ESP-NOW, and OTA state.
  The subtitle says whether `ESPNOW_COMMAND_SUBSTITUTION` is on or the robot is in shadow mode.
- **Refused**: frames with our session magic from a MAC not on the allowed list, with the last
  such MAC.
- **Allowed dongles**: the robot accepts only dongles paired from this page, up to 8, saved in
  NVS (namespace `espnow_pair`) so they survive reboots and reflashes. None are built into the
  firmware. To pair a new dongle, plug it in near the robot, open this page, and press ALLOW
  next to its MAC, or type the MAC and press ADD. REMOVE unpairs. All three need the robot
  disarmed. The page queues the request and `loop()` applies it, since the NVS write stalls
  both cores for a few ms. Erasing flash (`pio run -t erase`) clears the list, and the robot
  then accepts no dongle until one is paired again.
- **Traffic**: received frames by kind with per-second rates, late commands the tracker dropped,
  wrong-size frames (robot and dongle built from different protocol headers), receive queue
  overflows (the loop fell behind the WiFi task), telemetry sent, and send results: ACKed,
  failed after retries, and refused by `esp_now_send()`.

### WiFi settings

The WiFi panel saves up to 8 networks (SSID and password) to NVS, where they survive reboots and
reflashing. Passwords never come back to the page.

- **Access point**: boots as `MR-STABS` and never looks for a router.
- **Client**: boots joining the network picked in **Connect to**. With no connection after 30 s
  it gives up and brings up the `MR-STABS` access point instead, until the next boot or apply. A
  connection that drops after joining retries on its own and does not fall back.

**Save and apply** stores the choice and restarts WiFi with it straight away, no reboot. The page
loses its connection when the network changes. The header pill shows AP or CLIENT; red means
the robot fell back to its access point.

Settings only change while the robot is disarmed: an NVS write stalls both cores for a few ms.
Everything else runs in a task on core 0, so connecting, the 30 s timeout, and the fallback never
block the control loop, and the robot drives normally in either mode.

With `ESPNOW_ENABLED` on, the robot stays an access point on the ESP-NOW channel whatever is
saved, since joining a router would move the channel.

The Jetson reads the robot at `esp32_host`, an IP literal. In client mode, give the robot a DHCP
reservation on the router and set `esp32_host` to it.

### Telemetry ring

Every 10 ms the loop stores one `espnow_protocol::TelemetryPayload` (155 B, the ESP-NOW telemetry
frame) in a ring in PSRAM: 9000 records, 90 s. Without PSRAM it falls back to 500 records
(5 s) in internal RAM; the boot log over USB says which. Writing never waits on a reader; a reader
drops any record the loop overwrote mid-copy (`lib/telemetry_ring`, tested in
`test/test_telemetry_ring`).

`GET /telemetry?from=<seq>&max=<n>` returns a `TelemetryBatchHeader` (in
`firmware/shared/espnow_protocol`) and then up to 256 records from `from` on, oldest first. With
no `from` it returns the header alone, which tells a new client where the head is. The header's
`boot_id` changes on every boot, when seqs restart at 0. The Jetson's `Esp32WifiTelemetry` polls
it every 100 ms for the sysid profile, so a WiFi dropout shorter than 90 s costs no rows.

### Recording Data

1. Click **Record** on the dashboard. The stream switches to full loop rate and the browser accumulates every data point.
2. Click **Stop** when done.
3. Click **Download CSV** to save the recorded data as a timestamped CSV file.

Recording happens entirely in the browser -- the ESP32 does not store data, so there is no RAM limit on recording duration (limited only by browser memory).

### Checking for drive twitches

`scripts/log_drive_twitch.py` records the same stream from the command line and checks two causes
of twitching while driving: the flip-switch-DOWN upside-down detection flipping the throttle by
mistake, and DShot 3D direction reversals that heading hold causes. Join the MR-STABS WiFi, then:

```bash
./scripts/log_drive_twitch.py --seconds 60           # capture, then analyze
./scripts/log_drive_twitch.py --csv mr_stabs_x.csv   # analyze a dashboard CSV
```

### Zero Overhead

The diagnostics server does no work when no browser is connected. There is no impact on combat mode performance when the dashboard is not open.
