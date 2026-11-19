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
| Guard state | `NOMINAL`; `HOT` past 35 C with full power; `LIMITING` while the scale is under 100%; `OFF` (red) with no sensor for a motor; `TRIPPED` (red) once a motor's rise reaches `trip_rise_c`, drive cut until disarm |
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
`docs/experiments/firmware/espnow_link/espnow_transport_notes_2026-11-15.md`). `./scripts/ota` builds the image, copies it to the ZED Box, and
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
`firmware/blheli/README.md`.

Bench only: props off, wheels clear, transmitter disarmed.

### From the diagnostics page

1. Open the page, find ESC SETTINGS, press ENTER TUNING MODE.
2. READ BOTH. Every read keeps a backup (`esc<N>_<time>.json`, the CLI's format) in the
   browser's localStorage. Nothing is stored on the robot, and nothing downloads on its own.
3. Edit in the table. Amber rows differ between left and right; → and ← copy one value across.
   An outlined cell is edited and not yet written.
4. APPLY on one ESC, or APPLY BOTH, shows each change and any warning. WRITE keeps the ESC's
   current settings as `esc<N>_before_write_<time>.json`, writes, reads back to verify, and keeps
   the result as `esc<N>_after_write_<time>.json`.
5. RESTORE loads a backup into the form for the same ESC, then the same review and write.
6. DOWNLOAD CURRENT saves a zip with the newest backup of each ESC; DOWNLOAD HISTORY saves every
   backup this browser holds. localStorage is per origin, so `mr-stabs.local` and `192.168.4.1`
   keep separate histories. CLEAR HISTORY deletes them from the browser.
7. EXIT TUNING MODE.

### From the CLI over WiFi

```bash
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local read
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local backup firmware/blheli/backups
venv/bin/python firmware/blheli/blheli32.py --url http://mr-stabs.local write new.json --esc 0
```

The CLI enters tuning mode for the command and leaves it after. Use `192.168.4.1` on the robot's
access point if mDNS does not resolve.

Tuning mode waits up to 3 s after the previous exit before it starts, while the ESCs restart, and
holds the first frame 1 s while they drop into their bootloader.

### Over USB, without WiFi

`scripts/usb_dashboard.py` serves the whole diagnostics page on localhost through the USB cable.
The robot's log lines, including `esc tuning:` lines for every failed bootloader exchange, print
in the same terminal.

```bash
venv/bin/python firmware/mr_stabs_mk2/scripts/usb_dashboard.py --log robot.log
# open http://localhost:8080, or:
venv/bin/python firmware/blheli/blheli32.py --url http://localhost:8080 read
```

Opening the port does not reset the robot. The script leaves DTR up, since pyserial dropping DTR
while RTS is still high resets the chip. `pio device monitor` and the script cannot share the port.

How it works: each browser request goes down the serial line as one base64 line, `src/usb_bridge.cpp`
replays it against the robot's own web server on 127.0.0.1 and streams the response back in
frames with a CRC32, and the script writes it to the browser. A damaged or stalled GET is sent
again (a loopback TCP response occasionally stalls for 10 s); a POST gets a 502 instead, since the
robot may have acted on it. Two constraints found on the bench:

- The bridge task runs on core 1, where `Serial.begin()` put the USB-Serial/JTAG interrupt.
  Writing from core 0 lost whole 64-byte packets.
- Arduino's `log_e()` output goes through the bridge task as whole lines, so it never splits a
  frame.

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

## Notes moved from code comments

### crash reports and scripts/coredump

- `mr_stabs_mk2/include/crash_report.h`: Why the robot last rebooted, and the core dump a crash left in flash. A panic or a task watchdog timeout writes an ELF core dump to the coredump partition (partitions.csv), then reboots. read() picks it up on the next boot and keeps the summary for the diagnostics page; the full dump, with every task's stack, stays in flash until erased. Decode it over USB with scripts/coredump.

### CRSF telemetry

- `mr_stabs_mk2/include/crsf_bridge.h`: Battery and attitude frames alternate, one every 50 ms, so each reaches the radio at 10 Hz. That matches the BNO055's 100 ms sample interval; faster would repeat the same attitude.
- `mr_stabs_mk2/src/crsf_bridge.cpp`: Capacity used and percent remaining are not tracked yet, so they go out as 0.

### ESP-NOW channel sweep

- `mr_stabs_mk2/include/espnow_link.h`: ~6 dongle repeats at 250 Hz, plus switch settling

### ESP-NOW telemetry rate

- `mr_stabs_mk2/include/espnow_link.h`: The control loop runs at ~1 kHz. Telemetry at that rate would hold the channel ~35% of the time at 6 Mbps, so it is sent at 100 Hz, which still resolves every 10 ms control tick.

### ESP-NOW OTA

- `mr_stabs_mk2/include/espnow_link.h`: OTA transfer gives up after this long without a valid OTA frame. The running image is untouched until Update.end() switches the boot slot, so an abort always leaves the old firmware running.
- `mr_stabs_mk2/include/espnow_link.h`: Every boot reports OTA_VALIDATED, with its MD5, once it has run this long and heard a dongle. The flasher waits for it as proof the new image runs.
- `mr_stabs_mk2/scripts/espnow_ota.py`: Flash mr_stabs_mk2 over ESP-NOW through the USB dongle. Run it on the machine the dongle is plugged into, normally the ZED Box (scripts/ota does that over ssh). The robot must be disarmed. The transfer is go-back-N over the 0x10-0x14 messages in docs/experiments/firmware/espnow_link/espnow_transport_notes_2026-11-15.md, then the script waits for the rebooted robot to report the new image valid and checks its MD5. venv/bin/python firmware/mr_stabs_mk2/scripts/espnow_ota.py firmware.bin --manage-service
- `mr_stabs_mk2/scripts/ota`: Build mr_stabs_mk2 and flash it over ESP-NOW (docs/experiments/firmware/espnow_link/espnow_transport_notes_2026-11-15.md). The robot must be disarmed. The flasher then waits for the new image to report running, ~10 s after boot with a dongle in range. There is no rollback: a bad image is rescued over USB with scripts/upload. Usage: scripts/ota # through the dongle on the ZED Box, over ssh scripts/ota --local [PORT] # through a dongle on this machine (default /dev/espnow_dongle) Env, same as scripts/deploy_to_jetson.sh: JETSON_HOST ZED Box hostname or IP (default: jetson) JETSON_USER SSH user (default: what ~/.ssh/config resolves) JETSON_KEY SSH private key (default: ~/.ssh/<host> if it exists) JETSON_PATH repo on the box (default: ~/auto-battlebot)
- `mr_stabs_mk2/scripts/ota`: -t so sudo can prompt for the service stop and start.
- `mr_stabs_mk2/src/espnow_link.cpp`: Deep enough to hold the host's 16-chunk OTA window while a 64 KB block erase stalls loop().
- `mr_stabs_mk2/src/espnow_link.cpp`: Tell the host where to resume right away rather than waiting for its timeout, but at most every 20 ms: a lost chunk drops the rest of the window, one status per chunk.
- `mr_stabs_mk2/src/espnow_link.cpp`: The Arduino bootloader is built without rollback, so nothing here reverts a bad image: this is the flasher's proof that the new image runs, for 10 s, with the radio working. A bad image is rescued over USB (scripts/upload).
- `mr_stabs_mk2/src/espnow_link.cpp`: The flasher waits for this to confirm the new image is the one running. Repeated over 5 s because a single frame can be lost.
- `shared/espnow_protocol/src/espnow_protocol.h`: image accepted, rebooting sent after reboot once the new image marked itself valid
- `shared/espnow_protocol/src/espnow_protocol.h`: Update.begin refused: size does not fit the slot

### I2C troubleshooting

- `mr_stabs_mk2/include/i2c_bus.h`: Reads the idle level of both lines. Both should be high between transactions; a line stuck low means a device is holding it, and every transaction will fail until power is cycled.

### runtime tuning

- `mr_stabs_mk2/include/tuning_store.h`: The runtime settings (lib/tuning) shared by the tasks that change them and the loop that uses them. The diagnostics page writes from the web server task and ESP-NOW from loop(), so every access copies under a spinlock. loop() applies a change on its next pass and saves it to NVS only while disarmed, since a flash write stalls both cores for a few ms; a change made while armed is saved at the next disarm, and lost if the robot powers off first.
- `mr_stabs_mk2/lib/tuning/tuning.h`: Every setting the diagnostics page and ESP-NOW can change at runtime. The robot keeps them in NVS, one float per field under the field's key, so adding a field keeps the others' values.

### BNO055 status registers

- `mr_stabs_mk2/include/updown_sensor.h`: OPR_MODE, 0x08 for IMUPLUS SYS_STATUS, 5 when fusion is running ST_RESULT, 0x0F when accel, mag, gyro and MCU all passed SYS_ERR, 0 when there is no error CALIB_STAT: sys, gyro, accel, mag, two bits each Gyro sign check: samples where the gyro and the heading change agreed or disagreed.
- `mr_stabs_mk2/src/updown_sensor.cpp`: 0x35..0x3A: CALIB_STAT, ST_RESULT, INT_STA, SYS_CLK_STATUS, SYS_STATUS, SYS_ERR

### BNO055 sample rate

- `mr_stabs_mk2/include/updown_sensor.h`: The BNO055 fusion output updates at 100 Hz, so reading faster returns repeated values.

### Sensor and I2C panel

- `mr_stabs_mk2/include/updown_sensor.h`: Rereads the status registers if DETAILS_INTERVAL has passed and the sensor is up. Two short I2C reads. Every sample calls it, so CALIB_STAT stays fresh for telemetry; the diagnostics page also calls it, for while the link is down and samples stop.
- `mr_stabs_mk2/src/i2c_bus.cpp`: The I2C driver leaves both pins as open-drain with input enabled, so the GPIO input register reads the real line level without disturbing the bus.

### IMU

- `mr_stabs_mk2/include/updown_sensor.h`: The same rate from the change in heading between the last two samples. Laggier and coarser than the gyro; kept to check the gyro's sign.
- `mr_stabs_mk2/src/main.cpp`: Read the IMU every loop whatever the radio is doing, so heading stays fresh for the PID and the diagnostics page and telemetry track the robot with no handset. Rate-limited inside the sensor. Reconnect stays off: it blocks the loop for over a second.
- `mr_stabs_mk2/src/updown_sensor.cpp`: IMUPLUS fuses accelerometer and gyro only. NDOF (the library default) also fuses the magnetometer, which the drive motor current can bend, moving heading under throttle. Heading is now relative to the orientation at boot.
- `mr_stabs_mk2/src/updown_sensor.cpp`: An absent or unplugged sensor is never read: each failed read blocks the loop for milliseconds, and at the 10 ms sample interval that starved the ESCs of DShot frames. begin() blocks for over a second, so it is only retried when the caller allows it. main.cpp never does, so a sensor that drops out stays off, and heading hold with it, until the next reboot.

### USB dashboard protocol

- `mr_stabs_mk2/include/usb_bridge.h`: Serves the diagnostics page over the USB serial port, for when WiFi is not reachable. scripts/usb_dashboard.py on the laptop listens on localhost and sends each browser request down the serial line; this task replays it against the robot's own web server on 127.0.0.1 and sends the raw HTTP response back. Log lines keep flowing on the same port between frames. Frames are single lines. Each data frame carries a CRC32 of its bytes, and the laptop retries a GET whose response arrives damaged. laptop -> robot "\x1eQ <id> <base64 request>\n" open a connection and send the request "\x1eC <id>\n" the browser went away, close it robot -> laptop "\x1eD <id> <base64 bytes> <crc32>\n" response bytes, in order "\x1eE <id>\n" the connection closed
- `mr_stabs_mk2/scripts/usb_dashboard.py`: Serve the robot's diagnostics page on localhost over USB serial. Every browser request goes down the USB cable to the robot, which replays it against its own web server (src/usb_bridge.cpp) and streams the raw response back. The robot's log lines print here, and to --log when given. Nothing on the robot needs WiFi. venv/bin/python firmware/mr_stabs_mk2/scripts/usb_dashboard.py # then open http://localhost:8080

### current calibration

- `mr_stabs_mk2/include/vbat_sensor.h`: R1's nominal value. Its tolerance and the pour resistance inside the Kelvin taps set the current reading's gain error; calibrate it against a clamp meter.

### INA238 sampling

- `mr_stabs_mk2/include/vbat_sensor.h`: A sample is two reads (bus voltage, then shunt voltage), each one register-pointer write plus a 2-byte read, about 47 SCL clocks. Wire1 runs at the default 100 kHz, so that is ~1 ms on the wire plus driver overhead, blocking. The control loop has no fixed period (it varies with radio state, BNO055 reads, and recording mode), so sampling runs on a time interval instead of every Nth loop. 10 ms still resolves a punch's sag and current spike, which last 100s of ms. Loops in between repeat the last values.

### INA238 registers

- `mr_stabs_mk2/include/vbat_sensor.h`: DEVICE_ID at boot, 0x238x for an INA238

### WiFi modes

- `mr_stabs_mk2/include/wifi_manager.h`: Chooses between the robot's own access point and joining a router, from settings the diagnostics page saves to NVS. AP mode: the access point comes up on boot and the robot never looks for a router. Client mode: the robot joins the selected network on boot. With no connection after CONNECT_TIMEOUT_MS it gives up and brings up the access point instead, until the next boot or the next apply. A connection that drops later reconnects on its own. Everything after the first start runs in a task on core 0, so the timeout, the fallback, and a mode switch from the page never stall the control loop on core 1.
- `mr_stabs_mk2/include/wifi_manager.h`: AP mode client mode, waiting for the first connection client mode, joined client mode, joined once and lost it; the driver retries client mode, gave up after CONNECT_TIMEOUT_MS
- `mr_stabs_mk2/src/espnow_link.cpp`: Pinned with the SoftAP up, the AP already owns this channel and this is a no-op.
- `mr_stabs_mk2/src/wifi_manager.cpp`: Modem sleep holds incoming packets for up to 100-300 ms.
- `mr_stabs_mk2/src/wifi_manager.cpp`: Leave no stale password behind in NVS.

### WiFi modes and ESP-NOW channel

- `mr_stabs_mk2/include/wifi_manager.h`: Starts WiFi from the saved settings before returning, so ArduinoOTA and the web server can bind afterwards. force_ap keeps the access point up on ap_channel whatever is saved: ESP-NOW needs a fixed channel, which a router would take over.

### bidirectional DShot

- `mr_stabs_mk2/lib/DShot/DShot.cpp`: DShot bit timing in nanoseconds Bit‑1: 75% active, 25% idle Bit‑0: 37.5% active, 62.5% idle Inverted for bidirectional DShot: the line idles high and each bit starts low.
- `mr_stabs_mk2/lib/DShot/DShot.cpp`: The receiver arms before the frame and records frame, gap, and reply as one reception, so it can never arm too late for the reply. The idle threshold must outlast the gap (about 30 us); 80 us of quiet ends the reception.
- `mr_stabs_mk2/lib/DShot/DShot.cpp`: A high run this long is the gap between our frame and the reply. The frame's longest high run is 2.1 us and the reply's is 8 us at DShot300.
- `mr_stabs_mk2/lib/DShot/DShot.cpp`: the RX memory block's 48 items
- `mr_stabs_mk2/lib/DShot/DShot.cpp`: rmt_config() pointed the pin at one channel or the other. Connect both: the TX channel's output and the RX channel's input on the same pad, with the output enable taken from the GPIO register so send() can release the line for the reply. The pull-up holds it high then.
- `mr_stabs_mk2/lib/DShot/DShot.cpp`: Drain the ring and keep the first reception that decodes. The driver re-arms the receiver after every reception, so anything after the reply is noise picked up while the line sits on the pull-up for the rest of the loop (worst with WiFi streaming every loop).
- `mr_stabs_mk2/lib/DShot/DShot.cpp`: Armed before the frame, so no software latency can miss the reply's start. The capture holds our frame, the gap, and the reply; collect_reply() skips to the gap.
- `mr_stabs_mk2/lib/DShot/DShot.h`: Bidirectional DShot: inverted frames out, and the ESC's eRPM reply read back on the same pin through an RX channel. BLHeli_32 sees the inverted signal and switches modes on its own.
- `mr_stabs_mk2/lib/DShot/DShot.h`: Collects the reply to the previous frame, arms the receiver, and sends this frame. The reply arrives about 30 us after the frame, so it is read one call later and never waited on.
- `mr_stabs_mk2/lib/erpm_decode/erpm_decode.h`: Decodes a bidirectional DShot eRPM reply from the RMT's run lengths. Pure C++, so the native env tests it against hand-built pulse trains. The DShot driver lives in lib/DShot, which needs the Arduino core. The ESC answers each inverted DShot frame with 21 bits at 5/4 of the DShot bit rate. The line idles high and the first falling edge is the start bit. Every edge is a 1 and every bit period without an edge is a 0, so a run of n bit periods between edges decodes to a 1 followed by n - 1 zeros. The 20 bits after the start bit are four 5-bit GCR groups, one per nibble of eeem mmmm mmmm cccc: a period of m << e us and a 4-bit CRC.
- `mr_stabs_mk2/lib/erpm_decode/erpm_decode.h`: the longest period the reply encodes
- `mr_stabs_mk2/lib/esc_map/esc_map.h`: DShot 3D mode: 48-1047 reverse, 1048-2047 forward, higher value = faster in both directions.

### tuning

- `mr_stabs_mk2/lib/tuning/tuning.cpp`: Ranges only catch typos and torn writes; the defaults in yaw_control.h say what is sensible.

### BNO055 gyro range

- `mr_stabs_mk2/lib/yaw_control/yaw_control.h`: One full wrap of a BNO055 gyro axis in rad/s: the register is int16 at 16 LSB per deg/s, so it spans 65536 / 16 = 4096 deg/s.

### partitions and core dumps

- `mr_stabs_mk2/platformio.ini`: Adds a core dump partition; see the header of partitions.csv. Envs copy it by name below. OTA uploads do not rewrite the partition table: flash over USB once to apply it.
- `mr_stabs_mk2/scripts/coredump`: Read the core dump from the robot over USB and decode it: the exception, registers, and every task's backtrace. After a task watchdog reset, find the stuck task in the thread list (the summary names whatever ran on the core that took the watchdog interrupt). Pass the ELF of the build that crashed; the diagnostics page shows its SHA-256 prefix to compare. Reading resets the robot. Usage: scripts/coredump [PORT] [ELF]
- `mr_stabs_mk2/web/app.js`: A task watchdog aborts from its interrupt on core 0, so the summary names whatever ran there. The stuck task is in the full decode (scripts/coredump).

### build flags

- `mr_stabs_mk2/platformio.ini`: Serial on the chip's USB-Serial/JTAG port instead of TinyUSB. That port is hardware: it stays up when the firmware hangs, and esptool can reset the chip into the ROM bootloader through it, so a bad image is rescued over USB without pressing BOOT (glued on this robot). AsyncTCP's task defaults to any core, so serving the web page or /telemetry could preempt the control loop on core 1. Pinned to core 0 with WiFi.

### capturing CRSF telemetry

- `mr_stabs_mk2/scripts/capture_crsf_telemetry.py`: Capture the raw CRSF telemetry frames an EdgeTX radio receives, and decode the sensors. Needs the custom EdgeTX build in ~/edgetx, whose `telemetry on` CLI command mirrors every byte from the external module to USB serial. Plug the radio in over USB and pick "USB Serial (VCP)". Prints each battery and attitude frame as raw bytes and decoded values, then a count of every frame type seen, and writes the same to a log file.

### web page embedding

- `mr_stabs_mk2/scripts/embed_web.py`: PlatformIO pre-build step: gzip web/ into a C header the diagnostics server serves. Writes web_assets.h into the build directory and puts that directory on the include path, so nothing generated lands in the source tree. The header is rewritten only when its content changes, so an untouched web/ does not trigger a recompile. gzip runs with mtime 0 to keep the output byte-identical between builds.

### drive twitch logging

- `mr_stabs_mk2/scripts/log_drive_twitch.py`: Log Mr Stabs over the diagnostics stream and check two causes of twitching while driving. 1. Spurious upside-down flips. With the flip switch DOWN, the firmware takes is_upside_down from the BNO055 gravity vector and negates the throttle when it changes. It flips upside down below accel_z = -1 m/s^2 and back above +1 m/s^2. A flip that reverts within --flip-window while the switch did not move is a false detection: the drive reversed under the driver. 2. DShot 3D reversals. A wheel command crossing from one side of the ESC deadzone to the other is a direction reversal, which the ESC handles with a stall. A reversal is blamed on heading hold when the driver's own stick mix for that wheel kept its sign. Join the MR-STABS WiFi, arm, and drive. Ctrl-C stops the capture early. --csv analyzes a file from an earlier capture or from the dashboard's Download CSV button instead.

### diagnostics HTTP API

- `mr_stabs_mk2/scripts/log_drive_twitch.py`: The connect greeting is also a data line; only full diag rows have every column.
- `mr_stabs_mk2/src/diagnostics_server.cpp`: Records go out exactly as the ring holds them, after a TelemetryBatchHeader. No `from` returns just the header, which tells a new client where head is without replaying the whole ring.
- `shared/espnow_protocol/src/espnow_protocol.h`: Body of the robot's `GET /telemetry?from=<seq>&max=<n>` over WiFi: this header, then `count` TelemetryPayload records, oldest first. The firmware fills a ring of them at the ESP-NOW telemetry rate whether or not anyone reads it, so a client that loses the link catches up on everything still in the ring once it is back. telemetry_seq in each record is the ring's seq.

### flashing

- `mr_stabs_mk2/scripts/ota_wifi`: Flash over WiFi with ArduinoOTA. With no argument it targets the robot's own access point (join MR-STABS first). In client mode pass the address the router gave it, or mr-stabs.local: ./scripts/ota_wifi 192.168.50.40
- `mr_stabs_mk2/scripts/upload`: Flash over USB. No button needed: esptool resets the chip into its ROM bootloader through the USB-Serial/JTAG port, even with the firmware hung. Uses /dev/mr_stabs (install_udev_rules.sh) when present, since the ESP-NOW dongle shows the same 303a:1001 and auto-detect can pick it. Usage: scripts/upload [PORT]

### memory

- `mr_stabs_mk2/src/crash_report.cpp`: The summary is large for the stack; it is read once, at boot.
- `mr_stabs_mk2/src/diagnostics_server.cpp`: PSRAM first: a full batch is 30 KB, too much to ask of fragmented internal RAM.
- `mr_stabs_mk2/src/main.cpp`: Every 100 Hz telemetry frame, kept in PSRAM for GET /telemetry, so a WiFi client that drops out catches up instead of losing the gap. 9000 records of 155 B is 1.4 MB of the 2 MB PSRAM, 90 s. Without PSRAM it falls back to a few seconds in internal RAM.

### ESC Pin Mapping

- `mr_stabs_mk2/src/esc_config.cpp`: UART1 is the CRSF receiver

### General

- `mr_stabs_mk2/src/espnow_link.cpp`: WiFi task context: filter, copy, and return. Never touch ESCs or flash here.
- `mr_stabs_mk2/include/vbat_sensor.h`: `RETRY_INTERVAL_US` is 1 s so an unplugged sensor cable costs one blocked I2C transaction per second rather than one per sample.

### motor guard

- `mr_stabs_mk2/src/main.cpp`: The thermal state survives a watchdog, panic, or software reset, so a reset mid-match resumes the estimate. A power-on starts cold. Whether it survives a brownout reset is untested.
- `mr_stabs_mk2/src/main.cpp`: Before the ESP-NOW OTA return, so the guard's model keeps cooling during a transfer.

### status LED

- `mr_stabs_mk2/src/main.cpp`: Tuning mode: a slow blue breath, so the robot reads as not drivable from across the room.

### transmitter trim

- `mr_stabs_mk2/src/main.cpp`: The stick's resting trim (about 1% on this transmitter) would otherwise creep a wheel.

### ESC startup

- `mr_stabs_mk2/src/main.cpp`: Sends DShot stop frames every 5 ms from a timer while setup() runs, so the ESCs see an unbroken signal from the moment their pins are driven until loop() takes over. The IMU, WiFi, and ESP-NOW setup between those points takes about 2.5 s.
- `mr_stabs_mk2/src/main.cpp`: ESCs need continuous DShot zero-throttle frames to initialize (~2 seconds). The timer has sent them since the ESC pins came up; top that up to 2 s, then hand over to loop(). Before the timer, the 2.4 s of IMU and WiFi setup left the line idle, and a reset of the ESP32 alone made the running ESCs play their wake-up jingle twice. A pack power-on still plays it twice: the ESCs sample the line before any firmware can drive it (measured 212 ms after reset).

### I2C bus speed

- `mr_stabs_mk2/src/main.cpp`: BNO055 IMU lives on the Wire1 I2C bus Stays at the default 100 kHz. The BNO055 stretches the clock, which can fail at 400 kHz, and every failed read blocks the loop for the 50 ms Wire timeout, starving the ESCs of DShot frames. Recorded for the diagnostics page: shows which devices answered at boot.

### watchdog and core dumps

- `mr_stabs_mk2/src/main.cpp`: A loop() pass that takes over CONFIG_ESP_TASK_WDT_TIMEOUT_S (5 s) panics: the core dump goes to flash and the robot reboots, instead of hanging with the ESCs on their last command. The core already watches core 0's idle task. Core 1's idle task is not watched: loop() spins on core 1 without yielding, so it never runs. Started last: setup() blocks longer than that.

### INA238 configuration

- `mr_stabs_mk2/src/vbat_sensor.cpp`: MODE=1011 (shunt and bus voltage, continuous), VBUSCT=101 and VSHCT=101 (1052 us each), VTCT=101 (unused), AVG=001 (4 samples). One averaged pair takes 8.4 ms, which fits inside one SAMPLE_INTERVAL_US, so every read returns a fresh average and ESC switching ripple is averaged out. A chip that browns out resets to 0xFB68 (continuous bus, shunt, and temperature at 1052 us, no averaging), which still updates both readings every 3.2 ms.
- `mr_stabs_mk2/src/vbat_sensor.cpp`: Current comes from the raw shunt voltage at the reset-default ADCRANGE=0 (+-163.84 mV). ADCRANGE=1 would give 4x the resolution, but a brownout clears it back to 0 and the current would then read 4x high with nothing to flag it. 5 uV per LSB over the 0.5 mohm shunt is 10 mA per LSB, +-327 A full scale.

### Stall protection

- `mr_stabs_mk2/test/test_motor_guard/test_motor_guard.cpp`: Runs the guard against the plant the way main.cpp does: update with the previous sample's duty and the measured pack current, then scale this sample's command.
- `mr_stabs_mk2/test/test_motor_guard/test_motor_guard.cpp`: The model bound d V / R_cold only holds with R_cold the lowest resistance the circuit has.
- `mr_stabs_mk2/test/test_motor_guard/test_motor_guard.cpp`: Full reverse at speed. eRPM never reads 0 at this sample rate, so the shunt tiebreak must flip the sign. Until it does, the estimate errs high.

### diagnostics page

- `mr_stabs_mk2/web/app.js`: While recording the robot sends every loop tick, so the DOM updates once per frame instead of once per message.
- `mr_stabs_mk2/web/app.js`: Rebuilt only when the list changes, so a poll does not close a dropdown the operator has open.
- `mr_stabs_mk2/web/app.js`: The lockout box above explains forced AP mode; the note would only repeat it.
- `mr_stabs_mk2/web/app.js`: Rebuilt only when the list changes, so a button is never replaced under a click.
- `mr_stabs_mk2/web/app.js`: MAC on its own line and the state under it, so neither is cut off in a narrow column.
- `mr_stabs_mk2/web/esc_panel.js`: keep what the operator typed

### bidirectional DShot (RMT channels)

- `lib/DShot/DShot.cpp`: The ESP32-S3 RX-only RMT channels are 4 to 7. Their matrix signals count from `RMT_SIG_IN0_IDX`.
