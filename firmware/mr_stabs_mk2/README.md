# MR STABS MK2 Firmware

ESP32-S3 firmware for the MR STABS battlebot. Runs on an Adafruit QT Py ESP32-S3.

## Modes

The firmware has two modes, toggled by the `button_state` switch on the transmitter (rising edge toggles between them):

**Combat mode** (default) -- DShot300 motor control via the CRSF radio link. Differential drive mixing, accelerometer-based flip detection, upside-down compensation. LED shows a rainbow cycle.

**Tuning mode** -- Motors stop. USB serial passthrough activates so BLHeliSuite32 can configure both ESCs. LED shows a blue breathing pulse. OTA firmware updates also available in this mode.

## Hardware

| Component | Connection |
|---|---|
| Board | Adafruit QT Py ESP32-S3 (4MB Flash, 2MB PSRAM) |
| Left ESC | Pin A2, DShot300 via RMT channel 0 |
| Right ESC | Pin A3, DShot300 via RMT channel 1 |
| Radio RX | Crossfire Nano, UART on pins 17 (TX) / 18 (RX) |
| Accelerometer | ADXL375, I2C |
| Pack voltage and current | Matek I2C-INA-BM (INA228) at 0x45, on the BNO055's I2C bus (`Wire1`) |

### INA228 pack voltage and current wiring

The Matek I2C-INA-BM board carries an INA228 and a 200 uohm shunt. The firmware reads bus voltage and shunt voltage; current is shunt voltage over the nominal 200 uohm.

- Battery + and the ESC + lead solder to the two sides of the shunt, as close to it as possible. The voltage sense input is on board and rated 0-85 V, so no series resistor or clamp is needed.
- JST-GH-4P cable to the QT Py: 5 V, GND, SCL, SDA on `Wire1`. The board takes 4-9 V and makes its own 3.3 V for the INA228.
- Default address 0x45 (decimal 69); 0x44 and 0x41 are the alternatives.

The INA228 bus voltage is +-0.1% out of the box, so there is no calibration constant. Current accuracy is the shunt tolerance, about +-2%; check it once against a clamp meter. If the chip does not answer at boot, or reports a device ID other than INA228 (Matek also ships the board with an INA238), `vbat` and `ibat` read `nan` and the robot runs normally.

### CRSF telemetry

While the radio link is up, the firmware sends two CRSF sensor frames back to the transmitter, alternating every 50 ms so each arrives at 10 Hz:

| Frame | EdgeTX sensors | Source |
|---|---|---|
| Battery (0x08) | `RxBt` volts, `Curr` amps | INA228. `Capa` and `Bat%` are sent as 0: capacity is not tracked yet |
| Attitude (0x1E) | `Yaw`, `Roll`, `Ptch` | BNO055 Euler angles. Yaw is heading wrapped to +-180 degrees |

After flashing, run Telemetry > Discover new sensors on the radio. A NaN reading (no INA228) goes out as 0.

## Building and Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
# Compile
./scripts/compile

# Upload via USB
./scripts/upload

# Upload via OTA (connect to MR-STABS WiFi first)
./scripts/ota

# Serial monitor
./scripts/monitor

# Host-side unit tests (test/, GoogleTest, no board needed)
./scripts/test
```

Code with unit tests lives in `lib/` so the `native` environment can build it without the
Arduino core; `pio test` does not compile `src/`. The heading-hold PID is in `lib/pid/`.

## OTA Updates

The ESP32 creates a WiFi access point on boot:

- **SSID:** `MR-STABS`
- **Password:** `havocbots`

Connect your computer to this network, then run `./scripts/ota`. The OTA endpoint is at `192.168.4.1`. OTA is handled in both combat and tuning modes.

## Configuring ESCs via USB Passthrough

The firmware includes a BLHeli serial passthrough that lets BLHeliSuite32 read and write ESC parameters over USB.

### Prerequisites

- Install [BLHeliSuite32](https://github.com/bitdump/BLHeli) on your Linux PC (available as `blhelisuite32-bin` on the AUR, or run via WINE).
- A USB cable connected to the ESP32-S3.
- ESCs powered (battery connected).

### Connecting

1. Toggle the `button_state` switch on the transmitter. The LED changes to a blue breathing pulse and motors stop.
2. Open BLHeliSuite32 on your PC.
3. Select interface: **BLHeli32 Bootloader (Betaflight/Cleanflight)**.
4. Select port: the ESP32's serial port (typically `/dev/ttyACM0`).
5. Leave baud rate at **115200**.
6. Click **Connect**, then **Read Setup**.

Both ESCs appear as ESC 1 and ESC 2 in the interface.

### Saving Settings

1. Adjust parameters as needed (motor direction, startup power, timing, etc.).
2. Click **Write Setup** to send the configuration to both ESCs.
3. Settings persist in the ESC's non-volatile memory across power cycles.

### Saving an INI Backup

1. After reading the ESC setup, use **File > Save Setup** to export a `.ini` file.
2. To restore, use **File > Load Setup** then **Write Setup**.

### Exiting Tuning Mode

Toggle the `button_state` switch again. DShot reinitializes, the LED returns to rainbow, and the robot returns to combat mode.

## ESC Pin Mapping

| BLHeliSuite32 Label | Physical ESC | GPIO Pin |
|---|---|---|
| ESC 1 | Left motor | A2 |
| ESC 2 | Right motor | A3 |

## Diagnostics Dashboard

A live diagnostics web page is available over the WiFi access point. No extra software needed -- just a browser.

1. Connect your computer to the **MR-STABS** WiFi network.
2. Open `http://192.168.4.1` in a browser.

The dashboard streams all diagnostic data at 10 Hz:

- Radio state (connected, armed, stick percentages, switches)
- Motor commands (left, right)
- Accelerometer (x, y, z)
- Orientation (upside down detection)
- Loop timing and WiFi client count
- Pack voltage and current (`vbat` and `ibat`, the last two CSV columns, sampled every 10 ms and repeated in between; `ibat` is positive while discharging)
- Current mode (combat / tuning)

### Sensor and I2C panel

Below the live values, the page polls `/status` once a second and shows the health of `Wire1`
and both sensors. Red rows are the ones to look at.

- **I2C bus**: idle level of SDA and SCL (a line stuck low means a device is holding it), and
  which addresses answered the last scan. The boot scan runs before either sensor starts.
  **Rescan bus** queues a new scan, which runs once the robot is disarmed.
- **BNO055**: whether `begin()` succeeded, the chip ID it read (0xA0) and the I2C result of that
  read, dropouts after boot, sample count and age. Once running, it also shows the operation
  mode (IMUPLUS is 0x08), system status (5 is fusion running), system error, self-test bits,
  and calibration.
- **INA228**: present, device ID (0x228x), last I2C result, read and failure counts.

A sensor that fails at boot or drops out stops being read, so the robot keeps driving. The
BNO055 is only retried after a reboot.

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
