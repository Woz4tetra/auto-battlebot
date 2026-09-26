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
| Pack voltage | INA219 breakout at 0x40, on the BNO055's I2C bus (`Wire1`) |

### INA219 pack voltage wiring

The INA219 reads only bus voltage, in its 32 V range. Current is not measured.

- One sense wire from pack + through a 1 kohm series resistor to VIN+ only. VIN- stays unconnected: the chip reads the bus through the onboard 0.1 ohm shunt, which carries only its microamps. Wiring pack + to both pins would put the shunt in parallel with the resistor and bypass it.
- After the resistor, a 0.1 uF cap to ground (100 us RC, filters the power-on ringing) or an SMAJ18A TVS to ground. The bus input is 26 V absolute max, a full 4S LiHV pack is 17.4 V, and closing the switch rings the leads against the ESC input caps up to nearly twice pack voltage. The resistor also limits fault current, so a failed chip cannot put pack voltage onto the shared I2C bus.
- VCC from the QT Py's 3.3 V, so the I2C pull-ups stay at 3.3 V.
- GND at battery negative.

Calibrate once against a multimeter, at rest and during a punch, and set `VBAT_CAL_SCALE` in `include/vbat_sensor.h`. The series resistor shifts the reading about 0.3%. If the INA219 does not answer at boot, `vbat` reads `nan` and the robot runs normally.

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
```

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
- Pack voltage (`vbat`, last CSV column, sampled every 10 ms and repeated in between)
- Current mode (combat / tuning)

### Recording Data

1. Click **Record** on the dashboard. The stream switches to full loop rate and the browser accumulates every data point.
2. Click **Stop** when done.
3. Click **Download CSV** to save the recorded data as a timestamped CSV file.

Recording happens entirely in the browser -- the ESP32 does not store data, so there is no RAM limit on recording duration (limited only by browser memory).

### Zero Overhead

The diagnostics server does no work when no browser is connected. There is no impact on combat mode performance when the dashboard is not open.
