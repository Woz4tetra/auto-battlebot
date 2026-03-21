# MR STABS MK2 Firmware

ESP32-S3 firmware for the MR STABS battlebot. Runs on an Adafruit QT Py ESP32-S3.

## Modes

The firmware has two modes, toggled by the `button_state` switch on the transmitter (rising edge toggles between them):

**Combat mode** (default) -- DShot300 motor control via the CRSF radio link. Differential drive mixing, accelerometer-based flip detection, upside-down compensation.

**Tuning mode** -- Motors stop. BLE passthrough activates so the BLHeli_32 Android app can configure both ESCs wirelessly. OTA firmware updates also available in this mode.

## Hardware

| Component | Connection |
|---|---|
| Board | Adafruit QT Py ESP32-S3 (4MB Flash, 2MB PSRAM) |
| Left ESC | Pin A2, DShot300 via RMT channel 0 |
| Right ESC | Pin A3, DShot300 via RMT channel 1 |
| Radio RX | Crossfire Nano, UART on pins 17 (TX) / 18 (RX) |
| Accelerometer | ADXL375, I2C |

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

Connect your computer to this network, then run `./scripts/ota`. The OTA endpoint is at `192.168.4.1`. OTA is handled in both combat mode and tuning mode.

## Configuring ESCs via BLHeli Passthrough

The firmware includes an integrated BLHeli passthrough that lets you read and write ESC parameters from your phone over Bluetooth -- no USB cable or reflashing required.

### Prerequisites

- Install the **BLHeli_32** app (by Steffen Skaug) on your Android phone. The app was removed from Google Play but the APK is still available on [Aptoide](https://blheli-32.en.aptoide.com/app). See [Installing the APK](#installing-the-apk) below.
- Power on the robot and ensure the transmitter is connected.

### Installing the APK

The BLHeli_32 app must be sideloaded since it is no longer on Google Play. No root required.

1. On your phone, go to **Settings > Apps > Special app access > Install unknown apps** (path varies by manufacturer and Android version).
2. Enable the permission for your browser (e.g. Chrome).
3. Open the [Aptoide download page](https://blheli-32.en.aptoide.com/app) in your browser and download the APK.
4. Tap the downloaded `.apk` file from your notification bar or Downloads folder.
5. Tap **Install** when prompted.

On some phones the permission prompt appears automatically the first time you open an APK, so step 1 may not be needed.

### Connecting

1. Toggle the `button_state` switch on the transmitter. The serial console prints `Entering tuning mode` and motors stop.
2. Open the BLHeli_32 app on your phone.
3. Tap the Bluetooth icon and scan for devices. Select **MR-STABS-ESC**.
4. Once connected, tap **Read Setup**. The app reads parameters from both ESCs (they appear as ESC 1 and ESC 2 in the interface).

### Recommended First-Time Settings

Since the firmware uses DShot in 3D (bidirectional) mode, verify these settings on both ESCs:

- **Motor Direction:** Bidirectional / 3D
- **Low RPM Power Protect:** Off (or low, to avoid stalling at low speeds)
- **Startup Power:** Adjust to the minimum that reliably starts your motors
- **Motor Timing:** Medium (15-22.5 degrees) is a safe default
- **Min/Max Throttle:** Leave at defaults for DShot (the protocol handles calibration digitally)

### Saving Settings

After changing parameters in the app:

1. Tap **Write Setup** to send the new configuration to both ESCs. The app writes to each ESC in sequence.
2. The ESCs store settings in their own non-volatile memory. The settings persist across power cycles -- you only need to do this once.

### Saving an INI Backup

The BLHeli_32 app can export your ESC configuration to a file so you can restore it later or copy it to replacement ESCs:

1. After reading the ESC setup, tap the **menu** (three dots) or **save** icon.
2. Select **Save Setup File** (or **Save ini**).
3. Choose a location on your phone to save the `.ini` file. Name it something descriptive like `mr_stabs_left_right.ini`.
4. To restore from a backup, use **Load Setup File** then **Write Setup**.

### Exiting Tuning Mode

Toggle the `button_state` switch again. The serial console prints `Exiting tuning mode`, BLE shuts down, DShot reinitializes, and the robot returns to combat mode.

## ESC Pin Mapping

The passthrough reports 2 ESCs to the BLHeli_32 app:

| App Label | Physical ESC | GPIO Pin |
|---|---|---|
| ESC 1 | Left motor | A2 |
| ESC 2 | Right motor | A3 |
