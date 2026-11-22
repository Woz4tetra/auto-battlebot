# Mr Stabs Mk2 firmware services

## Wi-Fi manager and firmware services

- The core would otherwise write the station config to flash on every `WiFi.begin()`, and a flash write stalls the control loop, so the settings live in our own NVS namespace. Settings writes go to flash too, so the diagnostics page changes them only while disarmed. The snapshot the page shows never carries passwords. The task owns the manager's state once `begin()` returns, and `snapshot()` reads it under `_lock`.
- Core dump erase writes flash, so it waits for the loop and refuses while armed.
- CRSF telemetry: values go back to the radio as CRSF sensors, and NaN fields are sent as 0.
- BNO055 (`updown_sensor.cpp`): samples read the gyro, Euler, linear and gravity registers through `read_registers` and decode them in `bno055_decode`. A failed read, or a failed per-sample CHIP_ID check, marks the sensor lost and reads stop until `begin()` succeeds. The gyro unwrap resets on reconnect.

## USB bridge

- `write_whole` writes a frame in one call once the USB ring buffer has room for all of it, so log lines and frames never split each other. It returns false when the laptop stopped reading.
- Arduino's `log_e()` and friends write one character at a time through the ROM putc hook, from any task or core, with no lock, so they would land in the middle of a frame. The bridge collects them and lets the bridge task write whole lines between frames.
- The bridge task runs on core 1, where `Serial.begin()` put the USB-Serial/JTAG interrupt. Writing from core 0 raced it and lost whole 64-byte packets: 2 to 5 damaged frames per 10 kB file, none on core 1 (236 frames checked on 2026-10-23).
