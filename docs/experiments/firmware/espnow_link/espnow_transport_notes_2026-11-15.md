# ESP-NOW transport notes

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/serial/serial_port.cpp`

Beside `fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);`:

Linux raises DTR and RTS together on open, and this never touches them. Changing them can reset an ESP32-S3 USB-Serial/JTAG device (the ESP-NOW dongle): DTR low with RTS high holds it in reset.

### `src/transmitter/espnow_transmitter.cpp`

Beside `append_integer(json, "imu_calib_sys", (t.imu_calibration >> 6) & 0x03);`:

CALIB_STAT packs four 0-3 levels: sys in bits 7-6, then gyro, accel, mag.

### `src/transmitter/espnow_transmitter.cpp`

Beside `const int64_t sample = static_cast<int64_t>(espnow_monotonic_us() - telemetry.jetson_tx_us);`:

Unsigned subtraction wraps correctly across the 32-bit rollover. Each sample also holds the robot's wait for its next 10 ms telemetry slot, so the per-period minimum is the transport estimate.

### `src/transmitter/espnow_transmitter.cpp`

Beside `const uint32_t sent = link_stats_->sent - link_stats_at_last_log_->sent;`:

ACK rate over the last period. Only meaningful once robot_known: broadcast frames report success with no ACK behind them.
