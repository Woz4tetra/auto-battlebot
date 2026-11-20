# ESP-NOW link (firmware)

`firmware/mr_stabs_mk2/src/espnow_link.cpp` receives drive frames and OTA updates on the WiFi task callbacks and the main loop.

## Shared state

- The diagnostics counters (`tx_ok`, `tx_fail`, `tx_errors`, `rx_rejected`, `rx_queue_full`) are `std::atomic<uint32_t>` with relaxed ordering. The WiFi task writes them and the diagnostics page only needs them roughly current.
- The allowed dongle list is read by `on_recv` on the WiFi task. Only `loop()` rewrites it, and only while disarmed.
- `allowed_count` drops to 0 before a rewrite and rises after it, so a frame racing a change is at worst refused once.

## Opening the USB port

`auto_battlebot/robot_link/esp32_serial.py` `open_esp32_serial()` opens the dongle and robot USB ports for `espnow_ota.py`, `espnow_diag.py` and `usb_dashboard.py`.

- The ESP32-S3 USB-Serial/JTAG unit resets the chip when DTR goes low while RTS is high. Two resets within about 500 ms drop an Adafruit board into its TinyUF2 bootloader.
- Linux raises DTR and RTS together when the device node opens. pyserial 3.5 (`serialposix.Serial.open`) then applies its stored DTR state and RTS state with one ioctl each, DTR first. Both default to high.
- The helper sets `dtr = True` and `rts = True` before `open()`, so the lines never leave the kernel's high/high state. Setting either low before `open()` makes pyserial clear it after the kernel raised both.
- `usb_dashboard.py` opens the robot port the same way, with `exclusive=False`. That path has not been checked on hardware.
- `reset_dongle()` in `espnow_diag.py` is the one deliberate reset: DTR low, then RTS low, then both high again.
- The scripts import `auto_battlebot`, so they need the repo venv.

## OTA

- `ota_begin()` stamps `last_ota_frame_ms_` with the loop's `now_ms`, not `millis()`.
- `update()` compares against `now_ms`. A later `millis()` makes `now_ms - last_ota_frame_ms_` wrap and trips the silence abort at once.
- `scripts/espnow_ota.py` sends 240-byte chunks (`OTA_CHUNK_DATA`) with at most 16 in flight past the last acknowledged offset.
- The flasher waits 0.5 s for progress before it rewinds to the acknowledged offset. A flash block erase takes about 150 ms and stalls acknowledgements, so the timeout sits above it.
- When the robot drops a chunk, every chunk behind it in flight draws the same next offset. The flasher goes back once per offset within 0.1 s, so one drop does not rewind the window once per stale acknowledgement.

- `espnow_ota.py` has no rollback. If the new image installs but does not report running for 10 s with the dongle in range, the tool raises `OtaError` and the robot is recovered over USB with `scripts/upload`.

## Dongle build envs

`firmware/espnow_dongle/platformio.ini` has one env, `qtpy_nopsram`: the development dongle, an Adafruit QT Py ESP32-S3 with 8 MB flash and no PSRAM. The match dongle will be an ESP32-S3 with a u.FL antenna. Add an env with its board and nothing else changes.

## Workarounds

- Dongle send timeout: `service_radio()` in `firmware/espnow_dongle/src/main.cpp` clears the in-flight send and counts a failure after `SEND_CALLBACK_TIMEOUT_MS` (50 ms) without `on_sent`. A send callback that never arrives would wedge the queue. ESP-NOW reports every send within a few ms, so this only covers a driver hiccup. That the driver drops callbacks is unverified.
- Dongle USB re-arm: HWCDC's ISR disables the `IN_EMPTY` interrupt each time it fires, and only `Serial.write()` turns it back on. Skipping `write()` while the buffer is full leaves it off for good once the host stops reading, and after a USB reset nothing drains the buffer again, so the dongle goes silent until power cycled. The dongle re-arms the interrupt the way `write()` does. `HWCDC.cpp:129` disables it in the ISR and `write()` re-enables it at 437 and 456. The silence was not reproduced.
- Diagnostics web page (`firmware/espnow_dongle/scripts/espnow_diag_web/app.js`): Space sets neutral wherever focus is. The handlers run in the capture phase on both key events, so a focused button or slider never also acts on Space. A hidden tab may stop running timers, so the page drops to neutral on `visibilitychange` instead of relying on the server deadman (`COMMAND_DEADMAN_S` in `espnow_diag.py`).
- Channel change: after every channel change `espnow_link.cpp` re-adds the paired peers and resets the ESP-NOW rate to 6 Mbps, because peers and the rate do not survive `esp_wifi_set_channel`. Unverified.
- Link role: `espnow_link.h` holds no authority. `main.cpp` substitutes the drive command only after every CRSF gate passes, so ESP-NOW can never arm the robot or extend its armed window. SA (ch5) down is the pilot's "accept autonomous commands" position, and SA up forces the stick values on ch0/ch1. With substitution false the link is in shadow mode: commands are received and reported but never drive. The ZED Box sends over ESP-NOW alone, so the Jetson cannot drive in that mode.
- Nothing rate-limits the arbitrated output. A source switch, an ESP-NOW dropout, or an orientation flip steps the wheels in one tick.
- `pair()` adds a dongle to the paired list in NVS and works only disarmed, because the NVS write stalls both cores for a few ms. `fill_status()` leaves `enabled`, `substitution` and `driving` to the caller, which owns those flags. The receive counters are read from `loop()` context.
- An ESP-NOW firmware transfer owns the loop: motors stopped, no control, same as ArduinoOTA. It starts only disarmed, and arming aborts it inside `update()`. Tuning mode counts as armed for this check, so no transfer starts while the ESC pins belong to the bootloader link. Tuning mode also refuses a firmware upload for that reason.
- ArduinoOTA `handle()` runs the whole upload inside one `loop()` pass, so `onStart` calls `disableLoopWDT()` and `onError` re-enables it.
- Before the setup-frame timer is deleted, `main.cpp` waits `delay(10)` after `esp_timer_stop` so a callback already running finishes before `loop()` sends its own DShot frames. Unverified.
