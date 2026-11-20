# ESP-NOW link (firmware)

`firmware/mr_stabs_mk2/src/espnow_link.cpp` receives drive frames and OTA updates on the WiFi task callbacks and the main loop.

## Shared state

- The diagnostics counters (`tx_ok`, `tx_fail`, `tx_errors`, `rx_rejected`, `rx_queue_full`) are `std::atomic<uint32_t>` with relaxed ordering. The WiFi task writes them and the diagnostics page only needs them roughly current.
- The allowed dongle list is read by `on_recv` on the WiFi task. Only `loop()` rewrites it, and only while disarmed.
- `allowed_count` drops to 0 before a rewrite and rises after it, so a frame racing a change is at worst refused once.

## OTA

- `ota_begin()` stamps `last_ota_frame_ms_` with the loop's `now_ms`, not `millis()`.
- `update()` compares against `now_ms`. A later `millis()` makes `now_ms - last_ota_frame_ms_` wrap and trips the silence abort at once.
- `scripts/espnow_ota.py` sends 240-byte chunks (`OTA_CHUNK_DATA`) with at most 16 in flight past the last acknowledged offset.
- The flasher waits 0.5 s for progress before it rewinds to the acknowledged offset. A flash block erase takes about 150 ms and stalls acknowledgements, so the timeout sits above it.
- When the robot drops a chunk, every chunk behind it in flight draws the same next offset. The flasher goes back once per offset within 0.1 s, so one drop does not rewind the window once per stale acknowledgement.

- `espnow_ota.py` has no rollback. If the new image installs but does not report running for 10 s with the dongle in range, the tool raises `OtaError` and the robot is recovered over USB with `scripts/upload`.
