# ESP-NOW link (firmware)

`firmware/mr_stabs_mk2/src/espnow_link.cpp` receives drive frames and OTA updates on the WiFi task callbacks and the main loop.

## Shared state

- The diagnostics counters (`tx_ok`, `tx_fail`, `tx_errors`, `rx_rejected`, `rx_queue_full`) are `std::atomic<uint32_t>` with relaxed ordering. The WiFi task writes them and the diagnostics page only needs them roughly current.
- The allowed dongle list is read by `on_recv` on the WiFi task. Only `loop()` rewrites it, and only while disarmed.
- `allowed_count` drops to 0 before a rewrite and rises after it, so a frame racing a change is at worst refused once.

## OTA

- `ota_begin()` stamps `last_ota_frame_ms_` with the loop's `now_ms`, not `millis()`.
- `update()` compares against `now_ms`. A later `millis()` makes `now_ms - last_ota_frame_ms_` wrap and trips the silence abort at once.
