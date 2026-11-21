# Mr Stabs Mk2 firmware telemetry and services

## Batch header

The Mr Stabs Mk2 firmware serves its telemetry ring over WiFi at `GET /telemetry`. No C++ client reads it. Every reply starts with a `TelemetryBatchHeader` (`firmware/shared/espnow_protocol/src/espnow_protocol.h`), encoded by `encode_telemetry_batch` in `firmware/shared/espnow_protocol/src/telemetry_batch.h`.

- `record_size` is `sizeof(TelemetryPayload)`. A reader built from a different header sees a different size and must drop the reply.
- `boot_id` is random per boot. A change means the seqs restarted at 0.
- `capacity` is how many records the ring holds. `next_seq` is the seq the next record gets, so `next_seq - 1` is the newest. `oldest_seq` is the oldest seq still in the ring. `first_seq` is the seq of the first record in this reply.
- A reply carries at most `TELEMETRY_BATCH_MAX` (256) records.
- The ring holds two minutes of records.

## Firmware ring concurrency

`firmware/mr_stabs_mk2/lib/telemetry_ring` is a fixed-size ring with one writer and any number of readers on other tasks or cores.

- Neither side locks. The control loop pushes without waiting on a reader.
- A reader throws away any record the writer overwrote while it was copying, the way a seqlock does.
- `push()` is non-const and `read()` is const, so readers hold a `const` ring and cannot write.
- Capacity 0 makes every push a no-op. `storage` must hold `capacity` records and outlive the ring.

## Wi-Fi manager and firmware services

- The core would otherwise write the station config to flash on every `WiFi.begin()`, and a flash write stalls the control loop, so the settings live in our own NVS namespace. Settings writes go to flash too, so the diagnostics page changes them only while disarmed. The snapshot the page shows never carries passwords. The task owns the manager's state once `begin()` returns, and `snapshot()` reads it under `_lock`.
- Core dump erase writes flash, so it waits for the loop and refuses while armed.
- CRSF telemetry: values go back to the radio as CRSF sensors, and NaN fields are sent as 0.
- BNO055 (`updown_sensor.cpp`): samples read the gyro, Euler, linear and gravity registers through `read_registers` and decode them in `bno055_decode`. A failed read, or a failed per-sample CHIP_ID check, marks the sensor lost and reads stop until `begin()` succeeds. The gyro unwrap resets on reconnect.

## USB bridge

- `write_whole` writes a frame in one call once the USB ring buffer has room for all of it, so log lines and frames never split each other. It returns false when the laptop stopped reading.
- Arduino's `log_e()` and friends write one character at a time through the ROM putc hook, from any task or core, with no lock, so they would land in the middle of a frame. The bridge collects them and lets the bridge task write whole lines between frames.
- The bridge task runs on core 1, where `Serial.begin()` put the USB-Serial/JTAG interrupt. Writing from core 0 raced it and lost whole 64-byte packets: 2 to 5 damaged frames per 10 kB file, none on core 1 (236 frames checked on 2026-10-23).
