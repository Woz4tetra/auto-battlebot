# ESP32 WiFi telemetry

`Esp32WifiTelemetry` reads the Mr Stabs Mk2 firmware's telemetry ring over WiFi (`GET /telemetry`).

- A worker thread polls with the seq after the last record it holds, so a reply carries every record since, and a link that drops costs time but no rows while the ring still holds them.
- The first poll after start or a robot reboot only learns where the ring's head is: history from before this client was listening is not replayed.
- Each record becomes an `Esp32DiagnosticsEvent` stamped with the reply's arrival time. All but the newest row of a poll that caught up are marked backfill, since their arrival time says when the poll ran, not when the robot sent them.
- Nothing blocks the caller on the network.
- The ring holds two minutes, so `poll_period` only sets how stale the newest row is when it lands, not whether rows are lost.

Tests `StartsAtHeadThenReadsEverythingNew` and `RebootResyncsToTheNewHead` cover the head-only first poll and the reboot resync.

## Batch header

Every `GET /telemetry` reply starts with a `TelemetryBatchHeader` (`firmware/shared/espnow_protocol/src/espnow_protocol.h`).

- `record_size` is `sizeof(TelemetryPayload)`. The client drops a reply whose size differs, since that means the robot and the app were built from different headers.
- `boot_id` is random per boot. A change means the seqs restarted at 0.
- `capacity` is how many records the ring holds. `next_seq` is the seq the next record gets, so `next_seq - 1` is the newest. `oldest_seq` is the oldest seq still in the ring. `first_seq` is the seq of the first record in this reply.
- A reply carries at most `TELEMETRY_BATCH_MAX` (256) records. The client asks again at once when a reply comes back full, instead of waiting a `poll_period`. `FullBatchIsFollowedByAnImmediateRequest` covers it.

## Firmware ring concurrency

`firmware/mr_stabs_mk2/lib/telemetry_ring` is a fixed-size ring with one writer and any number of readers on other tasks or cores.

- Neither side locks. The control loop pushes without waiting on a reader.
- A reader throws away any record the writer overwrote while it was copying, the way a seqlock does.
- `push()` is non-const and `read()` is const, so readers hold a `const` ring and cannot write.
- Capacity 0 makes every push a no-op. `storage` must hold `capacity` records and outlive the ring.
