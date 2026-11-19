# ESP32 WiFi telemetry

`Esp32WifiTelemetry` reads the Mr Stabs Mk2 firmware's telemetry ring over WiFi (`GET /telemetry`).

- A worker thread polls with the seq after the last record it holds, so a reply carries every record since, and a link that drops costs time but no rows while the ring still holds them.
- The first poll after start or a robot reboot only learns where the ring's head is: history from before this client was listening is not replayed.
- Each record becomes an `Esp32DiagnosticsEvent` stamped with the reply's arrival time. All but the newest row of a poll that caught up are marked backfill, since their arrival time says when the poll ran, not when the robot sent them.
- Nothing blocks the caller on the network.
- The ring holds two minutes, so `poll_period` only sets how stale the newest row is when it lands, not whether rows are lost.

Tests `StartsAtHeadThenReadsEverythingNew` and `RebootResyncsToTheNewHead` cover the head-only first poll and the reboot resync.

## Event decoding

- `Esp32DiagnosticsEvent::host_receive_wall_ns` is the host wall clock (the MCAP log time base) when the line or reply arrived. `robot_millis` is `millis()` on the ESP32.
- `parse_esp32_diagnostics_csv` accepts 20 fields (no `vbat`), 21 (no `ibat`), 22 (no yaw rates), 24 (no guard or `seq`), or 37. It returns empty for any other count or for a field that does not parse as its type. `RejectsUnknownFieldCounts` covers it.
- The firmware prints `nan` for `vbat`, `ibat` or a yaw rate it has no value for, and the parser leaves that optional empty. The yaw loop commands `nan` when it is not running. Firmware that predates a field omits the column, so the optional is empty there too.
- `motor_speed_*_rad_s` is signed and NaN when the firmware has no estimate. `stall_flags` on the wire is bit 0 left, bit 1 right, and the event carries `left_stalled` and `right_stalled`. `MotorGuardState` and `MotorGuardSensors` match `motor_guard::State` and `motor_guard::Sensors` in `firmware/mr_stabs_mk2/lib/motor_guard/motor_guard.h`.
- `to_esp32_diagnostics_json` writes non-finite numbers as `null`. A bare `nan` makes the Foxglove JSON payload unparseable. `NonFiniteBecomesNull` covers it. The JSON keys keep the firmware's names (`left_cmd`, `orientation_x`, `motor_amps_left`), not the C++ member names.
- `ParsesLinePrintedByFirmwareFormat` reads the `snprintf` format string out of `firmware/mr_stabs_mk2/src/diagnostics_server.cpp` and checks it against the 37 columns the parser expects, so a firmware column change fails the host test.
- `decode_esp32_telemetry_batch` returns empty when the magic, version or record size differs from this build's `espnow_protocol.h`, or when the body length disagrees with the header count. `RejectsWrongMagicVersionSizeOrLength` covers it.
- `to_esp32_diagnostics_event` takes `seq` from the ring's `telemetry_seq`. The ring record has no AP client count, so `wifi_clients` reads 0. Non-finite floats become empty optionals the way a `nan` CSV field does. `RecordBecomesDiagnosticsEvent` covers it.
- `esp32_host` must be an IPv4 literal: the transmitter config rejects anything else at parse, and no DNS lookup happens.

## Batch header

Every `GET /telemetry` reply starts with a `TelemetryBatchHeader` (`firmware/shared/espnow_protocol/src/espnow_protocol.h`).

- `record_size` is `sizeof(TelemetryPayload)`. The client drops a reply whose size differs, since that means the robot and the app were built from different headers.
- `boot_id` is random per boot. A change means the seqs restarted at 0, and `Esp32WifiTelemetryStats::boot_id_changes` counts them (published as `reboots`).
- `capacity` is how many records the ring holds. `next_seq` is the seq the next record gets, so `next_seq - 1` is the newest. `oldest_seq` is the oldest seq still in the ring. `first_seq` is the seq of the first record in this reply.
- A reply carries at most `TELEMETRY_BATCH_MAX` (256) records. The client asks again at once when a reply comes back full, instead of waiting a `poll_period`. `FullBatchIsFollowedByAnImmediateRequest` covers it.

## Firmware ring concurrency

`firmware/mr_stabs_mk2/lib/telemetry_ring` is a fixed-size ring with one writer and any number of readers on other tasks or cores.

- Neither side locks. The control loop pushes without waiting on a reader.
- A reader throws away any record the writer overwrote while it was copying, the way a seqlock does.
- `push()` is non-const and `read()` is const, so readers hold a `const` ring and cannot write.
- Capacity 0 makes every push a no-op. `storage` must hold `capacity` records and outlive the ring.
