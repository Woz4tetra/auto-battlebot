# ESP32 WiFi telemetry

`Esp32WifiTelemetry` reads the Mr Stabs Mk2 firmware's telemetry ring over WiFi (`GET /telemetry`).

- A worker thread polls with the seq after the last record it holds, so a reply carries every record since, and a link that drops costs time but no rows while the ring still holds them.
- The first poll after start or a robot reboot only learns where the ring's head is: history from before this client was listening is not replayed.
- Each record becomes an `Esp32DiagnosticsEvent` stamped with the reply's arrival time. All but the newest row of a poll that caught up are marked backfill, since their arrival time says when the poll ran, not when the robot sent them.
- Nothing blocks the caller on the network.
- The ring holds two minutes, so `poll_period` only sets how stale the newest row is when it lands, not whether rows are lost.

Tests `StartsAtHeadThenReadsEverythingNew` and `RebootResyncsToTheNewHead` cover the head-only first poll and the reboot resync.
