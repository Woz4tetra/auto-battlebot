#pragma once

#include <espnow_protocol.h>

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "esp32_diagnostics/esp32_diagnostics_event.hpp"

namespace auto_battlebot {

/** One `GET /telemetry` reply from the Mr Stabs Mk2 firmware's telemetry ring. */
struct Esp32TelemetryBatch {
    espnow_protocol::TelemetryBatchHeader header{};
    std::vector<espnow_protocol::TelemetryPayload> records;
};

/**
 * Decode a reply body. Returns empty when the magic, version, or record size does not match this
 * build's `espnow_protocol.h`, or when the body length disagrees with the header's count.
 */
std::optional<Esp32TelemetryBatch> decode_esp32_telemetry_batch(std::string_view body);

/**
 * One ring record as a `/robot/esp32_diagnostics` event. The ring's `telemetry_seq` becomes `seq`;
 * the record has no AP client count, so `wifi_clients` reads 0. Non-finite floats become empty
 * optionals the way a `nan` CSV field does.
 */
Esp32DiagnosticsEvent to_esp32_diagnostics_event(const espnow_protocol::TelemetryPayload &record,
                                                 uint64_t host_receive_ns, bool backfill);

}  // namespace auto_battlebot
