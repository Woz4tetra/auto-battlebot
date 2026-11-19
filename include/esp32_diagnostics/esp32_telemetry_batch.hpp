#pragma once

#include <espnow_protocol.h>

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "esp32_diagnostics/esp32_diagnostics_event.hpp"

namespace auto_battlebot {

struct Esp32TelemetryBatch {
    espnow_protocol::TelemetryBatchHeader header{};
    std::vector<espnow_protocol::TelemetryPayload> records;
};

std::optional<Esp32TelemetryBatch> decode_esp32_telemetry_batch(std::string_view body);

Esp32DiagnosticsEvent to_esp32_diagnostics_event(const espnow_protocol::TelemetryPayload &record,
                                                 uint64_t host_receive_wall_ns, bool backfill);

}  // namespace auto_battlebot
