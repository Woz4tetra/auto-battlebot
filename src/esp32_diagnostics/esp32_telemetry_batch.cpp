#include "esp32_diagnostics/esp32_telemetry_batch.hpp"

#include <cmath>
#include <cstring>

namespace auto_battlebot {
namespace {
std::optional<double> finite(float value) {
    if (!std::isfinite(value)) return std::nullopt;
    return static_cast<double>(value);
}
}  // namespace

std::optional<Esp32TelemetryBatch> decode_esp32_telemetry_batch(std::string_view body) {
    using espnow_protocol::TelemetryBatchHeader;
    using espnow_protocol::TelemetryPayload;
    Esp32TelemetryBatch batch;
    if (body.size() < sizeof(TelemetryBatchHeader)) return std::nullopt;
    std::memcpy(&batch.header, body.data(), sizeof(TelemetryBatchHeader));
    const TelemetryBatchHeader &h = batch.header;
    if (h.magic != espnow_protocol::TELEMETRY_BATCH_MAGIC ||
        h.version != espnow_protocol::TELEMETRY_BATCH_VERSION ||
        h.record_size != sizeof(TelemetryPayload)) {
        return std::nullopt;
    }
    if (body.size() !=
        sizeof(TelemetryBatchHeader) + static_cast<size_t>(h.count) * sizeof(TelemetryPayload)) {
        return std::nullopt;
    }
    batch.records.resize(h.count);
    if (h.count > 0) {
        std::memcpy(batch.records.data(), body.data() + sizeof(TelemetryBatchHeader),
                    h.count * sizeof(TelemetryPayload));
    }
    return batch;
}

Esp32DiagnosticsEvent to_esp32_diagnostics_event(const espnow_protocol::TelemetryPayload &r,
                                                 uint64_t host_receive_ns, bool backfill) {
    Esp32DiagnosticsEvent e;
    e.host_receive_ns = host_receive_ns;
    e.timestamp_ms = r.timestamp_ms;
    e.radio_connected = r.radio_connected != 0;
    e.armed = r.armed != 0;
    e.a_percent = r.a_percent;
    e.b_percent = r.b_percent;
    e.button_state = r.button_state != 0;
    e.flip_switch = r.flip_switch;
    e.left_cmd = r.left_cmd;
    e.right_cmd = r.right_cmd;
    e.accel_x = r.accel_x;
    e.accel_y = r.accel_y;
    e.accel_z = r.accel_z;
    e.is_upside_down = r.is_upside_down != 0;
    e.loop_us = r.loop_us;
    e.orientation_x = r.orientation_x;
    e.orientation_y = r.orientation_y;
    e.orientation_z = r.orientation_z;
    e.pid_setpoint = r.pid_setpoint;
    e.pid_output = r.pid_output;
    e.vbat = finite(r.vbat);
    e.ibat = finite(r.ibat);
    e.yaw_rate = finite(r.yaw_rate);
    e.yaw_rate_cmd = finite(r.yaw_rate_cmd);
    e.seq = r.telemetry_seq;
    e.backfill = backfill;
    return e;
}

}  // namespace auto_battlebot
