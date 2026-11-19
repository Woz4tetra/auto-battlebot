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
                                                 uint64_t host_receive_wall_ns, bool backfill) {
    Esp32DiagnosticsEvent e;
    e.host_receive_wall_ns = host_receive_wall_ns;
    e.robot_millis = r.timestamp_ms;
    e.radio_connected = r.radio_connected != 0;
    e.armed = r.armed != 0;
    e.a_percent = r.a_percent;
    e.b_percent = r.b_percent;
    e.button_state = r.button_state != 0;
    e.flip_switch = r.flip_switch;
    e.left_cmd_post_mixer_pct = r.left_cmd;
    e.right_cmd_post_mixer_pct = r.right_cmd;
    e.accel_x = r.accel_x;
    e.accel_y = r.accel_y;
    e.accel_z = r.accel_z;
    e.is_upside_down = r.is_upside_down != 0;
    e.loop_us = r.loop_us;
    e.orientation_x_deg = r.orientation_x;
    e.orientation_y_deg = r.orientation_y;
    e.orientation_z_deg = r.orientation_z;
    e.pid_setpoint = r.pid_setpoint;
    e.pid_output = r.pid_output;
    e.vbat = finite(r.pack_volts);
    e.ibat = finite(r.pack_amps_discharge);
    e.yaw_rate_cw_deg_s = finite(r.yaw_rate_cw_dps);
    e.yaw_rate_cmd_cw_deg_s = finite(r.yaw_rate_cmd_cw_dps);
    e.guard = Esp32MotorGuard{
        .motor_rise_left_c = r.motor_rise_left_c,
        .motor_rise_right_c = r.motor_rise_right_c,
        .motor_speed_left_rad_s = r.motor_speed_left_rad_s,
        .motor_speed_right_rad_s = r.motor_speed_right_rad_s,
        .motor_current_left_a = r.motor_amps_est_left,
        .motor_current_right_a = r.motor_amps_est_right,
        .guard_scale = r.guard_scale,
        .guard_gain = r.guard_shunt_gain,
        .guard_state = static_cast<MotorGuardState>(r.guard_state),
        .guard_sensors = static_cast<MotorGuardSensors>(r.guard_sensors),
        .left_stalled = (r.stall_flags & 1) != 0,
        .right_stalled = (r.stall_flags & 2) != 0,
        .erpm_crc_errors_since_boot = r.erpm_crc_errors,
    };
    e.seq = r.telemetry_seq;
    e.backfill = backfill;
    return e;
}

}  // namespace auto_battlebot
