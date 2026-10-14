#include "transmitter/espnow_transmitter.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "foxglove_adapters/json_schemas.hpp"

namespace auto_battlebot {
namespace {
constexpr auto kReconnectInterval = std::chrono::seconds(1);
// The dongle sends link stats every 100 ms, so this much silence means the port is dead: the
// dongle re-enumerated under an fd we still hold, or its firmware stopped writing.
constexpr auto kDongleSilence = std::chrono::milliseconds(500);
constexpr auto kHealthPeriod = std::chrono::seconds(1);
constexpr const char *kTopic = "/robot/espnow_telemetry";

void append_key(std::string &json, const char *key) {
    if (json.size() > 1) json += ',';
    json += '"';
    json += key;
    json += "\":";
}

void append_number(std::string &json, const char *key, double value) {
    append_key(json, key);
    if (!std::isfinite(value)) {
        json += "null";
        return;
    }
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.9g", value);
    json += buffer;
}

void append_integer(std::string &json, const char *key, int64_t value) {
    append_key(json, key);
    json += std::to_string(value);
}

void append_bool(std::string &json, const char *key, bool value) {
    append_key(json, key);
    json += value ? "true" : "false";
}

void append_null(std::string &json, const char *key) {
    append_key(json, key);
    json += "null";
}

}  // namespace

uint32_t espnow_monotonic_us() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

std::string to_espnow_telemetry_json(const espnow_protocol::TelemetryPayload &t,
                                     uint64_t host_receive_ns,
                                     std::optional<int64_t> round_trip_us) {
    std::string json = "{";
    append_integer(json, "host_receive_ns", static_cast<int64_t>(host_receive_ns));
    append_integer(json, "timestamp_ms", t.timestamp_ms);
    append_integer(json, "loop_us", t.loop_us);
    append_number(json, "a_percent", t.a_percent);
    append_number(json, "b_percent", t.b_percent);
    append_number(json, "left_cmd", t.left_cmd);
    append_number(json, "right_cmd", t.right_cmd);
    append_number(json, "accel_x", t.accel_x);
    append_number(json, "accel_y", t.accel_y);
    append_number(json, "accel_z", t.accel_z);
    append_number(json, "orientation_x", t.orientation_x);
    append_number(json, "orientation_y", t.orientation_y);
    append_number(json, "orientation_z", t.orientation_z);
    append_number(json, "pid_setpoint", t.pid_setpoint);
    append_number(json, "pid_output", t.pid_output);
    append_number(json, "vbat", t.vbat);
    append_number(json, "ibat", t.ibat);
    append_number(json, "yaw_rate", t.yaw_rate);
    append_number(json, "yaw_rate_cmd", t.yaw_rate_cmd);
    append_bool(json, "radio_connected", t.radio_connected != 0);
    append_bool(json, "armed", t.armed != 0);
    append_bool(json, "autonomy_enabled", t.autonomy_enabled != 0);
    append_bool(json, "button_state", t.button_state != 0);
    append_integer(json, "flip_switch", t.flip_switch);
    append_bool(json, "is_upside_down", t.is_upside_down != 0);
    append_bool(json, "espnow_active", t.espnow_active != 0);
    append_integer(json, "espnow_channel", t.espnow_channel);
    if (t.espnow_age_ms == UINT32_MAX) {
        append_null(json, "espnow_age_ms");
    } else {
        append_integer(json, "espnow_age_ms", t.espnow_age_ms);
    }
    append_integer(json, "espnow_seq", t.espnow_seq);
    append_integer(json, "jetson_tx_us", t.jetson_tx_us);
    append_integer(json, "espnow_lq", t.espnow_lq);
    if (round_trip_us) {
        append_integer(json, "round_trip_us", *round_trip_us);
    } else {
        append_null(json, "round_trip_us");
    }
    json += '}';
    return json;
}

EspNowTransmitter::EspNowTransmitter(const EspNowTransmitterConfiguration &config,
                                     std::shared_ptr<VizSink> sink,
                                     std::shared_ptr<McapRecorder> mcap_recorder)
    : config_(config),
      logger_(DiagnosticsLogger::get_logger("espnow_transmitter")),
      processor_(
          make_drive_processor(config.drive_processor,
                               {
                                   .velocity_saturation_limit = config.velocity_saturation_limit,
                                   .zero_deadzone_percent = config.zero_deadzone_percent,
                                   .lifted_deadzone_percent = config.lifted_deadzone_percent,
                                   .reverse_linear = config.reverse_linear_channel,
                                   .reverse_angular = config.reverse_angular_channel,
                               },
                               logger_)),
      telemetry_channel_(kTopic, "json",
                         VizSchema::jsonschema(foxglove_adapters::kEspNowTelemetrySchemaName,
                                               foxglove_adapters::kEspNowTelemetrySchema),
                         false, std::move(sink), std::move(mcap_recorder)) {}

bool EspNowTransmitter::initialize() { return reconnect_if_needed(); }

CommandFeedback EspNowTransmitter::update() {
    reconnect_if_needed();
    if (dongle_.is_open()) {
        const std::vector<uint8_t> bytes = dongle_.read_available();
        if (!bytes.empty()) parser_.feed(bytes.data(), bytes.size(), on_frame, this);
    }
    log_health();
    // Feedback comes from handset channels, which this transport does not carry.
    return {};
}

void EspNowTransmitter::send(VelocityCommand command) {
    reconnect_if_needed();
    if (!dongle_.is_open() || !enabled_) return;

    // No slew limit: the firmware rate-limits the arbitrated output.
    const auto channels = processor_->process(command);
    const espnow_protocol::CommandPayload payload{
        ++seq_,
        espnow_monotonic_us(),
        static_cast<float>(channels.channel_a),
        static_cast<float>(channels.channel_b),
    };
    if (!write_frame(espnow_protocol::MSG_COMMAND, &payload, sizeof(payload))) {
        logger_->warning("dongle_write_failed", {{"seq", static_cast<int>(payload.seq)}});
        dongle_.close();
        next_reconnect_attempt_ = std::chrono::steady_clock::now();
    }
}

void EspNowTransmitter::on_frame(void *context, uint8_t type, const uint8_t *payload,
                                 uint8_t length) {
    static_cast<EspNowTransmitter *>(context)->handle_frame(type, payload, length);
}

void EspNowTransmitter::handle_frame(uint8_t type, const uint8_t *payload, uint8_t length) {
    last_frame_time_ = std::chrono::steady_clock::now();
    switch (type) {
        case espnow_protocol::MSG_LINK_STATS: {
            if (length != sizeof(espnow_protocol::LinkStatsPayload)) return;
            espnow_protocol::LinkStatsPayload stats;
            std::memcpy(&stats, payload, sizeof(stats));
            link_stats_ = stats;
            break;
        }
        case espnow_protocol::MSG_TELEMETRY: {
            if (length != sizeof(espnow_protocol::TelemetryPayload)) return;
            espnow_protocol::TelemetryPayload telemetry;
            std::memcpy(&telemetry, payload, sizeof(telemetry));
            handle_telemetry(telemetry);
            break;
        }
        default:
            break;
    }
}

void EspNowTransmitter::handle_telemetry(const espnow_protocol::TelemetryPayload &telemetry) {
    telemetry_frames_++;
    std::optional<int64_t> round_trip_us;
    if (telemetry.espnow_age_ms != UINT32_MAX && telemetry.espnow_seq != 0) {
        // Unsigned subtraction wraps correctly across the 32-bit rollover. Each sample also holds
        // the robot's wait for its next 10 ms telemetry slot, so the per-period minimum is the
        // transport estimate.
        const int64_t sample = static_cast<int64_t>(espnow_monotonic_us() - telemetry.jetson_tx_us);
        round_trip_us = sample;
        if (!min_round_trip_us_ || sample < *min_round_trip_us_) min_round_trip_us_ = sample;
    }
    const uint64_t now_ns = wall_time_ns();
    telemetry_channel_.log(to_espnow_telemetry_json(telemetry, now_ns, round_trip_us), now_ns);
}

void EspNowTransmitter::log_health() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_health_log_ < kHealthPeriod) return;
    const double elapsed_s = last_health_log_ == std::chrono::steady_clock::time_point{}
                                 ? 0.0
                                 : std::chrono::duration<double>(now - last_health_log_).count();
    last_health_log_ = now;

    DiagnosticsData data;
    data["dongle_open"] = static_cast<int>(dongle_.is_open());
    data["crc_errors"] = static_cast<int>(parser_.crc_errors());
    data["telemetry_per_second"] =
        elapsed_s > 0.0
            ? static_cast<double>(telemetry_frames_ - telemetry_frames_at_last_log_) / elapsed_s
            : 0.0;
    telemetry_frames_at_last_log_ = telemetry_frames_;
    if (min_round_trip_us_) data["min_round_trip_us"] = static_cast<int>(*min_round_trip_us_);
    min_round_trip_us_.reset();
    if (link_stats_) {
        data["channel"] = static_cast<int>(link_stats_->channel);
        data["robot_known"] = static_cast<int>(link_stats_->robot_known);
        data["sent"] = static_cast<int>(link_stats_->sent);
        data["failed"] = static_cast<int>(link_stats_->failed);
        data["last_send_us"] = static_cast<int>(link_stats_->last_rtt_us);
        if (link_stats_at_last_log_) {
            // ACK rate over the last period. Only meaningful once robot_known: broadcast frames
            // report success with no ACK behind them.
            const uint32_t sent = link_stats_->sent - link_stats_at_last_log_->sent;
            const uint32_t acked = link_stats_->acked - link_stats_at_last_log_->acked;
            data["ack_percent"] = sent > 0 ? 100.0 * acked / sent : 0.0;
        }
        link_stats_at_last_log_ = link_stats_;
    }

    if (!dongle_.is_open()) {
        logger_->warning("link", data, "dongle not open at " + config_.dongle_serial_path);
    } else if (!link_stats_ || link_stats_->robot_known == 0) {
        logger_->warning("link", data, "dongle has not heard the robot");
    } else {
        logger_->info("link", data);
    }
}

bool EspNowTransmitter::reconnect_if_needed() {
    const auto now = std::chrono::steady_clock::now();

    if (dongle_.is_open()) {
        if (now - last_frame_time_ < kDongleSilence) return true;
        // Open but quiet. A port whose device went away reads empty forever, so only a fresh open
        // recovers it. Rate-limited like the open itself so an absent dongle is not thrashed.
        if (now < next_reconnect_attempt_) return true;
        const auto silent_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time_).count();
        logger_->warning("dongle_silent_reconnecting",
                         {{"silent_ms", static_cast<int>(silent_ms)}});
        dongle_.close();
    }

    if (now < next_reconnect_attempt_) return false;
    next_reconnect_attempt_ = now + kReconnectInterval;

    if (!dongle_.open(config_.dongle_serial_path)) {
        logger_->debug("dongle_not_found", {{"path", config_.dongle_serial_path}});
        return false;
    }
    logger_->info("dongle_opened", {{"path", config_.dongle_serial_path}});
    // One silence window of grace for the first link stats frame.
    last_frame_time_ = now;
    parser_ = espnow_protocol::SerialFrameParser();
    link_stats_.reset();
    link_stats_at_last_log_.reset();
    return true;
}

bool EspNowTransmitter::write_frame(uint8_t type, const void *payload, uint8_t length) {
    uint8_t frame[espnow_protocol::SERIAL_MAX_FRAME];
    const size_t size = espnow_protocol::encode_serial_frame(
        type, static_cast<const uint8_t *>(payload), length, frame);
    return dongle_.write(frame, size);
}

}  // namespace auto_battlebot
