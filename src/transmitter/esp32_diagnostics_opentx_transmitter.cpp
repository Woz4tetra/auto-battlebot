#include "transmitter/esp32_diagnostics_opentx_transmitter.hpp"

#include <algorithm>
#include <cmath>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "foxglove_adapters/json_schemas.hpp"

namespace auto_battlebot {
namespace {
constexpr const char *kTopic = "/robot/esp32_diagnostics";
constexpr auto kHealthPeriod = std::chrono::seconds(1);

Esp32WifiTelemetryOptions to_options(const Esp32DiagnosticsOpenTxTransmitterConfiguration &c) {
    Esp32WifiTelemetryOptions options;
    options.host = c.esp32_host;
    options.port = c.esp32_port;
    options.retry_period =
        std::chrono::milliseconds(static_cast<int64_t>(std::llround(c.reconnect_period_s * 1e3)));
    return options;
}
}  // namespace

Esp32DiagnosticsOpenTxTransmitter::Esp32DiagnosticsOpenTxTransmitter(
    const Esp32DiagnosticsOpenTxTransmitterConfiguration &config,
    std::shared_ptr<ClockInterface> clock, std::shared_ptr<VizSink> sink,
    std::shared_ptr<McapRecorder> mcap_recorder)
    : OpenTxTransmitter(config, std::move(clock)),
      diagnostics_(std::make_unique<Esp32WifiTelemetry>(to_options(config))),
      channel_(kTopic, "json",
               VizSchema::jsonschema(foxglove_adapters::kEsp32DiagnosticsSchemaName,
                                     foxglove_adapters::kEsp32DiagnosticsSchema),
               false, std::move(sink), std::move(mcap_recorder)),
      health_logger_(DiagnosticsLogger::get_logger("esp32_diagnostics")) {}

Esp32DiagnosticsOpenTxTransmitter::~Esp32DiagnosticsOpenTxTransmitter() {
    // Stop first, so the worker is gone before the channel it feeds.
    diagnostics_->stop();
}

bool Esp32DiagnosticsOpenTxTransmitter::initialize() {
    diagnostics_->start();
    return OpenTxTransmitter::initialize();
}

CommandFeedback Esp32DiagnosticsOpenTxTransmitter::update() {
    CommandFeedback feedback = OpenTxTransmitter::update();
    publish_events();
    log_health();
    return feedback;
}

void Esp32DiagnosticsOpenTxTransmitter::publish_events() {
    for (const Esp32DiagnosticsEvent &event : diagnostics_->drain()) {
        if (last_robot_timestamp_ms_ && event.timestamp_ms > *last_robot_timestamp_ms_) {
            max_robot_gap_ms_ =
                std::max(max_robot_gap_ms_, event.timestamp_ms - *last_robot_timestamp_ms_);
        }
        last_robot_timestamp_ms_ = event.timestamp_ms;
        clock_offset_ms_ = static_cast<double>(event.host_receive_ns) * 1e-6 -
                           static_cast<double>(event.timestamp_ms);
        channel_.log(to_esp32_diagnostics_json(event), event.host_receive_ns);
    }
}

void Esp32DiagnosticsOpenTxTransmitter::log_health() {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_health_log_ < kHealthPeriod) return;
    const double elapsed_s = last_health_log_ == std::chrono::steady_clock::time_point{}
                                 ? 0.0
                                 : std::chrono::duration<double>(now - last_health_log_).count();
    last_health_log_ = now;

    const Esp32WifiTelemetryStats stats = diagnostics_->stats();
    const uint64_t new_events = stats.events - events_at_last_health_log_;
    events_at_last_health_log_ = stats.events;

    DiagnosticsData data;
    data["connected"] = static_cast<int>(stats.connected);
    data["events_per_second"] = elapsed_s > 0.0 ? static_cast<double>(new_events) / elapsed_s : 0.0;
    data["events"] = static_cast<int>(stats.events);
    data["polls"] = static_cast<int>(stats.polls);
    data["poll_failures"] = static_cast<int>(stats.poll_failures);
    data["parse_errors"] = static_cast<int>(stats.parse_errors);
    data["backfilled"] = static_cast<int>(stats.backfilled);
    data["lost_records"] = static_cast<int>(stats.lost_records);
    data["lag_records"] = static_cast<int>(stats.lag_records);
    data["ring_capacity"] = static_cast<int>(stats.ring_capacity);
    data["reboots"] = static_cast<int>(stats.reboots);
    data["dropped_events"] = static_cast<int>(stats.dropped_events);
    data["max_robot_gap_ms"] = static_cast<int>(max_robot_gap_ms_);
    if (clock_offset_ms_) data["clock_offset_ms"] = *clock_offset_ms_;
    max_robot_gap_ms_ = 0;

    if (stats.connected) {
        health_logger_->info("stream", data);
    } else {
        health_logger_->warning("stream", data, "not reaching the robot's telemetry ring");
    }
}

}  // namespace auto_battlebot
