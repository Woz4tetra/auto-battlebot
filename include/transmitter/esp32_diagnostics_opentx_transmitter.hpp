#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "esp32_diagnostics/esp32_wifi_telemetry.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "transmitter/config.hpp"
#include "transmitter/opentx_transmitter.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

class Esp32DiagnosticsOpenTxTransmitter : public OpenTxTransmitter {
   public:
    Esp32DiagnosticsOpenTxTransmitter(const Esp32DiagnosticsOpenTxTransmitterConfiguration &config,
                                      std::shared_ptr<ClockInterface> clock,
                                      std::shared_ptr<VizSink> sink,
                                      std::shared_ptr<McapRecorder> mcap_recorder);
    ~Esp32DiagnosticsOpenTxTransmitter() override;

    bool initialize() override;

    CommandFeedback update() override;

   private:
    void publish_events();
    void log_health();

    std::unique_ptr<Esp32WifiTelemetry> diagnostics_;
    OutputChannel channel_;
    std::shared_ptr<DiagnosticsModuleLogger> health_logger_;

    std::chrono::steady_clock::time_point last_health_log_{};
    uint64_t events_at_last_health_log_ = 0;
    uint64_t max_robot_clock_step_since_health_log_ = 0;
    std::optional<uint64_t> last_robot_timestamp_ms_;
    std::optional<double> newest_arrival_lag_ms_;
};

}  // namespace auto_battlebot
