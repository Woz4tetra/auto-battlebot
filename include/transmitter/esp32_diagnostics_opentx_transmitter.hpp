#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "esp32_diagnostics/esp32_wifi_diagnostics.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "transmitter/config.hpp"
#include "transmitter/opentx_transmitter.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

/**
 * OpenTxTransmitter that also records the robot firmware's diagnostics stream.
 *
 * The radio side is the base class, untouched. On top of it an Esp32WifiDiagnostics worker holds
 * the ESP32's event stream open, and update() drains what arrived since the last tick into one
 * `/robot/esp32_diagnostics` JSON message per firmware event, logged at the host arrival time.
 * Stream health goes to the `esp32_diagnostics` diagnostics module once a second.
 */
class Esp32DiagnosticsOpenTxTransmitter : public OpenTxTransmitter {
   public:
    Esp32DiagnosticsOpenTxTransmitter(const Esp32DiagnosticsOpenTxTransmitterConfiguration &config,
                                      std::shared_ptr<ClockInterface> clock,
                                      std::shared_ptr<VizSink> sink,
                                      std::shared_ptr<McapRecorder> mcap_recorder);
    ~Esp32DiagnosticsOpenTxTransmitter() override;

    /** Starts the diagnostics worker, then opens the radio. The base class calls initialize()
     *  again on every serial reconnect, so starting the worker is idempotent. */
    bool initialize() override;

    CommandFeedback update() override;

   private:
    void publish_events();
    void log_health();

    std::unique_ptr<Esp32WifiDiagnostics> diagnostics_;
    OutputChannel channel_;
    std::shared_ptr<DiagnosticsModuleLogger> health_logger_;

    std::chrono::steady_clock::time_point last_health_log_{};
    uint64_t events_at_last_health_log_ = 0;
    /** Largest robot-clock step between consecutive events since the last health log. */
    uint64_t max_robot_gap_ms_ = 0;
    std::optional<uint64_t> last_robot_timestamp_ms_;
    /** Host arrival minus robot stamp for the newest event, milliseconds. */
    std::optional<double> clock_offset_ms_;
};

}  // namespace auto_battlebot
