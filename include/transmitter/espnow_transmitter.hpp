#pragma once

#include <espnow_protocol.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "diagnostics_logger/diagnostics_module_logger.hpp"
#include "mcap_recorder/mcap_recorder.hpp"
#include "publisher/output_channel.hpp"
#include "serial/serial_port.hpp"
#include "transmitter/config.hpp"
#include "transmitter/drive_processor_interface.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {

/**
 * Sends autonomous velocity commands to Mr Stabs Mk2 through the ESP-NOW USB dongle, cutting the
 * handset mixer and the Crossfire frame period out of the command path, and records the telemetry
 * the robot sends back on /robot/espnow_telemetry.
 *
 * Send-only for control. There is no handset channel input on this transport, so it reports no
 * command feedback, no init button, and no status; under CompositeTransmitter those come from the
 * OpenTX child. It applies no linear slew limit, and neither does the firmware: commands reach the
 * wheels unlimited.
 *
 * ESP-NOW carries no arm authority. The firmware substitutes the two channel values only after
 * every CRSF gate passes and only while SA (ch5) enables autonomy, and falls back to the trainer
 * channels when they go stale. See docs/plans/espnow_link_plan.md.
 */
class EspNowTransmitter : public TransmitterInterface {
   public:
    EspNowTransmitter(const EspNowTransmitterConfiguration &config, std::shared_ptr<VizSink> sink,
                      std::shared_ptr<McapRecorder> mcap_recorder);

    bool initialize() override;
    /** Drains telemetry and link stats from the dongle. Always returns empty feedback. */
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override { return false; }
    void enable() override { enabled_ = true; }
    void disable() override { enabled_ = false; }

   private:
    static void on_frame(void *context, uint8_t type, const uint8_t *payload, uint8_t length);
    void handle_frame(uint8_t type, const uint8_t *payload, uint8_t length);
    void handle_telemetry(const espnow_protocol::TelemetryPayload &telemetry);
    void log_health();
    bool reconnect_if_needed();
    bool write_frame(uint8_t type, const void *payload, uint8_t length);

    EspNowTransmitterConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::unique_ptr<DriveProcessorInterface> processor_;
    OutputChannel telemetry_channel_;

    SerialPort dongle_;
    espnow_protocol::SerialFrameParser parser_;
    uint32_t seq_ = 0;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point next_reconnect_attempt_ =
        std::chrono::steady_clock::now();

    std::optional<espnow_protocol::LinkStatsPayload> link_stats_;
    std::chrono::steady_clock::time_point last_frame_time_{};  // any frame from the dongle
    std::optional<espnow_protocol::LinkStatsPayload> link_stats_at_last_log_;
    uint64_t telemetry_frames_ = 0;
    uint64_t telemetry_frames_at_last_log_ = 0;
    /** Smallest telemetry round trip since the last health log. */
    std::optional<int64_t> min_round_trip_us_;
    std::chrono::steady_clock::time_point last_health_log_{};
};

/** Monotonic microseconds, truncated to 32 bits: the clock the command's jetson_tx_us and the
 *  telemetry echo of it share. */
uint32_t espnow_monotonic_us();

/** The /robot/espnow_telemetry JSON for one frame. Non-finite floats become null.
 *  round_trip_us is null until the robot has accepted a command. */
std::string to_espnow_telemetry_json(const espnow_protocol::TelemetryPayload &telemetry,
                                     uint64_t host_receive_ns,
                                     std::optional<int64_t> round_trip_us);

}  // namespace auto_battlebot
