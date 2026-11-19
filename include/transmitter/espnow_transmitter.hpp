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

class EspNowTransmitter : public TransmitterInterface {
   public:
    EspNowTransmitter(const EspNowTransmitterConfiguration &config, std::shared_ptr<VizSink> sink,
                      std::shared_ptr<McapRecorder> mcap_recorder);

    bool initialize() override;
    /** Drains telemetry and link stats from the dongle. Feedback is the robot's latest drive
     *  command, empty once telemetry is older than kTelemetryStale. */
    CommandFeedback update() override;
    TransmitterStatus get_status() const override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override { return false; }
    void enable() override { enabled_ = true; }
    void disable() override;

   private:
    static void on_frame(void *context, uint8_t type, const uint8_t *payload, uint8_t length);
    void handle_frame(uint8_t type, const uint8_t *payload, uint8_t length);
    void handle_telemetry(const espnow_protocol::TelemetryPayload &telemetry);
    void log_health();
    bool reconnect_if_needed();
    bool write_frame(uint8_t type, const void *payload, uint8_t length);
    /** Caps how fast the normalized linear command changes. Passes it through while the limit is
     *  off or SA is off, holding a standstill baseline, so engaging SA ramps up from zero. */
    double limit_linear_acceleration(double linear_command);

    EspNowTransmitterConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::unique_ptr<DriveProcessorInterface> processor_;
    OutputChannel telemetry_channel_;

    SerialPort dongle_;
    espnow_protocol::SerialFrameParser parser_;
    uint32_t seq_ = 0;
    bool enabled_ = false;
    /** Normalized units per second: max_motor_rpm_per_s / max_motor_rpm, 0 when that is unset. */
    double max_linear_rate_per_sec_ = 0.0;
    double prev_linear_command_ = 0.0;
    std::optional<std::chrono::steady_clock::time_point> last_send_time_;
    std::chrono::steady_clock::time_point next_reconnect_attempt_ =
        std::chrono::steady_clock::now();

    std::optional<espnow_protocol::LinkStatsPayload> link_stats_;
    std::chrono::steady_clock::time_point last_dongle_frame_time_{};
    std::optional<espnow_protocol::LinkStatsPayload> link_stats_at_last_log_;
    uint64_t telemetry_frames_ = 0;
    uint64_t telemetry_frames_at_last_log_ = 0;
    std::optional<int64_t> min_round_trip_us_since_health_log_;
    std::chrono::steady_clock::time_point last_health_log_{};

    std::optional<espnow_protocol::TelemetryPayload> latest_telemetry_;
    std::chrono::steady_clock::time_point latest_telemetry_time_{};
};

constexpr std::chrono::milliseconds kTelemetryStale{100};

/** The drive command a telemetry frame reports, as the two channel values on the wire (what
 *  send() would write, or the handset's ch0/ch1). Undoes the firmware's upside-down flip and its
 *  ch0 negation (crsf_bridge reads a_percent = -ch0). */
DriveProcessorInterface::Channels espnow_telemetry_channels(
    const espnow_protocol::TelemetryPayload &telemetry);

/** Monotonic microseconds, truncated to 32 bits: the clock the command's jetson_tx_us and the
 *  telemetry echo of it share. */
uint32_t espnow_monotonic_us();

std::string to_espnow_telemetry_json(const espnow_protocol::TelemetryPayload &telemetry,
                                     uint64_t host_receive_ns,
                                     std::optional<int64_t> round_trip_us);

}  // namespace auto_battlebot
