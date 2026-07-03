#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>

#include "channels/channels_parser.hpp"
#include "crsf/crsf_parser.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "serial/serial_port.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/config.hpp"
#include "transmitter/differential_drive_processor.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

class OpenTxTransmitter : public TransmitterInterface {
   public:
    OpenTxTransmitter(const OpenTxTransmitterConfiguration& config,
                      std::shared_ptr<ClockInterface> clock);

    /** Find and open the serial port, then enable telemetry and channel streaming. */
    bool initialize() override;

    /** Read available serial data, parse CRSF telemetry and channel updates. */
    CommandFeedback update() override;

    /** Convert velocity command (m/s, rad/s) to tank steering trainer channels. */
    void send(VelocityCommand command) override;

    /**
     * Returns true once per rising edge on the configured init button channel.
     * The channel value must cross init_button_threshold to count as a press.
     */
    bool did_init_button_press() override;

    bool is_connected() const override { return serial_.is_open(); }
    void enable() override;
    void disable() override;

   private:
    OpenTxTransmitterConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;
    DifferentialDriveProcessor processor_;

    // Linear-command slew limiter. Caps the normalized linear command rate so wheel-surface
    // acceleration stays below the slip/flip threshold (see config_.max_motor_rpm_per_sec).
    // Expressed in normalized units/second: max_motor_rpm_per_sec / max_motor_rpm. 0 = disabled.
    double max_linear_rate_per_sec_ = 0.0;
    double prev_linear_command_ = 0.0;
    std::optional<double> last_send_time_;
    SerialPort serial_;
    CrsfParser crsf_parser_;
    ChannelsParser channels_parser_;

    std::optional<std::array<int16_t, kMaxChannels>> latest_channels_;
    bool init_button_was_pressed_ = false;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point next_reconnect_attempt_ =
        std::chrono::steady_clock::now();

    bool reconnect_if_needed();
    void process_channel_updates(const std::vector<uint8_t>& bytes);
    void write_trainer_channels(int linear_value, int angular_value);

    /** Slew-limit the normalized linear command so wheel-surface acceleration stays under the
     *  configured RPM/s cap. Uses the logical clock for dt and passes the command through
     *  unchanged when the limiter is disabled or on the first tick after (re)enabling. */
    double limit_linear_acceleration(double linear_command);

    void handle_packet(const CrsfLinkStatistics& pkt);
    void handle_packet(const CrsfBattery& pkt);
    void handle_packet(const CrsfAttitude& pkt);
    void handle_packet(const CrsfFlightMode& pkt);

    /** Scale and clamp a normalized [-1, 1] value to the trainer range [-1000, 1000]. */
    static int to_trainer_value(double normalized);

    /**
     * Return the latest value for an RC channel, clamped to the trainer range [-1000, 1000].
     * Returns 0 if no channel data has been received yet or the index is out of range.
     */
    int get_channel_value(int channel_idx) const;
};

}  // namespace auto_battlebot
