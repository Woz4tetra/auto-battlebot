#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <tuple>

#include "channels/channels_parser.hpp"
#include "crsf/crsf_parser.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "serial/serial_port.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/config.hpp"
#include "transmitter/drive_processor_interface.hpp"
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

    /** Convert velocity command (m/s, rad/s) to trainer channels via the configured processor. */
    void send(VelocityCommand command) override;

    /**
     * Returns true once per rising edge on the configured init button channel.
     * The channel value must cross init_button_threshold to count as a press.
     */
    bool did_init_button_press() override;

    TransmitterStatus get_status() const override;

    /** Level-triggered behavior mode switch: RUN_AWAY while the configured channel reads
     *  above behavior_mode_threshold, ATTACK otherwise (including before any channel frame
     *  has arrived, so a disconnected radio reports ATTACK). */
    BehaviorMode behavior_mode() const override;

    void enable() override;
    void disable() override;

   private:
    OpenTxTransmitterConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;
    std::unique_ptr<DriveProcessorInterface> processor_;

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
    /** Decoded (behavior_mode, autonomy_enabled, init_button) flags as last written to the
     *  switch_states log entry; empty until the first channel frame is logged. */
    std::optional<std::tuple<bool, bool, bool>> last_logged_switch_states_;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point next_reconnect_attempt_ =
        std::chrono::steady_clock::now();
    /** Time of the last complete channel update. Hardware timeout, so this uses
     *  std::chrono::steady_clock directly rather than the logical clock_ (see
     *  time/clock_interface.hpp). */
    std::chrono::steady_clock::time_point last_channel_time_{};

    /** True while complete 32-channel updates keep arriving. Seeded on port open so a fresh
     *  (re)connect gets one timeout window of grace before it reports a stall. Assumes the
     *  port is open; callers pair it with serial_.is_open(). */
    bool channels_fresh() const;
    bool reconnect_if_needed();
    void process_channel_updates(const std::vector<uint8_t>& bytes);

    /** Log decoded switch flags (with the raw channel value next to each) once on the first
     *  channel frame and then on every change, so recordings show what the transmitter
     *  believed even when the driver never touched a switch. Kept out of behavior_mode() so
     *  that stays side-effect free. */
    void log_switch_states();
    void write_trainer_channels(int channel_a_value, int channel_b_value);

    /** Slew-limit the normalized linear command so wheel-surface acceleration stays under the
     *  configured RPM/s cap. Uses the logical clock for dt and passes the command through
     *  unchanged when the limiter is disabled or on the first tick after (re)enabling. The limit
     *  only applies while the trainer enable channel (config_.trainer_enable_channel) reads
     *  negative; until then the baseline is held at standstill so motion ramps from zero once the
     *  trainer switch engages. */
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
