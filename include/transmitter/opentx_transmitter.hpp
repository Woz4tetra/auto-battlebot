#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
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

using HardwareTimePoint = std::chrono::steady_clock::time_point;

class OpenTxTransmitter : public TransmitterInterface {
   public:
    using DeviceFinder = std::function<std::optional<std::string>()>;

    OpenTxTransmitter(const OpenTxTransmitterConfiguration& config,
                      std::shared_ptr<ClockInterface> clock,
                      DeviceFinder find_device = find_opentx_stm32_cdc_device);

    bool initialize() override;

    CommandFeedback update() override;

    void send(StickCommand command) override;

    bool did_init_button_press() override;

    TransmitterStatus get_status() const override;

    void enable() override;
    void disable() override;

   private:
    OpenTxTransmitterConfiguration config_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    std::shared_ptr<ClockInterface> clock_;
    std::unique_ptr<DriveProcessorInterface> processor_;
    DeviceFinder find_device_;

    double max_normalized_linear_rate_per_s_ = 0.0;
    double prev_linear_command_ = 0.0;
    std::optional<double> last_send_time_;
    SerialPort serial_;
    CrsfParser crsf_parser_;
    ChannelsParser channels_parser_;

    std::optional<std::array<int16_t, kMaxChannels>> latest_channels_;
    bool init_button_was_pressed_ = false;
    std::optional<std::tuple<bool, bool>> last_logged_switch_states_;
    bool enabled_ = false;
    std::chrono::steady_clock::time_point next_reconnect_attempt_ =
        std::chrono::steady_clock::now();
    HardwareTimePoint last_channel_time_{};

    bool channels_fresh() const;
    bool reconnect_if_needed();
    void process_channel_updates(const std::vector<uint8_t>& bytes);

    void log_switch_states();
    void write_trainer_channels(int channel_a_value, int channel_b_value);

    double limit_linear_acceleration(double linear_command);

    void handle_packet(const CrsfLinkStatistics& pkt);
    void handle_packet(const CrsfBattery& pkt);
    void handle_packet(const CrsfAttitude& pkt);
    void handle_packet(const CrsfFlightMode& pkt);

    static int to_trainer_value(double normalized);

    int get_channel_value(int channel_idx) const;
};

}  // namespace auto_battlebot
