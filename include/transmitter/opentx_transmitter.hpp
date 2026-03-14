#pragma once

#include <array>
#include <memory>
#include <optional>

#include "channels/channels_parser.hpp"
#include "crsf/crsf_parser.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "serial/serial_port.hpp"
#include "transmitter/config.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

class OpenTxTransmitter : public TransmitterInterface {
   public:
    explicit OpenTxTransmitter(const OpenTxTransmitterConfiguration& config);

    /** Find and open the serial port, then enable telemetry and channel streaming. */
    bool initialize() override;

    /** Read available serial data, parse CRSF telemetry and channel updates. */
    CommandFeedback update() override;

    /** Send trainer commands for linear_x (ch3) and angular_z (ch0). */
    void send(VelocityCommand command) override;

    /**
     * Returns true once per rising edge on the configured init button channel.
     * The channel value must cross init_button_threshold to count as a press.
     */
    bool did_init_button_press() override;

    bool is_connected() const override { return serial_.is_open(); }

   private:
    OpenTxTransmitterConfiguration config_;
    SerialPort serial_;
    CrsfParser crsf_parser_;
    ChannelsParser channels_parser_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    std::optional<std::array<int16_t, kMaxChannels>> latest_channels_;
    bool init_button_was_pressed_ = false;

    /** Scale and clamp a normalized [-1, 1] value to the trainer range [-1000, 1000]. */
    static int to_trainer_value(double normalized);
};

}  // namespace auto_battlebot
