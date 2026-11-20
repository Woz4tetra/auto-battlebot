#pragma once

#include <chrono>
#include <optional>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/config.hpp"
#include "transmitter/differential_drive_processor.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {
class PlaybackTransmitter : public TransmitterInterface {
   public:
    PlaybackTransmitter(const PlaybackTransmitterConfiguration &config,
                        std::shared_ptr<ClockInterface> clock);

    bool initialize() override;
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    TransmitterStatus get_status() const override {
        return {.connected = true, .receiving_channels = true};
    }

   private:
    double init_delay_seconds_;
    bool initialized_;
    std::shared_ptr<ClockInterface> clock_;
    std::optional<double> first_update_time_seconds_;
    bool init_button_pressed_;
    bool init_button_done_pressing_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    DifferentialDriveProcessor processor_;
    std::optional<BodyVelocity> last_processed_;
};

}  // namespace auto_battlebot
