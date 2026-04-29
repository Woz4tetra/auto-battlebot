#pragma once

#include <chrono>
#include <optional>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transmitter/config.hpp"
#include "transmitter/differential_drive_processor.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {
class PlaybackTransmitter : public TransmitterInterface {
   public:
    explicit PlaybackTransmitter(PlaybackTransmitterConfiguration &config);

    bool initialize() override;
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    bool is_connected() const override { return true; }

   private:
    double init_delay_seconds_;
    bool initialized_;
    std::chrono::steady_clock::time_point start_time_;
    bool init_button_pressed_;
    bool init_button_done_pressing_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    DifferentialDriveProcessor processor_;
    std::optional<DifferentialDriveProcessor::Result> last_processed_;
};

}  // namespace auto_battlebot
