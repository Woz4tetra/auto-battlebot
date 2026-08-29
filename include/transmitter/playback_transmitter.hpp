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
    PlaybackTransmitter(PlaybackTransmitterConfiguration &config,
                        std::shared_ptr<ClockInterface> clock);

    bool initialize() override;
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    TransmitterStatus get_status() const override {
        return {.connected = true, .receiving_channels = true};
    }
    BehaviorMode behavior_mode() const override { return behavior_mode_; }

   private:
    double init_delay_seconds_;
    BehaviorMode behavior_mode_;
    bool initialized_;
    std::shared_ptr<ClockInterface> clock_;
    /** Logical time of the first update(), so the init button fires on a fixed frame rather than
     *  after a wall-clock delay. Under synchronous playback a wall-clock delay lands on a
     *  different frame every run, which makes field initialization, and therefore the whole run,
     *  irreproducible. */
    std::optional<double> start_time_seconds_;
    bool init_button_pressed_;
    bool init_button_done_pressing_;
    std::shared_ptr<DiagnosticsModuleLogger> logger_;
    DifferentialDriveProcessor processor_;
    std::optional<BodyVelocity> last_processed_;
};

}  // namespace auto_battlebot
