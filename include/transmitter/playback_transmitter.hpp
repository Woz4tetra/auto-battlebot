#pragma once

#include "transmitter/transmitter_interface.hpp"
#include <chrono>

namespace auto_battlebot
{
    class PlaybackTransmitter : public TransmitterInterface
    {
    public:
        PlaybackTransmitter(double init_delay_seconds = 0.0);

        bool initialize() override;
        CommandFeedback update() override;
        void send(VelocityCommand command) override;
        bool did_init_button_press() override;

    private:
        double init_delay_seconds_;
        bool initialized_;
        std::chrono::steady_clock::time_point start_time_;
        bool init_button_pressed_;
        bool init_button_done_pressing_;
    };

} // namespace auto_battlebot
