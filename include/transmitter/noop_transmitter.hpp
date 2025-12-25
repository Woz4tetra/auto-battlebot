#pragma once

#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot
{
    class NoopTransmitter : public TransmitterInterface
    {
    public:
        bool initialize() override { return true; }
        CommandFeedback update() override { return CommandFeedback{}; }
        void send([[maybe_unused]] VelocityCommand command) override {}
        bool did_init_button_press() override { return false; }
    };

} // namespace auto_battlebot
