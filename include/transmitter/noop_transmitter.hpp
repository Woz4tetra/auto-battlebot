#pragma once

#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot
{
    class NoopTransmitter : public TransmitterInterface
    {
    public:
        bool initialize() override { return true; }
        void update() override {}
        void send(VelocityCommand command) override {}
        bool did_init_button_press() override { return false; }
    };

} // namespace auto_battlebot
