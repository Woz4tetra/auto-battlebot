#pragma once

#include "data_structures.hpp"

namespace auto_battlebot
{
    class TransmitterInterface
    {
    public:
        virtual ~TransmitterInterface() = default;
        virtual bool initialize() = 0;
        virtual void update() = 0;
        virtual void send(VelocityCommand command) = 0;
        virtual bool did_init_button_press() = 0;
    };

} // namespace auto_battlebot
