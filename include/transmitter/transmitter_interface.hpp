#pragma once

#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"

namespace auto_battlebot
{
    class TransmitterInterface
    {
    public:
        virtual ~TransmitterInterface() = default;
        virtual bool initialize() = 0;
        virtual CommandFeedback update() = 0;
        virtual void send(VelocityCommand command) = 0;
        virtual bool did_init_button_press() = 0;
    };

} // namespace auto_battlebot
