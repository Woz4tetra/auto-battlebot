#pragma once

#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"
#include "data_structures/transmitter_status.hpp"

namespace auto_battlebot {
class TransmitterInterface {
   public:
    virtual ~TransmitterInterface() = default;
    virtual bool initialize() = 0;
    virtual CommandFeedback update() = 0;
    virtual void send(StickCommand command) = 0;
    virtual bool did_init_button_press() = 0;
    virtual TransmitterStatus get_status() const { return {}; }
    virtual void enable() {}
    virtual void disable() {}
};

}  // namespace auto_battlebot
