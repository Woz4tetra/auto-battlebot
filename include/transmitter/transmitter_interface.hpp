#pragma once

#include "data_structures.hpp"
#include "data_structures/command_feedback.hpp"

namespace auto_battlebot {
class TransmitterInterface {
   public:
    virtual ~TransmitterInterface() = default;
    virtual bool initialize() = 0;
    virtual CommandFeedback update() = 0;
    virtual void send(VelocityCommand command) = 0;
    virtual bool did_init_button_press() = 0;
    /** Whether the transmitter is connected (e.g. to hardware or simulation). */
    virtual bool is_connected() const { return false; }
    /** Enable autonomy: start sending commands to hardware/sim. */
    virtual void enable() {}
    /** Disable autonomy: stop sending commands to hardware/sim. */
    virtual void disable() {}
};

}  // namespace auto_battlebot
