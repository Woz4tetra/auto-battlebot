#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot
{
    SimTransmitter::SimTransmitter([[maybe_unused]] SimTransmitterConfiguration &config)
        : init_button_done_pressing_(false)
    {
        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_transmitter");
    }

    bool SimTransmitter::initialize()
    {
        return true;
    }

    CommandFeedback SimTransmitter::update()
    {
        return CommandFeedback{};
    }

    void SimTransmitter::send([[maybe_unused]] VelocityCommand command)
    {
    }

    bool SimTransmitter::did_init_button_press()
    {
        if (!init_button_done_pressing_)
        {
            init_button_done_pressing_ = true;
            return true;
        }
        else
        {
            return false;
        }
    }

} // namespace auto_battlebot
