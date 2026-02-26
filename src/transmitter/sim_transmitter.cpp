#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot
{

SimTransmitter::SimTransmitter([[maybe_unused]] SimTransmitterConfiguration& config)
{
    diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_transmitter");
}

bool SimTransmitter::initialize()
{
    // Use the SimTcpClient singleton (should be connected by SimRgbdCamera)
    auto& client = SimTcpClient::instance();
    
    if (!client.is_connected())
    {
        diagnostics_logger_->warning("tcp_client_not_connected",
            {{"message", "SimRgbdCamera should be initialized first to establish connection"}});
        // Not a fatal error - connection may be established later
    }
    else
    {
        diagnostics_logger_->info("initialized",
            {{"tcp_connected", true}});
    }

    return true;
}

CommandFeedback SimTransmitter::update()
{
    // No feedback from simulation
    return CommandFeedback{};
}

void SimTransmitter::send(VelocityCommand command)
{
    auto& client = SimTcpClient::instance();

    if (!client.is_connected())
    {
        diagnostics_logger_->warning("send_failed",
            {{"reason", "TCP not connected"}});
        return;
    }

    if (client.send_velocity_command(command))
    {
        num_commands_sent_++;

        // Log periodically
        if (num_commands_sent_ % 100 == 0)
        {
            diagnostics_logger_->debug("commands_sent",
                {{"total", static_cast<int>(num_commands_sent_)},
                 {"linear_x", command.linear_x},
                 {"linear_y", command.linear_y},
                 {"angular_z", command.angular_z}});
        }
    }
    else
    {
        diagnostics_logger_->warning("send_velocity_failed",
            {{"linear_x", command.linear_x},
             {"linear_y", command.linear_y},
             {"angular_z", command.angular_z}});
    }
}

bool SimTransmitter::did_init_button_press()
{
    // Simulate init button press on first call
    if (!init_button_done_pressing_)
    {
        init_button_done_pressing_ = true;
        return true;
    }
    return false;
}

bool SimTransmitter::is_connected() const
{
    return SimTcpClient::instance().is_connected();
}

} // namespace auto_battlebot
