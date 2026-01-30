#include "transmitter/sim_transmitter.hpp"
#include "rgbd_camera/sim_rgbd_camera.hpp"

namespace auto_battlebot
{

SimTransmitter::SimTransmitter([[maybe_unused]] SimTransmitterConfiguration& config)
{
    diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_transmitter");
}

bool SimTransmitter::initialize()
{
    // Get the shared TCP client from SimRgbdCamera
    tcp_client_ = SimRgbdCamera::get_tcp_client();

    auto client = tcp_client_.lock();
    if (!client)
    {
        diagnostics_logger_->warning("tcp_client_not_available",
            {{"message", "SimRgbdCamera must be initialized first"}});
        // Not an error - the TCP client may not be ready yet
        // We'll try again when sending commands
    }
    else
    {
        diagnostics_logger_->info("initialized",
            {{"tcp_connected", client->is_connected()}});
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
    // Get TCP client (may have been initialized after us)
    auto client = tcp_client_.lock();
    if (!client)
    {
        tcp_client_ = SimRgbdCamera::get_tcp_client();
        client = tcp_client_.lock();
    }

    if (!client)
    {
        diagnostics_logger_->warning("send_failed",
            {{"reason", "TCP client not available"}});
        return;
    }

    if (!client->is_connected())
    {
        diagnostics_logger_->warning("send_failed",
            {{"reason", "TCP not connected"}});
        return;
    }

    if (client->send_velocity_command(command))
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

} // namespace auto_battlebot
