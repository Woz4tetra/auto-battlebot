#include "transmitter/sim_transmitter.hpp"
#include "communication/simulation_sync_manager.hpp"

namespace auto_battlebot
{
    SimTransmitter::SimTransmitter(SimTransmitterConfiguration &config)
        : enable_double_buffering_(config.enable_double_buffering),
          enable_sync_socket_(config.enable_sync_socket),
          init_button_done_pressing_(false)
    {
        buffer_size_ = SimulationVelocityCommand::SIZE;
        num_commands_sent = 0;

        diagnostics_logger_ = DiagnosticsLogger::get_logger("sim_transmitter");
    }

    bool SimTransmitter::initialize()
    {
        size_t total_size;
        if (enable_double_buffering_)
            total_size = buffer_size_ * 2; // Double buffering
        else
            total_size = buffer_size_;
        command_writer_ = std::make_unique<SharedMemoryWriter>("auto_battlebot_command", total_size);

        return command_writer_->open();
    }

    CommandFeedback SimTransmitter::update()
    {
        return CommandFeedback{};
    }

    void SimTransmitter::send(VelocityCommand command)
    {
        size_t buffer_offset = 0;
        if (enable_double_buffering_)
        {
            buffer_offset = num_commands_sent % 2 == 0 ? 0 : buffer_size_;
        }
        command_writer_->write_at(buffer_offset, SimulationVelocityCommand{
                                                     num_commands_sent,
                                                     command.linear_x,
                                                     command.linear_y,
                                                     command.angular_z,
                                                 });
        num_commands_sent++;

        // Signal to Unity that command is ready (if sync socket is enabled and connected)
        if (enable_sync_socket_ && SimulationSyncManager::instance().is_connected())
        {
            SimulationSyncManager::instance().signal_command_ready();
        }
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
