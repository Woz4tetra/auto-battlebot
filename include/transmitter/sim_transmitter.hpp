#pragma once

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "transmitter/config.hpp"
#include "shared_memory/simulation/simulation_velocity_command.hpp"
#include "shared_memory/shared_memory_writer.hpp"

namespace auto_battlebot
{
    class SimTransmitter : public TransmitterInterface
    {
    public:
        SimTransmitter(SimTransmitterConfiguration &config);

        bool initialize() override;
        CommandFeedback update() override;
        void send(VelocityCommand command) override;
        bool did_init_button_press() override;

    private:
        std::shared_ptr<DiagnosticsModuleLogger> diagnostics_logger_;

        bool enable_double_buffering_;
        bool enable_sync_socket_;

        size_t buffer_size_;
        uint64_t num_commands_sent;
        bool init_button_done_pressing_;

        std::unique_ptr<SharedMemoryWriter> command_writer_;
    };

} // namespace auto_battlebot
