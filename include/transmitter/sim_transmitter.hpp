#pragma once

#include <memory>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "transmitter/transmitter_interface.hpp"
#include "transmitter/config.hpp"
#include "communication/sim_tcp_client.hpp"

namespace auto_battlebot
{
    /**
     * @brief Simulation transmitter that sends velocity commands via TCP to Unity
     *
     * This class implements TransmitterInterface for simulation mode.
     * It uses the SimTcpClient singleton (shared with SimRgbdCamera) to send
     * velocity commands to Unity.
     */
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

        uint64_t num_commands_sent_ = 0;
        bool init_button_done_pressing_ = false;
    };

} // namespace auto_battlebot
