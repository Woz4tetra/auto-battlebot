#pragma once

#include <chrono>
#include <memory>

#include "simulation/sim_connection.hpp"
#include "transmitter/config.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

class SimTransmitter : public TransmitterInterface {
   public:
    explicit SimTransmitter(const SimTransmitterConfiguration &config);

    bool initialize() override;
    CommandFeedback update() override;
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    bool is_connected() const override;

   private:
    double init_delay_seconds_;
    std::chrono::steady_clock::time_point start_time_;
    bool init_button_pressed_ = false;
    bool init_button_done_ = false;
    bool initialized_ = false;
    std::shared_ptr<SimConnection> connection_;
};

}  // namespace auto_battlebot
