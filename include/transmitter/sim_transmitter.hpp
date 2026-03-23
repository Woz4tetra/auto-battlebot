#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <utility>

#include "data_structures/velocity.hpp"
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
    void enable() override;
    void disable() override;

   private:
    using TimePoint = std::chrono::steady_clock::time_point;

    double init_delay_seconds_;
    double command_delay_ms_;
    std::chrono::steady_clock::time_point start_time_;
    bool init_button_pressed_ = false;
    bool init_button_done_ = false;
    bool initialized_ = false;
    bool enabled_ = false;
    std::shared_ptr<SimConnection> connection_;
    std::deque<std::pair<TimePoint, VelocityCommand>> command_queue_;
};

}  // namespace auto_battlebot
