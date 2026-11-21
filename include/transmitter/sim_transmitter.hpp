#pragma once

#include <deque>
#include <memory>
#include <optional>
#include <utility>

#include "data_structures/velocity.hpp"
#include "simulation/sim_connection.hpp"
#include "time/clock_interface.hpp"
#include "transmitter/config.hpp"
#include "transmitter/transmitter_interface.hpp"

namespace auto_battlebot {

class SimTransmitter : public TransmitterInterface {
   public:
    SimTransmitter(const SimTransmitterConfiguration &config,
                   std::shared_ptr<ClockInterface> clock);

    bool initialize() override;
    CommandFeedback update() override;
    void send(StickCommand command) override;
    bool did_init_button_press() override;
    TransmitterStatus get_status() const override;
    void enable() override;
    void disable() override;

   private:
    double init_delay_seconds_;
    double command_delay_ms_;
    double start_time_s_ = 0.0;
    bool init_button_pressed_ = false;
    bool init_button_done_ = false;
    bool initialized_ = false;
    bool enabled_ = false;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<SimConnection> connection_;
    std::deque<std::pair<double, StickCommand>> command_queue_;
    std::optional<StickCommand> last_sent_command_;
};

}  // namespace auto_battlebot
