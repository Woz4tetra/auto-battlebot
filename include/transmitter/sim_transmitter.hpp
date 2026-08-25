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
    void send(VelocityCommand command) override;
    bool did_init_button_press() override;
    bool is_connected() const override;
    BehaviorMode behavior_mode() const override;
    void enable() override;
    void disable() override;

   private:
    double init_delay_seconds_;
    double command_delay_ms_;
    double velocity_saturation_limit_;
    BehaviorMode behavior_mode_;
    double start_time_ = 0.0;  // logical seconds
    bool init_button_pressed_ = false;
    bool init_button_done_ = false;
    bool initialized_ = false;
    bool enabled_ = false;
    std::shared_ptr<ClockInterface> clock_;
    std::shared_ptr<SimConnection> connection_;
    // Queued commands with their logical enqueue time, for the artificial actuation delay.
    std::deque<std::pair<double, VelocityCommand>> command_queue_;
    /** Last command issued through send() (post-saturation, normalized stick space), reported
     *  back as CommandFeedback. Cleared on disable(): with autonomy cut nothing reaches the
     *  simulated robot, so there is no command to report. */
    std::optional<VelocityCommand> last_sent_;
};

}  // namespace auto_battlebot
