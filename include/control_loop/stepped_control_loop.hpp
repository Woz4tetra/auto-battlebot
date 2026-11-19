#pragma once

#include <memory>
#include <optional>

#include "control_loop/config.hpp"
#include "control_loop/control_loop_interface.hpp"

namespace auto_battlebot {

class SteppedControlLoop : public ControlLoopInterface {
   public:
    SteppedControlLoop(
        std::shared_ptr<ControlLoop> loop, double rate_hz,
        int max_catchup_cycles = SteppedControlLoopConfiguration{}.max_catchup_cycles);

    void stop() override {}

    void pump_input() override { loop_->pump_input(); }

    void advance_to(double until) override;

   protected:
    void start_driver() override {}

   private:
    std::optional<double> period_s_;
    int max_catchup_cycles_;
    double last_advance_ = 0.0;
    bool started_ = false;
};

}  // namespace auto_battlebot
