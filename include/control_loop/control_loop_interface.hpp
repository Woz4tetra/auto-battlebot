#pragma once

#include <memory>
#include <utility>

#include "control_loop/control_loop.hpp"

namespace auto_battlebot {

class ControlLoopInterface {
   public:
    virtual ~ControlLoopInterface() = default;

    bool start() {
        const bool ok = loop_->initialize();
        start_driver();
        return ok;
    }

    virtual void stop() = 0;

    virtual void pump_input() {}

    virtual void advance_to([[maybe_unused]] double until) {}

    virtual bool is_healthy() const { return true; }

    void submit_measurement(ControlMeasurement measurement) {
        loop_->submit_measurement(std::move(measurement));
    }
    bool take_init_button_press() { return loop_->take_init_button_press(); }
    ControlOutput latest_output() const { return loop_->latest_output(); }
    void request_filter_reinit(int opponent_count) { loop_->request_filter_reinit(opponent_count); }
    void set_autonomy_enabled(bool enabled) { loop_->set_autonomy_enabled(enabled); }
    void set_behavior_mode(BehaviorMode mode) { loop_->set_behavior_mode(mode); }
    void set_target_override(std::optional<Pose2D> target) { loop_->set_target_override(target); }
    TransmitterStatus transmitter_status() const { return loop_->transmitter_status(); }
    NavigationVisualization last_visualization() const { return loop_->last_visualization(); }

   protected:
    explicit ControlLoopInterface(std::shared_ptr<ControlLoop> loop) : loop_(std::move(loop)) {}

    virtual void start_driver() = 0;

    std::shared_ptr<ControlLoop> loop_;
};

}  // namespace auto_battlebot
