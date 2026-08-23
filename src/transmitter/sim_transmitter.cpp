#include "transmitter/sim_transmitter.hpp"

#include <spdlog/spdlog.h>

#include "transmitter/drive_mixing.hpp"

namespace auto_battlebot {

SimTransmitter::SimTransmitter(const SimTransmitterConfiguration &config,
                               std::shared_ptr<ClockInterface> clock)
    : init_delay_seconds_(config.init_delay_seconds),
      command_delay_ms_(config.command_delay_ms),
      velocity_saturation_limit_(config.velocity_saturation_limit),
      clock_(std::move(clock)),
      connection_(SimConnection::instance()) {
    if (command_delay_ms_ > 0.0) {
        spdlog::info("SimTransmitter: artificial command delay = {} ms", command_delay_ms_);
    }
}

bool SimTransmitter::initialize() {
    start_time_ = clock_->now();
    initialized_ = true;
    return true;
}

CommandFeedback SimTransmitter::update() {
    if (!initialized_ || init_button_pressed_) return CommandFeedback{};

    const double elapsed = clock_->now() - start_time_;
    if (elapsed >= init_delay_seconds_) {
        init_button_pressed_ = true;
    }
    return CommandFeedback{};
}

void SimTransmitter::enable() { enabled_ = true; }

void SimTransmitter::disable() { enabled_ = false; }

void SimTransmitter::send(VelocityCommand command) {
    if (!connection_ || !enabled_) return;

    // The real transmitter's drive processor saturates before mixing to wheels, giving angular
    // priority and leaving linear whatever authority is left. There is no processor in the sim
    // path, so apply the same clip here or the simulated robot drives through turns that the real
    // one cannot.
    const auto saturated =
        saturate_velocity(command.linear_x, command.angular_z, velocity_saturation_limit_);
    command.linear_x = saturated.linear;
    command.angular_z = saturated.angular;

    if (command_delay_ms_ <= 0.0) {
        connection_->set_command(command);
        return;
    }

    const double now = clock_->now();
    command_queue_.push_back({now, command});

    while (!command_queue_.empty()) {
        const double age_ms = (now - command_queue_.front().first) * 1000.0;
        if (age_ms >= command_delay_ms_) {
            connection_->set_command(command_queue_.front().second);
            command_queue_.pop_front();
        } else {
            break;
        }
    }
}

bool SimTransmitter::did_init_button_press() {
    if (!init_button_done_ && init_button_pressed_) {
        init_button_done_ = true;
        return true;
    }
    return false;
}

bool SimTransmitter::is_connected() const { return connection_ && connection_->is_connected(); }

}  // namespace auto_battlebot
