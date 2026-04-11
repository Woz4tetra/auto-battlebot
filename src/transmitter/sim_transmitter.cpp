#include "transmitter/sim_transmitter.hpp"

#include <spdlog/spdlog.h>

namespace auto_battlebot {

SimTransmitter::SimTransmitter(const SimTransmitterConfiguration &config)
    : init_delay_seconds_(config.init_delay_seconds),
      command_delay_ms_(config.command_delay_ms),
      connection_(SimConnection::instance()) {
    if (command_delay_ms_ > 0.0) {
        spdlog::info("SimTransmitter: artificial command delay = {} ms", command_delay_ms_);
    }
}

bool SimTransmitter::initialize() {
    start_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
    return true;
}

CommandFeedback SimTransmitter::update() {
    if (!initialized_ || init_button_pressed_) return CommandFeedback{};

    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    if (elapsed >= init_delay_seconds_) {
        init_button_pressed_ = true;
    }
    return CommandFeedback{};
}

void SimTransmitter::enable() { enabled_ = true; }

void SimTransmitter::disable() { enabled_ = false; }

void SimTransmitter::send(VelocityCommand command) {
    if (!connection_ || !enabled_) return;

    if (command_delay_ms_ <= 0.0) {
        connection_->set_command(command);
        return;
    }

    auto now = std::chrono::steady_clock::now();
    command_queue_.push_back({now, command});

    while (!command_queue_.empty()) {
        double age_ms =
            std::chrono::duration<double, std::milli>(now - command_queue_.front().first).count();
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
