#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot {

SimTransmitter::SimTransmitter(const SimTransmitterConfiguration &config)
    : init_delay_seconds_(config.init_delay_seconds),
      connection_(SimConnection::instance()) {}

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

void SimTransmitter::send(VelocityCommand command) {
    if (connection_) connection_->set_command(command);
}

bool SimTransmitter::did_init_button_press() {
    if (!init_button_done_ && init_button_pressed_) {
        init_button_done_ = true;
        return true;
    }
    return false;
}

bool SimTransmitter::is_connected() const {
    return connection_ && connection_->is_connected();
}

}  // namespace auto_battlebot
