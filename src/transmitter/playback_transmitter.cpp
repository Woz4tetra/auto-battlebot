#include "transmitter/playback_transmitter.hpp"

#include "enums/frame_id.hpp"

namespace auto_battlebot {

PlaybackTransmitter::PlaybackTransmitter(PlaybackTransmitterConfiguration &config)
    : init_delay_seconds_(config.init_delay_seconds),
      initialized_(false),
      start_time_(),
      init_button_pressed_(false),
      init_button_done_pressing_(false),
      logger_(DiagnosticsLogger::get_logger("playback_transmitter")),
      processor_(
          {
              .velocity_saturation_limit = config.velocity_saturation_limit,
              .zero_deadzone_percent = config.zero_deadzone_percent,
              .lifted_deadzone_percent = config.lifted_deadzone_percent,
              .reverse_linear = config.reverse_linear_channel,
              .reverse_angular = config.reverse_angular_channel,
          },
          logger_) {}

bool PlaybackTransmitter::initialize() {
    start_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
    return true;
}

CommandFeedback PlaybackTransmitter::update() {
    if (!initialized_ || init_button_pressed_) {
        return CommandFeedback{};
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time_);

    if (elapsed.count() >= init_delay_seconds_) {
        init_button_pressed_ = true;
    }

    if (!last_processed_) return CommandFeedback{};

    CommandFeedback feedback;
    feedback.commands[FrameId::OUR_ROBOT_1] = {
        .linear_x = last_processed_->linear,
        .linear_y = 0.0,
        .angular_z = last_processed_->angular,
    };
    return feedback;
}

void PlaybackTransmitter::send(VelocityCommand command) {
    last_processed_ = processor_.process(command);
}

bool PlaybackTransmitter::did_init_button_press() {
    if (!init_button_done_pressing_ && init_button_pressed_) {
        init_button_done_pressing_ = true;
        return true;
    } else {
        return false;
    }
}

}  // namespace auto_battlebot
