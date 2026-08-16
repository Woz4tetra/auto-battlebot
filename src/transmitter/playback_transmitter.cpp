#include "transmitter/playback_transmitter.hpp"

#include "enums/frame_id.hpp"

namespace auto_battlebot {

PlaybackTransmitter::PlaybackTransmitter(PlaybackTransmitterConfiguration &config,
                                         std::shared_ptr<ClockInterface> clock)
    : init_delay_seconds_(config.init_delay_seconds),
      initialized_(false),
      clock_(std::move(clock)),
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
    start_time_seconds_.reset();
    initialized_ = true;
    return true;
}

CommandFeedback PlaybackTransmitter::update() {
    if (!initialized_) {
        return CommandFeedback{};
    }

    // One-shot init button: fires once after the delay, mirroring the physical init button on the
    // radio. It must NOT gate command feedback. OpenTxTransmitter streams channel feedback every
    // frame regardless of the init button, and the robot filter's motion prediction depends on that
    // continuous stream. Gating feedback on init_button_pressed_ left the command-driven prediction
    // path inert (and therefore untested) in playback.
    if (!init_button_pressed_) {
        // Logical time, not wall clock: the delay has to land on the same frame every run or
        // replay is not reproducible.
        const double now = clock_ ? clock_->now() : 0.0;
        if (!start_time_seconds_) start_time_seconds_ = now;
        if (now - *start_time_seconds_ >= init_delay_seconds_) {
            init_button_pressed_ = true;
        }
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
