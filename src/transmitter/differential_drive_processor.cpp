#include "transmitter/differential_drive_processor.hpp"

#include <algorithm>

namespace auto_battlebot {

DifferentialDriveProcessor::DifferentialDriveProcessor(
    const Config& config, std::shared_ptr<DiagnosticsModuleLogger> logger)
    : config_(config), logger_(std::move(logger)) {}

DifferentialDriveProcessor::Channels DifferentialDriveProcessor::process(
    VelocityCommand command) const {
    const auto saturated =
        saturate_velocity(command.linear_x, command.angular_z, config_.velocity_saturation_limit);

    // The static-friction barrier exists per wheel, not per body axis, so we apply the lifted
    // deadzone in wheel space. Side-effect: when the robot is already driving forward, a small
    // steering input passes through cleanly (both wheels are well past the friction barrier); same
    // for a small linear nudge while spinning in place.
    const auto wheels_in = mix_to_wheels(saturated);
    const double zero_dz = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    const double lifted_dz = std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const WheelPair wheels_out{
        apply_lifted_deadzone(wheels_in.left, zero_dz, lifted_dz),
        apply_lifted_deadzone(wheels_in.right, zero_dz, lifted_dz),
    };

    auto out = inverse_mix_from_wheels(wheels_out);

    // Compensate for the physical motor wiring on the robot. Applied last so the wheel-space math
    // operates in a consistent body frame.
    if (config_.reverse_linear) out.linear = -out.linear;
    if (config_.reverse_angular) out.angular = -out.angular;

    logger_->debug("process", {{"linear_x_in", command.linear_x},
                               {"angular_z_in", command.angular_z},
                               {"linear_saturated", saturated.linear},
                               {"angular_saturated", saturated.angular},
                               {"wheel_left_pre", wheels_in.left},
                               {"wheel_right_pre", wheels_in.right},
                               {"wheel_left_post", wheels_out.left},
                               {"wheel_right_post", wheels_out.right},
                               {"linear_out", out.linear},
                               {"angular_out", out.angular}});

    return {.channel_a = out.linear, .channel_b = out.angular};
}

BodyVelocity DifferentialDriveProcessor::to_body_velocity(Channels channels) const {
    return {.linear = channels.channel_a, .angular = channels.channel_b};
}

}  // namespace auto_battlebot
