#include "transmitter/tank_drive_processor.hpp"

#include <algorithm>

namespace auto_battlebot {

TankDriveProcessor::TankDriveProcessor(const Config& config,
                                       std::shared_ptr<DiagnosticsModuleLogger> logger)
    : config_(config), logger_(std::move(logger)) {}

TankDriveProcessor::Channels TankDriveProcessor::process(VelocityCommand command) const {
    auto saturated =
        saturate_velocity(command.linear_x, command.angular_z, config_.velocity_saturation_limit);

    // Compensate for the physical motor wiring on the robot. Applied on the body axes before
    // mixing; the lifted deadzone is odd-symmetric, so the sign flips commute with it and the
    // result matches DifferentialDriveProcessor reversing after the deadzone.
    if (config_.reverse_linear) saturated.linear = -saturated.linear;
    if (config_.reverse_angular) saturated.angular = -saturated.angular;

    // The static-friction barrier exists per wheel, not per body axis, so the lifted deadzone is
    // applied in wheel space, which is also the space these channels are sent in.
    const auto wheels_in = mix_to_wheels(saturated);
    const double zero_dz = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    const double lifted_dz = std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const WheelPair wheels_out{
        apply_lifted_deadzone(wheels_in.left, zero_dz, lifted_dz),
        apply_lifted_deadzone(wheels_in.right, zero_dz, lifted_dz),
    };

    logger_->debug("process", {{"linear_x_in", command.linear_x},
                               {"angular_z_in", command.angular_z},
                               {"linear_saturated", saturated.linear},
                               {"angular_saturated", saturated.angular},
                               {"wheel_left_pre", wheels_in.left},
                               {"wheel_right_pre", wheels_in.right},
                               {"wheel_left_post", wheels_out.left},
                               {"wheel_right_post", wheels_out.right}});

    return {.channel_a = wheels_out.left, .channel_b = wheels_out.right};
}

BodyVelocity TankDriveProcessor::to_body_velocity(Channels channels) const {
    return inverse_mix_from_wheels({.left = channels.channel_a, .right = channels.channel_b});
}

}  // namespace auto_battlebot
