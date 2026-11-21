#include "transmitter/tank_drive_processor.hpp"

#include <algorithm>

namespace auto_battlebot {

TankDriveProcessor::TankDriveProcessor(const Config& config,
                                       std::shared_ptr<DiagnosticsModuleLogger> logger)
    : config_(config), logger_(std::move(logger)) {}

TankDriveProcessor::Channels TankDriveProcessor::process(StickCommand command) const {
    BodyVelocity body{.linear = command.forward, .angular = command.yaw};
    if (config_.reverse_linear) body.linear = -body.linear;
    if (config_.reverse_angular) body.angular = -body.angular;

    const auto wheels_in = mix_to_wheels(body);
    const double zero_dz = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    const double lifted_dz = std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const WheelPair wheels_out{
        apply_lifted_deadzone(wheels_in.left, zero_dz, lifted_dz),
        apply_lifted_deadzone(wheels_in.right, zero_dz, lifted_dz),
    };

    logger_->debug("process", {{"linear_x_in", command.forward},
                               {"angular_z_in", command.yaw},
                               {"wheel_left_pre", wheels_in.left},
                               {"wheel_right_pre", wheels_in.right},
                               {"wheel_left_post", wheels_out.left},
                               {"wheel_right_post", wheels_out.right}});

    return {.first_output = wheels_out.left, .second_output = wheels_out.right};
}

BodyVelocity TankDriveProcessor::to_body_velocity(Channels channels) const {
    auto body =
        inverse_mix_from_wheels({.left = channels.first_output, .right = channels.second_output});
    if (config_.reverse_linear) body.linear = -body.linear;
    if (config_.reverse_angular) body.angular = -body.angular;
    return body;
}

}  // namespace auto_battlebot
