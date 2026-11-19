#include "transmitter/differential_drive_processor.hpp"

#include <algorithm>

namespace auto_battlebot {

DifferentialDriveProcessor::DifferentialDriveProcessor(
    const Config& config, std::shared_ptr<DiagnosticsModuleLogger> logger)
    : config_(config), logger_(std::move(logger)) {}

DifferentialDriveProcessor::Channels DifferentialDriveProcessor::process(
    VelocityCommand command) const {
    const auto saturated = saturate_velocity(command.forward_mps, command.yaw_rate_radps,
                                             config_.velocity_saturation_limit);

    const auto wheels_in = mix_to_wheels(saturated);
    const double zero_dz = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    const double lifted_dz = std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const WheelPair wheels_out{
        apply_lifted_deadzone(wheels_in.left, zero_dz, lifted_dz),
        apply_lifted_deadzone(wheels_in.right, zero_dz, lifted_dz),
    };

    auto out = inverse_mix_from_wheels(wheels_out);

    if (config_.reverse_linear) out.linear = -out.linear;
    if (config_.reverse_angular) out.angular = -out.angular;

    logger_->debug("process", {{"linear_x_in", command.forward_mps},
                               {"angular_z_in", command.yaw_rate_radps},
                               {"linear_saturated", saturated.linear},
                               {"angular_saturated", saturated.angular},
                               {"wheel_left_pre", wheels_in.left},
                               {"wheel_right_pre", wheels_in.right},
                               {"wheel_left_post", wheels_out.left},
                               {"wheel_right_post", wheels_out.right},
                               {"linear_out", out.linear},
                               {"angular_out", out.angular}});

    return {.first_output = out.linear, .second_output = out.angular};
}

BodyVelocity DifferentialDriveProcessor::to_body_velocity(Channels channels) const {
    return {.linear = config_.reverse_linear ? -channels.first_output : channels.first_output,
            .angular = config_.reverse_angular ? -channels.second_output : channels.second_output};
}

}  // namespace auto_battlebot
