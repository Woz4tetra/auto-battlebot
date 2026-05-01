#include "transmitter/differential_drive_processor.hpp"

#include <algorithm>
#include <cmath>

namespace auto_battlebot {
namespace {

struct BodyVelocity {
    double linear;
    double angular;
};
struct WheelPair {
    double left;
    double right;
};

// Clamp a body-frame command to the configured velocity budget. Angular gets priority and linear
// fills the remaining headroom; saturation_limit == 0 falls back to clamping each axis to [-1, 1].
BodyVelocity saturate_velocity(double linear, double angular, double saturation_limit) {
    const double angular_clamped = std::clamp(angular, -1.0, 1.0);
    if (saturation_limit > 0.0) {
        const double limit = std::clamp(saturation_limit, 0.0, 1.0);
        const double headroom = std::max(0.0, limit - std::abs(angular_clamped));
        return {std::clamp(linear, -headroom, headroom), angular_clamped};
    }
    return {std::clamp(linear, -1.0, 1.0), angular_clamped};
}

// Differential-drive mixing. With saturation enabled (limit <= 1) the wheel magnitudes are
// guaranteed to be in [-1, 1]; with saturation disabled they may briefly exceed 1 but the
// per-wheel deadzone clamps them.
WheelPair mix_to_wheels(BodyVelocity body) {
    return {body.linear + body.angular, body.linear - body.angular};
}

BodyVelocity inverse_mix_from_wheels(WheelPair wheels) {
    return {0.5 * (wheels.left + wheels.right), 0.5 * (wheels.left - wheels.right)};
}

// Map |value| from (zero_deadzone, 1] -> [lifted_deadzone, 1] preserving sign. Below
// zero_deadzone the output is forced to 0. Above zero_deadzone the smallest non-zero output is
// lifted_deadzone, which is the minimum throttle that overcomes static friction on a wheel.
double apply_lifted_deadzone(double value, double zero_deadzone, double lifted_deadzone) {
    const double magnitude = std::abs(value);
    if (magnitude <= zero_deadzone) return 0.0;
    const double denom = std::max(1e-6, 1.0 - zero_deadzone);
    const double shifted = std::clamp((magnitude - zero_deadzone) / denom, 0.0, 1.0);
    const double lifted = lifted_deadzone + (1.0 - lifted_deadzone) * shifted;
    return std::copysign(std::clamp(lifted, 0.0, 1.0), value);
}

}  // namespace

DifferentialDriveProcessor::DifferentialDriveProcessor(
    const Config& config, std::shared_ptr<DiagnosticsModuleLogger> logger)
    : config_(config), logger_(std::move(logger)) {}

DifferentialDriveProcessor::Result DifferentialDriveProcessor::process(
    VelocityCommand command) const {
    const auto saturated =
        saturate_velocity(command.linear_x, command.angular_z, config_.velocity_saturation_limit);

    // The static-friction barrier exists per wheel, not per body axis, so we apply the lifted
    // deadzone in wheel space. Side-effect: when the robot is already driving forward, a small
    // steering input passes through cleanly (both wheels well past the friction barrier); same
    // for a small linear nudge while spinning in place.
    const auto wheels_in = mix_to_wheels(saturated);
    const double zero_dz = std::clamp(config_.zero_deadzone_percent, 0.0, 100.0) / 100.0;
    const double lifted_dz = std::clamp(config_.lifted_deadzone_percent, 0.0, 100.0) / 100.0;
    const WheelPair wheels_out{
        apply_lifted_deadzone(wheels_in.left, zero_dz, lifted_dz),
        apply_lifted_deadzone(wheels_in.right, zero_dz, lifted_dz),
    };

    auto out = inverse_mix_from_wheels(wheels_out);

    // Compensate for physical motor wiring on the robot. Applied last so the wheel-space math
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

    return {.linear = out.linear, .angular = out.angular};
}

}  // namespace auto_battlebot
