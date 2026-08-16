#include "transmitter/drive_mixing.hpp"

#include <algorithm>
#include <cmath>

namespace auto_battlebot {

BodyVelocity saturate_velocity(double linear, double angular, double saturation_limit) {
    const double angular_clamped = std::clamp(angular, -1.0, 1.0);
    if (saturation_limit > 0.0) {
        const double limit = std::clamp(saturation_limit, 0.0, 1.0);
        const double headroom = std::max(0.0, limit - std::abs(angular_clamped));
        return {std::clamp(linear, -headroom, headroom), angular_clamped};
    }
    return {std::clamp(linear, -1.0, 1.0), angular_clamped};
}

WheelPair mix_to_wheels(BodyVelocity body) {
    return {body.linear + body.angular, body.linear - body.angular};
}

BodyVelocity inverse_mix_from_wheels(WheelPair wheels) {
    return {0.5 * (wheels.left + wheels.right), 0.5 * (wheels.left - wheels.right)};
}

double apply_lifted_deadzone(double value, double zero_deadzone, double lifted_deadzone) {
    const double magnitude = std::abs(value);
    if (magnitude <= zero_deadzone) return 0.0;
    const double denom = std::max(1e-6, 1.0 - zero_deadzone);
    const double shifted = std::clamp((magnitude - zero_deadzone) / denom, 0.0, 1.0);
    const double lifted = lifted_deadzone + (1.0 - lifted_deadzone) * shifted;
    return std::copysign(std::clamp(lifted, 0.0, 1.0), value);
}

}  // namespace auto_battlebot
