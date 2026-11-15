#pragma once

namespace auto_battlebot {

/** Normalized body-frame command. Both axes are in [-1, 1]. */
struct BodyVelocity {
    double linear;
    double angular;
};

/** Normalized per-wheel commands. Both wheels are in [-1, 1]. */
struct WheelPair {
    double left;
    double right;
};

BodyVelocity saturate_velocity(double linear, double angular, double saturation_limit);

WheelPair mix_to_wheels(BodyVelocity body);

BodyVelocity inverse_mix_from_wheels(WheelPair wheels);

double apply_lifted_deadzone(double value, double zero_deadzone, double lifted_deadzone);

}  // namespace auto_battlebot
