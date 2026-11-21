#pragma once

namespace auto_battlebot {

struct BodyVelocity {
    double linear;
    double angular;
};

struct WheelPair {
    double left;
    double right;
};

WheelPair mix_to_wheels(BodyVelocity body);

BodyVelocity inverse_mix_from_wheels(WheelPair wheels);

double apply_lifted_deadzone(double value, double zero_deadzone, double lifted_deadzone);

}  // namespace auto_battlebot
