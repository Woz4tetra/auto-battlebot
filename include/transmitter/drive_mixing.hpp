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

/**
 * Clamp a body-frame command to the configured velocity budget. Angular gets priority and linear
 * fills the remaining headroom; saturation_limit == 0 falls back to clamping each axis to [-1, 1].
 */
BodyVelocity saturate_velocity(double linear, double angular, double saturation_limit);

/**
 * Differential-drive mixing. With saturation enabled (limit <= 1) the wheel magnitudes are
 * guaranteed to be in [-1, 1]; with saturation disabled they can briefly exceed 1 but the
 * per-wheel deadzone clamps them.
 */
WheelPair mix_to_wheels(BodyVelocity body);

/** Recover the body-frame command that produced the given wheel pair. */
BodyVelocity inverse_mix_from_wheels(WheelPair wheels);

/**
 * Map |value| from (zero_deadzone, 1] -> [lifted_deadzone, 1] preserving sign. Below
 * zero_deadzone the output is forced to 0. Above zero_deadzone the smallest non-zero output is
 * lifted_deadzone, which is the minimum throttle that overcomes static friction on a wheel.
 */
double apply_lifted_deadzone(double value, double zero_deadzone, double lifted_deadzone);

}  // namespace auto_battlebot
