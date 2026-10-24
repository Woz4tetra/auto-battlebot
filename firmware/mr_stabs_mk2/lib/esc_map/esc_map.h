#pragma once
#include <cmath>
#include <cstdint>

namespace esc_map {
const uint16_t DSHOT_STOP = 0;
// DShot 3D mode: 48-1047 reverse, 1048-2047 forward, higher value = faster in both directions.
const uint16_t DSHOT_3D_REV_MIN = 48;
const uint16_t DSHOT_3D_FWD_MIN = 1048;
const uint16_t DSHOT_3D_STEPS = 999;  // steps above zero in each direction
// The ESCs run every step from 1 to 65 at the same speed, the speed of step 65, and respond
// linearly from 66 up: both ESCs, both directions, no hysteresis, unchanged by sine mode,
// rampup power or PWM frequency (stand sweeps, 2026-10-24). Commands start at 66 so the
// smallest one past stop_threshold already moves the motor.
const uint16_t FLOOR_STEP = 66;

// Steps above zero for a command magnitude in percent: 0 inside stop_threshold, else
// FLOOR_STEP at the threshold up to DSHOT_3D_STEPS at 100%.
inline uint16_t steps(float magnitude, float stop_threshold) {
    magnitude = fminf(magnitude, 100.0f);
    if (magnitude < stop_threshold || magnitude <= 0.0f) return 0;
    return FLOOR_STEP + (uint16_t)(magnitude / 100.0f * (DSHOT_3D_STEPS - FLOOR_STEP));
}

inline uint16_t percent_to_dshot_3d(float signed_percent, float stop_threshold) {
    uint16_t n = steps(fabsf(signed_percent), stop_threshold);
    if (n == 0) return DSHOT_STOP;
    return (signed_percent > 0.0f ? DSHOT_3D_FWD_MIN : DSHOT_3D_REV_MIN) + n;
}

// Signed duty the ESC applies for a command, as a fraction of full: the DShot step over 999.
inline float duty(float signed_percent, float stop_threshold) {
    float d = steps(fabsf(signed_percent), stop_threshold) / (float)DSHOT_3D_STEPS;
    return signed_percent < 0.0f ? -d : d;
}
}  // namespace esc_map
