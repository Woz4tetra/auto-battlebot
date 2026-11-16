#pragma once
#include <cmath>
#include <cstdint>

namespace esc_map {
const uint16_t DSHOT_STOP = 0;
const uint16_t DSHOT_3D_REV_MIN = 48;
const uint16_t DSHOT_3D_FWD_MIN = 1048;
const uint16_t DSHOT_3D_STEPS = 999;
const uint16_t FLOOR_STEP = 66;

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

inline float duty(float signed_percent, float stop_threshold) {
    float d = steps(fabsf(signed_percent), stop_threshold) / (float)DSHOT_3D_STEPS;
    return signed_percent < 0.0f ? -d : d;
}
}  // namespace esc_map
