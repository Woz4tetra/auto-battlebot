#pragma once
#include <cstddef>

#include "yaw_control.h"

namespace tuning {
// Every setting the diagnostics page and ESP-NOW can change at runtime. The robot keeps them in
// NVS, one float per field under the field's key, so adding a field keeps the others' values.
struct Settings {
    float left_deadzone = 1.0f;   // % of command below which the left ESC gets DShot stop
    float right_deadzone = 1.0f;  // same, right ESC
    yaw_control::Config yaw;
};

struct Field {
    const char *key;  // JSON name and NVS key, so at most 15 characters
    const char *unit;
    float min;
    float max;
    float *(*ref)(Settings &);
};

extern const Field FIELDS[];
extern const size_t FIELD_COUNT;

const Field *find(const char *key);

// Validated value of `field` in `settings`.
inline float get(const Settings &settings, const Field &field) {
    return *field.ref(const_cast<Settings &>(settings));
}

// Sets `key` when it names a field and `value` is finite and inside its range.
bool set(Settings &settings, const char *key, float value);
}  // namespace tuning
