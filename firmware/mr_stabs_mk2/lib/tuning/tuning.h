#pragma once
#include <cstddef>

#include "yaw_control.h"

namespace tuning {
struct Settings {
    float left_deadzone = 1.0f;   // % of command below which the left ESC gets DShot stop
    float right_deadzone = 1.0f;  // same, right ESC
    yaw_control::Config yaw;
};

struct Field {
    const char *key;
    const char *unit;
    float min;
    float max;
    float *(*ref)(Settings &);
};

extern const Field FIELDS[];
extern const size_t FIELD_COUNT;

const Field *find(const char *key);

inline float get(const Settings &settings, const Field &field) {
    return *field.ref(const_cast<Settings &>(settings));
}

bool set(Settings &settings, const char *key, float value);

struct FormField {
    const char *name;
    const char *value;
};

const char *first_invalid_field(Settings settings, const FormField *fields, size_t count);
}  // namespace tuning
