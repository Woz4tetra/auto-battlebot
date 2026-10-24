#include "tuning.h"

#include <cmath>
#include <cstring>

namespace tuning {
// Ranges only catch typos and torn writes; the defaults in yaw_control.h say what is sensible.
const Field FIELDS[] = {
    {"left_deadzone", "%", 0.0f, 50.0f, [](Settings &s) { return &s.left_deadzone; }},
    {"right_deadzone", "%", 0.0f, 50.0f, [](Settings &s) { return &s.right_deadzone; }},
    {"max_rate", "deg/s", 0.0f, 2000.0f, [](Settings &s) { return &s.yaw.max_rate; }},
    {"feedforward", "%/(deg/s)", 0.0f, 1.0f, [](Settings &s) { return &s.yaw.feedforward; }},
    {"kp", "%/(deg/s)", 0.0f, 10.0f, [](Settings &s) { return &s.yaw.kp; }},
    {"ki", "%/deg", 0.0f, 10.0f, [](Settings &s) { return &s.yaw.ki; }},
    {"i_max", "%", 0.0f, 100.0f, [](Settings &s) { return &s.yaw.i_max; }},
    {"k_heading", "(deg/s)/deg", 0.0f, 100.0f, [](Settings &s) { return &s.yaw.k_heading; }},
    {"hold_rate_max", "deg/s", 0.0f, 2000.0f, [](Settings &s) { return &s.yaw.hold_rate_max; }},
    {"rev_kp_scale", "x", 0.0f, 20.0f, [](Settings &s) { return &s.yaw.reverse_kp_scale; }},
    {"rev_ff_scale", "x", 0.0f, 20.0f, [](Settings &s) { return &s.yaw.reverse_ff_scale; }},
    {"turn_threshold", "%", 0.0f, 50.0f, [](Settings &s) { return &s.yaw.turn_threshold; }},
    {"capture_rate", "deg/s", 0.0f, 2000.0f, [](Settings &s) { return &s.yaw.capture_rate; }},
    {"capture_timeout", "s", 0.0f, 5.0f, [](Settings &s) { return &s.yaw.capture_timeout; }},
    {"idle_throttle", "%", 0.0f, 100.0f, [](Settings &s) { return &s.yaw.idle_throttle; }},
};
const size_t FIELD_COUNT = sizeof(FIELDS) / sizeof(FIELDS[0]);

const Field *find(const char *key) {
    for (size_t i = 0; i < FIELD_COUNT; i++) {
        if (strcmp(FIELDS[i].key, key) == 0) return &FIELDS[i];
    }
    return nullptr;
}

bool set(Settings &settings, const char *key, float value) {
    const Field *field = find(key);
    if (!field || !std::isfinite(value) || value < field->min || value > field->max) return false;
    *field->ref(settings) = value;
    return true;
}
}  // namespace tuning
