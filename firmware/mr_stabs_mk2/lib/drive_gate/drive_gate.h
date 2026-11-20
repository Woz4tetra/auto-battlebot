#pragma once

#include <math.h>
#include <stdint.h>

namespace drive_gate {
enum class LoopMode : uint8_t { RADIO_LOST, STOPPED, DRIVE };

inline bool armed_now(bool radio_ok, bool armed) { return radio_ok && armed; }

inline LoopMode loop_mode(bool radio_ok, bool armed, bool tuning_owns_escs) {
    if (!radio_ok) return LoopMode::RADIO_LOST;
    if (!armed || tuning_owns_escs) return LoopMode::STOPPED;
    return LoopMode::DRIVE;
}

inline bool clear_guard_trip(LoopMode mode, bool armed) {
    return mode == LoopMode::STOPPED && !armed;
}

inline bool run_ota_handler(bool tuning_owns_escs) { return !tuning_owns_escs; }

inline bool espnow_arm_input(bool radio_ok, bool armed, bool tuning_owns_escs) {
    return armed_now(radio_ok, armed) || tuning_owns_escs;
}

inline bool erase_dump_allowed(bool radio_ok, bool armed) { return !armed_now(radio_ok, armed); }

inline bool substitution_active(bool espnow_enabled, bool substitution_enabled,
                                bool autonomy_switch_on, bool command_fresh) {
    return espnow_enabled && substitution_enabled && autonomy_switch_on && command_fresh;
}

enum class FlipSwitch : uint8_t { DOWN = 0, MIDDLE = 1, UP = 2 };

struct Orientation {
    bool upside_down;
    bool auto_steer;
};

inline Orientation resolve_orientation(FlipSwitch flip, bool sensed_upside_down) {
    switch (flip) {
        case FlipSwitch::UP:
            return {true, false};
        case FlipSwitch::MIDDLE:
            return {false, false};
        case FlipSwitch::DOWN:
            return {sensed_upside_down, true};
    }
    return {false, false};
}

inline float throttle_from_channel_percent(float channel_percent) { return -1 * channel_percent; }

inline float flipped_throttle(float a_percent, bool upside_down) {
    return upside_down ? -a_percent : a_percent;
}

inline float tail_first_throttle(float flipped_a_percent, bool upside_down) {
    return upside_down ? -flipped_a_percent : flipped_a_percent;
}

struct Pair {
    float left;
    float right;
};

inline Pair mix_pair(float flipped_a_percent, float steer_differential_pct) {
    Pair pair = {-1 * flipped_a_percent + steer_differential_pct,
                 -1 * flipped_a_percent - steer_differential_pct};
    const float max_command = fmaxf(fabsf(pair.left), fabsf(pair.right));
    if (max_command > 100.0f) {
        pair.left = pair.left / max_command * 100.0f;
        pair.right = pair.right / max_command * 100.0f;
    }
    return pair;
}

template <class Guard>
Pair guarded_pair(float flipped_a_percent, float steer_differential_pct, const Guard &guard) {
    Pair pair = mix_pair(flipped_a_percent, steer_differential_pct);
    guard.apply(pair.left, pair.right);
    return pair;
}
}  // namespace drive_gate
