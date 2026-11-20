#pragma once

#include <stdint.h>

namespace esc_tuning_plan {
enum class Action : uint8_t { NONE, EXIT, RELEASE_ESCS };

inline Action next_action(bool exit_requested, bool button_edge, bool released, uint32_t now_ms,
                          uint32_t last_activity_ms, uint32_t release_after_ms) {
    if (exit_requested || button_edge) return Action::EXIT;
    if (!released && now_ms - last_activity_ms >= release_after_ms) return Action::RELEASE_ESCS;
    return Action::NONE;
}
}  // namespace esc_tuning_plan
