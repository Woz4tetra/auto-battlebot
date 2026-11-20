#pragma once

#include <stdint.h>

namespace armed_guard {
enum class Write : uint8_t { WIFI_SETTINGS, DONGLES, CRASH_DUMP, ESC_TUNING };

inline const char *refusal(Write write, bool armed) {
    if (!armed) return nullptr;
    switch (write) {
        case Write::WIFI_SETTINGS:
            return "disarm to change WiFi settings";
        case Write::DONGLES:
            return "disarm to change dongles";
        case Write::CRASH_DUMP:
            return "disarm to erase the dump";
        case Write::ESC_TUNING:
            return "disarm to enter tuning mode";
    }
    return "disarm first";
}

enum class EnterDecision : uint8_t { ENTER, WAIT_STARTUP, REFUSE_ARMED, REFUSE_DRIVING };

inline EnterDecision enter_decision(uint32_t now_ms, uint32_t combat_since_ms, uint32_t reentry_ms,
                                    bool armed, bool driving) {
    if (now_ms - combat_since_ms < reentry_ms) return EnterDecision::WAIT_STARTUP;
    if (armed) return EnterDecision::REFUSE_ARMED;
    if (driving) return EnterDecision::REFUSE_DRIVING;
    return EnterDecision::ENTER;
}
}  // namespace armed_guard
