#pragma once

#include <armed_guard.h>
#include <stdint.h>

namespace pair_decision {
enum class Decision : uint8_t { ADD, REFUSE_ARMED, REFUSE_BAD_MAC, REFUSE_EXISTS, REFUSE_FULL };

inline Decision decide(bool armed, uint8_t first_mac_byte, bool already_paired,
                       uint8_t paired_count, uint8_t max_paired) {
    if (armed_guard::refusal(armed_guard::Write::DONGLES, armed) != nullptr)
        return Decision::REFUSE_ARMED;
    if ((first_mac_byte & 0x01) != 0) return Decision::REFUSE_BAD_MAC;
    if (already_paired) return Decision::REFUSE_EXISTS;
    if (paired_count >= max_paired) return Decision::REFUSE_FULL;
    return Decision::ADD;
}
}  // namespace pair_decision
