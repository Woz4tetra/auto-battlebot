#pragma once

#include <stddef.h>
#include <stdint.h>

#include "espnow_protocol.h"

namespace espnow_protocol {

class CommandRepeater {
   public:
    CommandRepeater(uint32_t silence_ms, uint32_t period_us)
        : _silence_ms(silence_ms), _period_us(period_us) {}

    void on_command(const uint8_t *payload, uint8_t length, uint32_t now_ms, uint32_t now_us) {
        _length = encode_air_frame(MSG_COMMAND, payload, length, _frame);
        _last_command_ms = now_ms;
        _last_repeat_us = now_us;
    }

    bool commanding(uint32_t now_ms) const {
        return _length > 0 && now_ms - _last_command_ms < _silence_ms;
    }

    bool repeat_due(uint32_t now_ms, uint32_t now_us) {
        if (!commanding(now_ms)) return false;
        if (now_us - _last_repeat_us < _period_us) return false;
        _last_repeat_us = now_us;
        return true;
    }

    const uint8_t *frame() const { return _frame; }
    size_t length() const { return _length; }

   private:
    uint32_t _silence_ms;
    uint32_t _period_us;
    uint8_t _frame[AIR_MAX_FRAME] = {};
    size_t _length = 0;
    uint32_t _last_command_ms = 0;
    uint32_t _last_repeat_us = 0;
};

}  // namespace espnow_protocol
