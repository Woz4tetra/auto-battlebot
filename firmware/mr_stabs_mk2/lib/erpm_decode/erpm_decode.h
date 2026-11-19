#pragma once
#include <cstddef>
#include <cstdint>

namespace erpm_decode {

const uint32_t REPLY_BITS = 21;
const uint16_t PAYLOAD_STOPPED = 0x0FFF;

const uint8_t GCR_ENCODE[16] = {0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17,
                                0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F};

inline uint16_t build_packet(uint16_t throttle, bool telemetry) {
    const uint16_t value = static_cast<uint16_t>((throttle << 1) | (telemetry ? 1 : 0));
    const uint16_t crc = static_cast<uint16_t>(~(value ^ (value >> 4) ^ (value >> 8)) & 0x0F);
    return static_cast<uint16_t>((value << 4) | crc);
}

inline uint8_t gcr_decode(uint32_t group) {
    for (uint8_t nibble = 0; nibble < 16; nibble++) {
        if (GCR_ENCODE[nibble] == group) return nibble;
    }
    return 0xFF;
}

inline bool runs_to_bits(const uint32_t *runs, size_t count, float bit_ticks, uint32_t &bits) {
    if (count == 0 || bit_ticks <= 0.0f) return false;
    uint32_t value = 0;
    uint32_t total = 0;
    for (size_t i = 0; i < count && total < REPLY_BITS; i++) {
        uint32_t len = static_cast<uint32_t>(runs[i] / bit_ticks + 0.5f);
        if (len == 0) len = 1;
        const bool last = i + 1 == count;
        if (total + len > REPLY_BITS) {
            if (!last) return false;
            len = REPLY_BITS - total;
        } else if (last) {
            len = REPLY_BITS - total;
        }
        value = (value << len) | (1u << (len - 1));
        total += len;
    }
    if (total != REPLY_BITS) return false;
    bits = value;
    return true;
}

inline bool bits_to_payload(uint32_t bits, uint16_t &payload) {
    uint32_t value = 0;
    for (int group = 3; group >= 0; group--) {
        const uint8_t nibble = gcr_decode((bits >> (5 * group)) & 0x1F);
        if (nibble == 0xFF) return false;
        value = (value << 4) | nibble;
    }
    uint32_t crc = value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12);
    if ((crc & 0xF) != 0xF) return false;
    payload = static_cast<uint16_t>(value >> 4);
    return true;
}

inline bool payload_to_erpm(uint16_t payload, uint32_t &erpm) {
    if (payload == PAYLOAD_STOPPED) {
        erpm = 0;
        return true;
    }
    const uint32_t period_us = (payload & 0x1FFu) << (payload >> 9);
    if (period_us == 0) return false;
    erpm = (60000000u + period_us / 2) / period_us;
    return true;
}

inline bool decode(const uint32_t *runs, size_t count, float bit_ticks, uint32_t &erpm) {
    uint32_t bits = 0;
    uint16_t payload = 0;
    return runs_to_bits(runs, count, bit_ticks, bits) && bits_to_payload(bits, payload) &&
           payload_to_erpm(payload, erpm);
}

inline float measured_bit_ticks(const uint32_t *runs, size_t count, float nominal_ticks) {
    float sum = 0.0f;
    int n = 0;
    for (size_t i = 0; i + 1 < count; i++) {
        if (runs[i] > 0.65f * nominal_ticks && runs[i] < 1.35f * nominal_ticks) {
            sum += runs[i];
            n++;
        }
    }
    return n > 0 ? sum / n : nominal_ticks;
}

inline bool decode_measured(const uint32_t *runs, size_t count, float nominal_ticks,
                            uint32_t &erpm) {
    return decode(runs, count, measured_bit_ticks(runs, count, nominal_ticks), erpm);
}

}  // namespace erpm_decode
