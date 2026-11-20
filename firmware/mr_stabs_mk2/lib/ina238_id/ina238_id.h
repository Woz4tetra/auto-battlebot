#pragma once

#include <stdint.h>

namespace ina238_id {
constexpr uint16_t INA238_DIE_ID = 0x238;
constexpr uint16_t INA228_DIE_ID = 0x228;

constexpr uint16_t die_id(uint16_t device_id) { return device_id >> 4; }

constexpr bool is_ina238(uint16_t device_id) { return die_id(device_id) == INA238_DIE_ID; }

template <typename ReadId, typename WriteConfig>
bool begin_sequence(ReadId read_id, WriteConfig write_config, uint16_t &device_id) {
    if (!read_id(&device_id)) return false;
    if (!is_ina238(device_id)) return false;
    return write_config();
}
}  // namespace ina238_id
