#pragma once

#include <stdint.h>

namespace ina238_id {
constexpr uint16_t INA238_DIE_ID = 0x238;
constexpr uint16_t INA228_DIE_ID = 0x228;

constexpr uint16_t die_id(uint16_t device_id) { return device_id >> 4; }

constexpr bool is_ina238(uint16_t device_id) { return die_id(device_id) == INA238_DIE_ID; }
}  // namespace ina238_id
