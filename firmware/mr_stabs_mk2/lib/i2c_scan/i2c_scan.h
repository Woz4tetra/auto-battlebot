#pragma once

#include <stddef.h>
#include <stdint.h>

namespace i2c_scan {
const uint8_t FIRST_ADDRESS = 0x08;
const uint8_t LAST_ADDRESS_EXCLUSIVE = 0x78;

template <class Wire, class BeforeProbe>
uint8_t scan(Wire &wire, BeforeProbe before_probe, uint8_t *found, uint8_t capacity) {
    uint8_t count = 0;
    for (uint8_t address = FIRST_ADDRESS; address < LAST_ADDRESS_EXCLUSIVE; address++) {
        before_probe();
        wire.beginTransmission(address);
        if (wire.endTransmission() == 0 && count < capacity) found[count++] = address;
    }
    return count;
}
}  // namespace i2c_scan
