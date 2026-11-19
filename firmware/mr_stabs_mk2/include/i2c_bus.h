#pragma once
#include <Arduino.h>
#include <Wire.h>

namespace i2c_bus {
const uint8_t MAX_FOUND = 16;

typedef struct {
    bool scanned;
    uint32_t scanned_at_ms;
    uint8_t count;
    uint8_t addresses[MAX_FOUND];
} scan_t;

typedef struct {
    bool sda_high;
    bool scl_high;
} lines_t;

bool scan(TwoWire *wire, scan_t *result, bool armed);

lines_t read_lines(uint8_t sda_pin, uint8_t scl_pin);
}  // namespace i2c_bus
