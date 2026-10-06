#pragma once
#include <Arduino.h>
#include <Wire.h>

// Bus-level I2C checks for the diagnostics page: which addresses answer, and whether a device is
// holding SDA or SCL low.
namespace i2c_bus {
const uint8_t MAX_FOUND = 16;

typedef struct {
    bool scanned;
    uint32_t scan_ms;  // millis() when the scan ran
    uint8_t count;
    uint8_t addresses[MAX_FOUND];
} scan_t;

typedef struct {
    bool sda_high;
    bool scl_high;
} lines_t;

// Probes every 7-bit address with an empty write. Absent addresses NACK in about 0.1 ms at
// 100 kHz, so a full scan blocks for ~15 ms: only run it with the robot disarmed.
void scan(TwoWire *wire, scan_t *result);

// Reads the idle level of both lines. Both should be high between transactions; a line stuck
// low means a device is holding it, and every transaction will fail until power is cycled.
lines_t read_lines(uint8_t sda_pin, uint8_t scl_pin);
}  // namespace i2c_bus
