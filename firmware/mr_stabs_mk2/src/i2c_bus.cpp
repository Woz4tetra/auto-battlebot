#include <driver/gpio.h>
#include <i2c_bus.h>

namespace i2c_bus {
void scan(TwoWire *wire, scan_t *result) {
    result->count = 0;
    for (uint8_t address = 0x08; address < 0x78; address++) {
        wire->beginTransmission(address);
        if (wire->endTransmission() == 0 && result->count < MAX_FOUND) {
            result->addresses[result->count++] = address;
        }
    }
    result->scanned = true;
    result->scan_ms = millis();
}

lines_t read_lines(uint8_t sda_pin, uint8_t scl_pin) {
    // The I2C driver leaves both pins as open-drain with input enabled, so the GPIO input
    // register reads the real line level without disturbing the bus.
    lines_t lines;
    lines.sda_high = gpio_get_level((gpio_num_t)sda_pin) != 0;
    lines.scl_high = gpio_get_level((gpio_num_t)scl_pin) != 0;
    return lines;
}
}  // namespace i2c_bus
