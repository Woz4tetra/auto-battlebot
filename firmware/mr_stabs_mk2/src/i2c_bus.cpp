#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include <i2c_bus.h>

namespace i2c_bus {
bool scan(TwoWire *wire, scan_t *result, bool armed) {
    if (armed) return false;
    result->count = 0;
    for (uint8_t address = 0x08; address < 0x78; address++) {
        esp_task_wdt_reset();
        wire->beginTransmission(address);
        if (wire->endTransmission() == 0 && result->count < MAX_FOUND) {
            result->addresses[result->count++] = address;
        }
    }
    result->scanned = true;
    result->scanned_at_ms = millis();
    return true;
}

lines_t read_lines(uint8_t sda_pin, uint8_t scl_pin) {
    lines_t lines;
    lines.sda_high = gpio_get_level((gpio_num_t)sda_pin) != 0;
    lines.scl_high = gpio_get_level((gpio_num_t)scl_pin) != 0;
    return lines;
}
}  // namespace i2c_bus
