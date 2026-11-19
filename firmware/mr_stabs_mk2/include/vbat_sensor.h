#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <i2c_status.h>

namespace vbat_sensor {
const uint8_t INA238_ADDRESS = 0x45;

const float SHUNT_OHMS = 0.0005f;

const uint32_t SAMPLE_INTERVAL_US = 10000;
const uint32_t RETRY_INTERVAL_US = 1000000;

struct status_t {
    bool present = false;
    uint16_t device_id = 0;
    I2cStatus last_error = I2C_NOT_TRIED;
    uint32_t reads = 0;
    uint32_t read_failures = 0;
    uint32_t last_ok_ms = 0;
};

class VbatSensor {
   public:
    // Checks the device ID and writes the ADC config. Returns false and reports NaN from
    // then on if the chip does not ACK or is not an INA238. Never blocks beyond two I2C
    // transactions.
    bool begin(TwoWire *wire = &Wire1);

    void update();

    float get_pack_volts() const { return last_volts; }

    float get_pack_amps_discharge() const { return last_amps; }

    const status_t &get_status() const { return status; }

   private:
    TwoWire *wire = nullptr;
    bool present = false;
    float last_volts = NAN;
    float last_amps = NAN;
    uint32_t sample_timer_us = 0;
    uint32_t next_interval_us = SAMPLE_INTERVAL_US;
    status_t status;

    bool read_register(uint8_t reg, uint16_t *value);
};
}  // namespace vbat_sensor
