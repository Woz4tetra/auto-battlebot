#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <i2c_status.h>
#include <ina238_sampler.h>

namespace vbat_sensor {
const uint8_t INA238_ADDRESS = 0x45;

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
    bool begin(TwoWire *wire = &Wire1);

    void update();

    float get_pack_volts() const { return sampler.volts(); }

    float get_pack_amps_discharge() const { return sampler.amps(); }

    const status_t &get_status() const { return status; }

   private:
    TwoWire *wire = nullptr;
    bool present = false;
    ina238_id::Sampler sampler;
    status_t status;

    bool read_register(uint8_t reg, uint16_t *value);
};
}  // namespace vbat_sensor
