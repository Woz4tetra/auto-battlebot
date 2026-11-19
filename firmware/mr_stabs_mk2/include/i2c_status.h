#pragma once
#include <stdint.h>

enum I2cStatus : uint8_t {
    I2C_OK = 0,
    I2C_DATA_TOO_LONG = 1,
    I2C_ADDRESS_NACK = 2,
    I2C_DATA_NACK = 3,
    I2C_BUS_ERROR = 4,
    I2C_TIMEOUT = 5,
    I2C_SHORT_READ = 6,
    I2C_NOT_TRIED = 255,
};

inline I2cStatus i2c_status_from_wire(uint8_t end_transmission_code) {
    return static_cast<I2cStatus>(end_transmission_code);
}
