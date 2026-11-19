#include <ina238_id.h>
#include <vbat_sensor.h>

using namespace vbat_sensor;

namespace {
const uint8_t REG_ADC_CONFIG = 0x01;
const uint8_t REG_VSHUNT = 0x04;
const uint8_t REG_VBUS = 0x05;
const uint8_t REG_DEVICE_ID = 0x3F;

const uint16_t ADC_CONFIG_VALUE = 0xBB69;

const float BUS_VOLTS_PER_LSB = 3.125e-3f;
const float SHUNT_VOLTS_PER_LSB = 5e-6f;
const float AMPS_PER_LSB = SHUNT_VOLTS_PER_LSB / SHUNT_OHMS;
}  // namespace

bool VbatSensor::begin(TwoWire *wire_bus) {
    wire = wire_bus;
    last_volts = NAN;
    last_amps = NAN;
    sample_timer_us = micros() - SAMPLE_INTERVAL_US;

    uint16_t device_id = 0;
    present = read_register(REG_DEVICE_ID, &device_id) && ina238_id::is_ina238(device_id);
    status.device_id = device_id;
    status.present = present;
    if (!present) return false;

    wire->beginTransmission(INA238_ADDRESS);
    wire->write(REG_ADC_CONFIG);
    wire->write((uint8_t)(ADC_CONFIG_VALUE >> 8));
    wire->write((uint8_t)(ADC_CONFIG_VALUE & 0xFF));
    status.last_error = i2c_status_from_wire(wire->endTransmission());
    present = status.last_error == I2C_OK;
    status.present = present;
    return present;
}

bool VbatSensor::read_register(uint8_t reg, uint16_t *value) {
    wire->beginTransmission(INA238_ADDRESS);
    wire->write(reg);
    status.last_error = i2c_status_from_wire(wire->endTransmission(false));
    if (status.last_error != I2C_OK) return false;
    if (wire->requestFrom(INA238_ADDRESS, (uint8_t)2) != 2) {
        status.last_error = I2C_SHORT_READ;
        return false;
    }
    uint16_t high = (uint16_t)wire->read();
    *value = (uint16_t)((high << 8) | (uint16_t)wire->read());
    return true;
}

void VbatSensor::update() {
    if (!present) return;
    uint32_t now = micros();
    if (now - sample_timer_us < next_interval_us) return;
    sample_timer_us = now;

    uint16_t bus_raw, shunt_raw;
    status.reads++;
    if (!read_register(REG_VBUS, &bus_raw)) {
        status.read_failures++;
        last_volts = NAN;
        last_amps = NAN;
        next_interval_us = RETRY_INTERVAL_US;
        return;
    }
    last_volts = (float)(int16_t)bus_raw * BUS_VOLTS_PER_LSB;
    status.last_ok_ms = millis();
    last_amps =
        read_register(REG_VSHUNT, &shunt_raw) ? (float)(int16_t)shunt_raw * AMPS_PER_LSB : NAN;
    next_interval_us = SAMPLE_INTERVAL_US;
}
