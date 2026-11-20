#include <ina238_id.h>
#include <ina238_sampler.h>
#include <vbat_sensor.h>

using namespace vbat_sensor;

namespace {
const uint8_t REG_ADC_CONFIG = 0x01;
const uint8_t REG_DEVICE_ID = 0x3F;

const uint16_t ADC_CONFIG_VALUE = 0xBB69;
}  // namespace

bool VbatSensor::begin(TwoWire *wire_bus) {
    wire = wire_bus;

    uint16_t device_id = 0;
    present = ina238_id::begin_sequence(
        [this](uint16_t *id) { return read_register(REG_DEVICE_ID, id); },
        [this]() {
            wire->beginTransmission(INA238_ADDRESS);
            wire->write(REG_ADC_CONFIG);
            wire->write((uint8_t)(ADC_CONFIG_VALUE >> 8));
            wire->write((uint8_t)(ADC_CONFIG_VALUE & 0xFF));
            status.last_error = i2c_status_from_wire(wire->endTransmission());
            return status.last_error == I2C_OK;
        },
        device_id);
    sampler.begin(present, micros());
    status.device_id = device_id;
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
    const ina238_id::SampleResult result =
        sampler.update(micros(), [this](uint8_t reg, uint16_t *value) {
            if (reg == ina238_id::Sampler::BUS_REGISTER) status.reads++;
            return read_register(reg, value);
        });
    if (result == ina238_id::SampleResult::READ_FAILED) status.read_failures++;
    if (result == ina238_id::SampleResult::READ_OK) status.last_ok_ms = millis();
}
