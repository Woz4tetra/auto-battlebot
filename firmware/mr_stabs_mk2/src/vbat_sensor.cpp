#include <vbat_sensor.h>

using namespace vbat_sensor;

namespace
{
    const uint8_t REG_CONFIG = 0x00;
    const uint8_t REG_BUS_VOLTAGE = 0x02;

    // BRNG=1 (32 V range), PG=/8, BADC=1011 (8-sample average, 4.26 ms), SADC=0011 (unused),
    // MODE=110 (bus voltage only, continuous). The 4.26 ms average fits inside one
    // SAMPLE_INTERVAL_US, so every read returns a fresh average and ESC switching ripple is
    // averaged out. A chip that browns out resets to 0x399F, which is also the 32 V range, so
    // readings stay valid after a reset.
    const uint16_t CONFIG_VALUE = 0x3D9E;

    // Bus voltage register: bits 15..3 are the reading, 4 mV per LSB.
    const float BUS_VOLTS_PER_LSB = 0.004f;
}

bool VbatSensor::begin(TwoWire *wire_bus)
{
    wire = wire_bus;
    wire->beginTransmission(INA219_ADDRESS);
    wire->write(REG_CONFIG);
    wire->write((uint8_t)(CONFIG_VALUE >> 8));
    wire->write((uint8_t)(CONFIG_VALUE & 0xFF));
    present = wire->endTransmission() == 0;
    last_volts = NAN;
    sample_timer_us = micros() - SAMPLE_INTERVAL_US;
    return present;
}

bool VbatSensor::read_bus_voltage(float *volts)
{
    wire->beginTransmission(INA219_ADDRESS);
    wire->write(REG_BUS_VOLTAGE);
    if (wire->endTransmission(false) != 0)
        return false;
    if (wire->requestFrom(INA219_ADDRESS, (uint8_t)2) != 2)
        return false;
    uint16_t high = (uint16_t)wire->read();
    uint16_t low = (uint16_t)wire->read();
    uint16_t raw = (high << 8) | low;
    *volts = (float)(raw >> 3) * BUS_VOLTS_PER_LSB * VBAT_CAL_SCALE;
    return true;
}

float VbatSensor::update()
{
    if (!present)
        return NAN;
    uint32_t now = micros();
    if (now - sample_timer_us < SAMPLE_INTERVAL_US)
        return last_volts;
    sample_timer_us = now;

    float volts;
    last_volts = read_bus_voltage(&volts) ? volts : NAN;
    return last_volts;
}
