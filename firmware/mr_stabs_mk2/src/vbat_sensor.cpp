#include <vbat_sensor.h>

using namespace vbat_sensor;

namespace
{
    const uint8_t REG_ADC_CONFIG = 0x01;
    const uint8_t REG_VSHUNT = 0x04;
    const uint8_t REG_VBUS = 0x05;
    const uint8_t REG_DEVICE_ID = 0x3F;

    // DEVICE_ID bits 15..4 are the die ID, bits 3..0 the revision. An INA228 (die 0x228) answers
    // at the same address with 24-bit VBUS and VSHUNT at different scales, so anything else is
    // rejected rather than misread.
    const uint16_t INA238_DIE_ID = 0x238;

    // MODE=1011 (shunt and bus voltage, continuous), VBUSCT=101 and VSHCT=101 (1052 us each),
    // VTCT=101 (unused), AVG=001 (4 samples). One averaged pair takes 8.4 ms, which fits inside
    // one SAMPLE_INTERVAL_US, so every read returns a fresh average and ESC switching ripple is
    // averaged out. A chip that browns out resets to 0xFB68 (continuous bus, shunt, and
    // temperature at 1052 us, no averaging), which still updates both readings every 3.2 ms.
    const uint16_t ADC_CONFIG_VALUE = 0xBB69;

    // VBUS and VSHUNT: 16-bit two's complement.
    const float BUS_VOLTS_PER_LSB = 3.125e-3f;
    // Current comes from the raw shunt voltage at the reset-default ADCRANGE=0 (+-163.84 mV).
    // ADCRANGE=1 would give 4x the resolution, but a brownout clears it back to 0 and the
    // current would then read 4x high with nothing to flag it. 5 uV per LSB over the 0.5 mohm
    // shunt is 10 mA per LSB, +-327 A full scale.
    const float SHUNT_VOLTS_PER_LSB = 5e-6f;
    const float AMPS_PER_LSB = SHUNT_VOLTS_PER_LSB / SHUNT_OHMS;
}

bool VbatSensor::begin(TwoWire *wire_bus)
{
    wire = wire_bus;
    last_volts = NAN;
    last_amps = NAN;
    sample_timer_us = micros() - SAMPLE_INTERVAL_US;

    uint16_t device_id = 0;
    present = read_register(REG_DEVICE_ID, &device_id) && (device_id >> 4) == INA238_DIE_ID;
    status.device_id = device_id;
    status.present = present;
    if (!present)
        return false;

    wire->beginTransmission(INA238_ADDRESS);
    wire->write(REG_ADC_CONFIG);
    wire->write((uint8_t)(ADC_CONFIG_VALUE >> 8));
    wire->write((uint8_t)(ADC_CONFIG_VALUE & 0xFF));
    status.last_error = wire->endTransmission();
    present = status.last_error == 0;
    status.present = present;
    return present;
}

bool VbatSensor::read_register(uint8_t reg, uint16_t *value)
{
    wire->beginTransmission(INA238_ADDRESS);
    wire->write(reg);
    status.last_error = wire->endTransmission(false);
    if (status.last_error != 0)
        return false;
    if (wire->requestFrom(INA238_ADDRESS, (uint8_t)2) != 2)
    {
        status.last_error = 6;
        return false;
    }
    uint16_t high = (uint16_t)wire->read();
    *value = (uint16_t)((high << 8) | (uint16_t)wire->read());
    return true;
}

void VbatSensor::update()
{
    if (!present)
        return;
    uint32_t now = micros();
    if (now - sample_timer_us < next_interval_us)
        return;
    sample_timer_us = now;

    uint16_t bus_raw, shunt_raw;
    status.reads++;
    if (!read_register(REG_VBUS, &bus_raw))
    {
        status.read_failures++;
        last_volts = NAN;
        last_amps = NAN;
        next_interval_us = RETRY_INTERVAL_US;
        return;
    }
    last_volts = (float)(int16_t)bus_raw * BUS_VOLTS_PER_LSB;
    status.last_ok_ms = millis();
    last_amps = read_register(REG_VSHUNT, &shunt_raw)
                    ? (float)(int16_t)shunt_raw * AMPS_PER_LSB
                    : NAN;
    next_interval_us = SAMPLE_INTERVAL_US;
}
