#include <vbat_sensor.h>

using namespace vbat_sensor;

namespace
{
    const uint8_t REG_ADC_CONFIG = 0x01;
    const uint8_t REG_VSHUNT = 0x04;
    const uint8_t REG_VBUS = 0x05;
    const uint8_t REG_DEVICE_ID = 0x3F;

    // DEVICE_ID bits 15..4 are the die ID, bits 3..0 the revision. The Matek board also ships
    // with an INA238 (die 0x238), whose VBUS and VSHUNT registers are 16 bits at different
    // scales; reading it with the INA228 scales would be wrong, so it is rejected.
    const uint16_t INA228_DIE_ID = 0x228;

    // MODE=1011 (shunt and bus voltage, continuous), VBUSCT=101 and VSHCT=101 (1052 us each),
    // VTCT=101 (unused), AVG=001 (4 samples). One averaged pair takes 8.4 ms, which fits inside
    // one SAMPLE_INTERVAL_US, so every read returns a fresh average and ESC switching ripple is
    // averaged out. A chip that browns out resets to 0xFB68 (continuous bus, shunt, and
    // temperature at 1052 us, no averaging), which still updates both readings every 3.2 ms.
    const uint16_t ADC_CONFIG_VALUE = 0xBB69;

    // VBUS and VSHUNT: 24 bits, bits 23..4 are a 20-bit two's complement reading.
    const float BUS_VOLTS_PER_LSB = 195.3125e-6f;
    // Current comes from the raw shunt voltage rather than the CURRENT register. CURRENT needs
    // SHUNT_CAL written, and a brownout clears it to 0, which would read as 0 A while the
    // voltage stayed valid. VSHUNT at the reset-default ADCRANGE=0 (+-163.84 mV) needs no setup:
    // 312.5 nV per LSB over the 200 uohm shunt is 1.5625 mA per LSB, +-819 A full scale.
    const float SHUNT_VOLTS_PER_LSB = 312.5e-9f;
    const float AMPS_PER_LSB = SHUNT_VOLTS_PER_LSB / SHUNT_OHMS;
}

bool VbatSensor::begin(TwoWire *wire_bus)
{
    wire = wire_bus;
    last_volts = NAN;
    last_amps = NAN;
    sample_timer_us = micros() - SAMPLE_INTERVAL_US;

    uint32_t device_id;
    present = read_register(REG_DEVICE_ID, 2, &device_id) && (device_id >> 4) == INA228_DIE_ID;
    if (!present)
        return false;

    wire->beginTransmission(INA228_ADDRESS);
    wire->write(REG_ADC_CONFIG);
    wire->write((uint8_t)(ADC_CONFIG_VALUE >> 8));
    wire->write((uint8_t)(ADC_CONFIG_VALUE & 0xFF));
    present = wire->endTransmission() == 0;
    return present;
}

bool VbatSensor::read_register(uint8_t reg, uint8_t num_bytes, uint32_t *value)
{
    wire->beginTransmission(INA228_ADDRESS);
    wire->write(reg);
    if (wire->endTransmission(false) != 0)
        return false;
    if (wire->requestFrom(INA228_ADDRESS, num_bytes) != num_bytes)
        return false;
    uint32_t raw = 0;
    for (uint8_t index = 0; index < num_bytes; index++)
        raw = (raw << 8) | (uint32_t)wire->read();
    *value = raw;
    return true;
}

bool VbatSensor::read_20_bit(uint8_t reg, int32_t *reading)
{
    uint32_t raw;
    if (!read_register(reg, 3, &raw))
        return false;
    // Shift the 24-bit register to the top of an int32 so the arithmetic shift sign-extends
    // the 20-bit reading.
    *reading = (int32_t)(raw << 8) >> 12;
    return true;
}

void VbatSensor::update()
{
    if (!present)
        return;
    uint32_t now = micros();
    if (now - sample_timer_us < SAMPLE_INTERVAL_US)
        return;
    sample_timer_us = now;

    int32_t bus_reading, shunt_reading;
    last_volts = read_20_bit(REG_VBUS, &bus_reading) ? (float)bus_reading * BUS_VOLTS_PER_LSB
                                                     : NAN;
    last_amps = read_20_bit(REG_VSHUNT, &shunt_reading) ? (float)shunt_reading * AMPS_PER_LSB
                                                        : NAN;
}
