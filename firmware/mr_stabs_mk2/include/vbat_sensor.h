#pragma once
#include <Arduino.h>
#include <Wire.h>

// Pack voltage from an INA219 breakout on the BNO055's I2C bus (Wire1). Only the bus voltage
// register is read; current is not measured.
//
// Wiring (INA219 breakout):
// - Pack + -> 1 kohm series resistor -> VIN+ only. VIN- stays unconnected, so the chip reads the
//   bus through the onboard 0.1 ohm shunt, which carries only the chip's microamps. Wiring pack +
//   to both VIN+ and VIN- would put the shunt in parallel with the resistor and bypass it.
// - After the resistor, a 0.1 uF cap to ground (100 us RC, filters the power-on ringing) or an
//   SMAJ18A TVS to ground. The bus input is 26 V absolute max against 17.4 V for a full 4S LiHV
//   pack, and closing the switch rings the leads against the ESC input caps up to nearly twice
//   pack voltage. The resistor also limits fault current, so a failed chip cannot put pack
//   voltage onto the shared I2C bus.
// - VCC from the QT Py's 3.3 V, so the I2C pull-ups stay at 3.3 V.
// - GND at battery negative.
namespace vbat_sensor
{
    const uint8_t INA219_ADDRESS = 0x40;

    // Multiplies the raw reading. Set once against a multimeter at rest and during a punch: the
    // 1 kohm series resistor shifts the reading about 0.3%.
    const float VBAT_CAL_SCALE = 1.0f;

    // A read is one register-pointer write plus a 2-byte read, about 47 SCL clocks. Wire1 runs
    // at the default 100 kHz, so that is ~0.5 ms on the wire plus driver overhead, blocking. The
    // control loop has no fixed period (it varies with radio state, BNO055 reads, and recording
    // mode), so the read runs on a time interval instead of every Nth loop. 10 ms costs one
    // ~0.5 ms stretch per 10 ms and still resolves a punch's sag, which lasts 100s of ms.
    // Loops in between repeat the last value.
    const uint32_t SAMPLE_INTERVAL_US = 10000;

    class VbatSensor
    {
    public:
        // Writes the config register. Returns false and reports NaN from then on if the chip
        // does not ACK. Never blocks beyond one I2C transaction.
        bool begin(TwoWire *wire = &Wire1);

        // Reads the bus voltage when SAMPLE_INTERVAL_US has passed, else returns the last value.
        // NaN when the chip was absent at boot or the last read failed.
        float update();

    private:
        TwoWire *wire = nullptr;
        bool present = false;
        float last_volts = NAN;
        uint32_t sample_timer_us = 0;

        bool read_bus_voltage(float *volts);
    };
}
