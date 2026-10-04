#pragma once
#include <Arduino.h>
#include <Wire.h>

// Pack voltage and current from a Matek I2C-INA-BM (TI INA228) on the BNO055's I2C bus (Wire1).
//
// Wiring (Matek I2C-INA-BM):
// - Battery + and the ESC + lead solder to the two sides of the onboard 200 uohm shunt, as close
//   to it as possible. The VBUS sense input is on board and rated 0-85 V, so no series resistor
//   or clamp is needed for a 4S pack.
// - JST-GH-4P to the QT Py: 5 V, GND, SCL, SDA. The board takes 4-9 V and runs the INA228 from
//   its own 3.3 V regulator.
// - Default address 0x45 (decimal 69). 0x44 and 0x41 are the alternatives.
namespace vbat_sensor
{
    const uint8_t INA228_ADDRESS = 0x45;

    // Matek's nominal shunt. Its tolerance is the current reading's: about +-2%.
    const float SHUNT_OHMS = 0.0002f;

    // A sample is two reads (bus voltage, then shunt voltage), each one register-pointer write
    // plus a 3-byte read, about 56 SCL clocks. Wire1 runs at the default 100 kHz, so that is
    // ~1.2 ms on the wire plus driver overhead, blocking. The control loop has no fixed period
    // (it varies with radio state, BNO055 reads, and recording mode), so sampling runs on a time
    // interval instead of every Nth loop. 10 ms still resolves a punch's sag and current spike,
    // which last 100s of ms. Loops in between repeat the last values.
    const uint32_t SAMPLE_INTERVAL_US = 10000;

    class VbatSensor
    {
    public:
        // Checks the device ID and writes the ADC config. Returns false and reports NaN from
        // then on if the chip does not ACK or is not an INA228. Never blocks beyond two I2C
        // transactions.
        bool begin(TwoWire *wire = &Wire1);

        // Samples voltage and current when SAMPLE_INTERVAL_US has passed, else does nothing.
        void update();

        // Pack volts. NaN when the chip was absent at boot or the last read failed.
        float get_volts() const { return last_volts; }

        // Pack amps, positive while discharging. NaN when the chip was absent at boot or the
        // last read failed.
        float get_amps() const { return last_amps; }

    private:
        TwoWire *wire = nullptr;
        bool present = false;
        float last_volts = NAN;
        float last_amps = NAN;
        uint32_t sample_timer_us = 0;

        bool read_register(uint8_t reg, uint8_t num_bytes, uint32_t *value);
        bool read_20_bit(uint8_t reg, int32_t *reading);
    };
}
