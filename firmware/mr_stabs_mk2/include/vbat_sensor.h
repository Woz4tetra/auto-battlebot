#pragma once
#include <Arduino.h>
#include <Wire.h>

// Pack voltage and current from the main board's TI INA238 on the BNO055's I2C bus (Wire1).
//
// Board wiring (hardware/mr_stabs_mk2_pcb/pcba/design.py):
// - High side: switched pack + (BAT_IN) through the 0.5 mohm shunt R1 to VBATT, the ESC bus.
//   Vin+ and Vin- are Kelvin taps on the shunt pads through 10 ohm filter resistors. VBUS sits on
//   the pack side of the shunt, so it reads the shunt's drop high: at most 35 mV in a 70 A stall.
// - VS, A0 and A1 on +3V3, which sets address 0x45.
namespace vbat_sensor
{
    const uint8_t INA238_ADDRESS = 0x45;

    // R1's nominal value. Its tolerance and the pour resistance inside the Kelvin taps set the
    // current reading's gain error; calibrate it against a clamp meter.
    const float SHUNT_OHMS = 0.0005f;

    // A sample is two reads (bus voltage, then shunt voltage), each one register-pointer write
    // plus a 2-byte read, about 47 SCL clocks. Wire1 runs at the default 100 kHz, so that is
    // ~1 ms on the wire plus driver overhead, blocking. The control loop has no fixed period
    // (it varies with radio state, BNO055 reads, and recording mode), so sampling runs on a time
    // interval instead of every Nth loop. 10 ms still resolves a punch's sag and current spike,
    // which last 100s of ms. Loops in between repeat the last values.
    const uint32_t SAMPLE_INTERVAL_US = 10000;
    // After a failed read (unplugged cable), wait this long before trying again, so a missing
    // chip costs one blocked transaction per second rather than one per sample.
    const uint32_t RETRY_INTERVAL_US = 1000000;

    // INA238 health for the diagnostics page. last_error is a Wire endTransmission() code
    // (0 ok, 2 address NACK, 3 data NACK, 4 bus error, 5 timeout), 6 for a short read, or 255
    // before the first read.
    struct status_t
    {
        bool present = false;
        uint16_t device_id = 0;  // DEVICE_ID at boot, 0x238x for an INA238
        uint8_t last_error = 255;
        uint32_t reads = 0;
        uint32_t read_failures = 0;
        uint32_t last_ok_ms = 0;
    };

    class VbatSensor
    {
    public:
        // Checks the device ID and writes the ADC config. Returns false and reports NaN from
        // then on if the chip does not ACK or is not an INA238. Never blocks beyond two I2C
        // transactions.
        bool begin(TwoWire *wire = &Wire1);

        // Samples voltage and current when SAMPLE_INTERVAL_US has passed, else does nothing.
        void update();

        // Pack volts. NaN when the chip was absent at boot or the last read failed.
        float get_volts() const { return last_volts; }

        // Pack amps, positive while discharging. NaN when the chip was absent at boot or the
        // last read failed.
        float get_amps() const { return last_amps; }

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
}
