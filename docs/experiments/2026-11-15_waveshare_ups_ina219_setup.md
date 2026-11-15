# 2026-11-15 waveshare ups ina219 setup

These facts were source comments in `src/` and moved here on 2026-11-15 so the code carries names and tests instead of prose. Each section names the file and the code it described.

### `src/battery/waveshare_ups.cpp`

Beside `constexpr uint16_t INA219_CONFIG_16V_5A_CONTINUOUS = 0x0EEF;`:

Config register bits matching the Waveshare reference setup (16V range, gain/ADC, continuous mode).

### `src/battery/waveshare_ups.cpp`

Beside `constexpr double INA219_SHUNT_RESISTANCE_OHM = 0.01;`:

Shunt resistor value on the UPS board; used with calibration to convert raw current counts.
