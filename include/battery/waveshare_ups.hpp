#pragma once

#include "battery/battery_options.hpp"
#include "battery/battery_soc_estimator.hpp"

namespace auto_battlebot {

/** Reads the Waveshare UPS's INA219 over /dev/i2c-<options.i2c_bus>. Returns false, leaving
 *  `sample_out` invalid, when the bus or the chip does not answer. */
bool read_waveshare_ups_sample(const BatteryOptions &options, BatterySample &sample_out);

}  // namespace auto_battlebot
