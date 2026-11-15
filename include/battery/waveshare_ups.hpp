#pragma once

#include "battery/battery_options.hpp"
#include "battery/battery_soc_estimator.hpp"

namespace auto_battlebot {

/** Read the Waveshare UPS INA219 battery sample. */
bool read_waveshare_ups_sample(const BatteryOptions &options, BatterySample &sample_out);

}  // namespace auto_battlebot
