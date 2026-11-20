#pragma once

#include "battery/battery_options.hpp"
#include "battery/battery_soc_estimator.hpp"

namespace auto_battlebot {

bool read_waveshare_ups_sample(const BatteryOptions &options, BatterySample &sample_out);

}  // namespace auto_battlebot
