#pragma once

#include <string>

namespace auto_battlebot {

struct BatteryOptions {
    int i2c_bus = 7;
    int i2c_address = 0x41;
    std::string ocv_table_file_path;
    double battery_capacity_ah = 5.0;
    double rest_current_threshold_a = 3.0;
    double rest_stability_delta_v = 0.2;
    double rest_window_sec = 20.0;
    double ocv_correction_gain = 0.08;
    double display_slew_pct_per_sec = 8.0;
    bool discharge_current_positive = true;
    double persist_interval_sec = 30.0;
    int invalid_read_grace = 3;
    std::string state_file_path;
};

}  // namespace auto_battlebot
