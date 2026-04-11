#pragma once

#include <string>

namespace auto_battlebot {

struct BatteryOptions {
    int i2c_bus = 7;
    int i2c_address = 0x41;
    double battery_capacity_ah = 5.0;
    double rest_current_threshold_a = 0.12;
    double rest_stability_delta_v = 0.01;
    double rest_window_sec = 20.0;
    double ocv_correction_gain = 0.08;
    double display_slew_pct_per_sec = 8.0;
    bool self_tune_enabled = true;
    double self_tune_learning_rate = 0.02;
    double self_tune_max_delta_pct = 0.3;
    int self_tune_min_samples = 5;
    bool discharge_current_positive = true;
    double persist_interval_sec = 30.0;
    int invalid_read_grace = 3;
    std::string state_file_path;
};

}  // namespace auto_battlebot
