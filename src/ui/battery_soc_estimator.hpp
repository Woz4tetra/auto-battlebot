#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "lvgl_platform_bound/lvgl_ui_services.hpp"

namespace auto_battlebot::ui_internal {

enum class BatteryFlowDirection { Discharge, Charge };

struct BatterySample {
    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
    uint16_t raw_bus_voltage = 0;
    int16_t raw_current = 0;
    int16_t raw_power = 0;
    double bus_voltage_v = 0.0;
    double current_a = 0.0;
    double power_w = 0.0;
    bool valid = false;
};

struct OcvTablePoint {
    double voltage_v = 0.0;
    double soc_percent = 0.0;
};

class BatterySocEstimator {
   public:
    explicit BatterySocEstimator(const BatteryOptions &options = {});

    BatteryReading update(const BatterySample &sample);
    void log_diagnostics(const BatterySample &sample, const BatteryReading &reading,
                         const BatteryOptions &source_options, bool read_ok) const;
    double debug_soc_percent() const;
    double debug_table_soc(size_t idx, BatteryFlowDirection direction) const;

   private:
    bool load_static_ocv_table();
    double lookup_ocv_soc(double voltage_v, BatteryFlowDirection direction) const;
    bool load_state();
    bool save_state() const;

    BatteryOptions options_;
    std::vector<OcvTablePoint> ocv_discharge_;
    std::vector<OcvTablePoint> ocv_charge_;
    double soc_percent_ = 50.0;
    double display_percent_ = 50.0;
    bool has_estimate_ = false;
    bool has_last_sample_ = false;
    int invalid_streak_ = 0;
    double near_rest_sec_ = 0.0;
    double last_voltage_v_ = 0.0;
    bool table_loaded_from_file_ = false;
    std::string table_load_status_ = "fallback_default";
    BatteryFlowDirection last_direction_ = BatteryFlowDirection::Discharge;
    std::chrono::steady_clock::time_point last_timestamp_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_persist_ =
        std::chrono::steady_clock::time_point::min();
};

}  // namespace auto_battlebot::ui_internal
