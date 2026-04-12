#include "battery_soc_estimator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include "diagnostics_logger/diagnostics_logger.hpp"
#include "directories.hpp"
#include <toml++/toml.h>

namespace auto_battlebot::ui_internal {
namespace {

constexpr int kStateVersion = 1;

std::vector<OcvTablePoint> make_default_discharge_table() {
    return {
        {.voltage_v = 9.0, .soc_percent = 0.0},    {.voltage_v = 10.2, .soc_percent = 15.0},
        {.voltage_v = 10.8, .soc_percent = 35.0},  {.voltage_v = 11.3, .soc_percent = 55.0},
        {.voltage_v = 11.8, .soc_percent = 75.0},  {.voltage_v = 12.2, .soc_percent = 90.0},
        {.voltage_v = 12.6, .soc_percent = 100.0},
    };
}

std::vector<OcvTablePoint> make_default_charge_table() {
    return {
        {.voltage_v = 9.2, .soc_percent = 0.0},     {.voltage_v = 10.35, .soc_percent = 15.0},
        {.voltage_v = 10.95, .soc_percent = 35.0},  {.voltage_v = 11.45, .soc_percent = 55.0},
        {.voltage_v = 11.95, .soc_percent = 75.0},  {.voltage_v = 12.35, .soc_percent = 90.0},
        {.voltage_v = 12.75, .soc_percent = 100.0},
    };
}

std::string trim_copy(const std::string &in) {
    size_t start = 0;
    while (start < in.size() && std::isspace(static_cast<unsigned char>(in[start]))) start++;
    size_t end = in.size();
    while (end > start && std::isspace(static_cast<unsigned char>(in[end - 1]))) end--;
    return in.substr(start, end - start);
}

double parse_double_or(const std::unordered_map<std::string, std::string> &kv,
                       const std::string &key, double fallback) {
    auto it = kv.find(key);
    if (it == kv.end()) return fallback;
    try {
        return std::stod(it->second);
    } catch (...) {
        return fallback;
    }
}

int parse_int_or(const std::unordered_map<std::string, std::string> &kv, const std::string &key,
                 int fallback) {
    auto it = kv.find(key);
    if (it == kv.end()) return fallback;
    try {
        return std::stoi(it->second);
    } catch (...) {
        return fallback;
    }
}

std::string default_state_path() {
    return auto_battlebot::get_project_path("data/battery_soc_state.txt").string();
}

std::string default_ocv_table_path() {
    return auto_battlebot::get_project_path("data/battery_ocv_table.toml").string();
}

bool parse_ocv_section(const toml::table &root, const std::string &section_name,
                       std::vector<OcvTablePoint> &out, std::string &error) {
    const toml::table *section = root[section_name].as_table();
    if (!section) {
        error = "missing section [" + section_name + "]";
        return false;
    }
    const toml::array *voltage_arr = (*section)["voltage_v"].as_array();
    const toml::array *soc_arr = (*section)["soc_percent"].as_array();
    if (!voltage_arr || !soc_arr) {
        error = "section [" + section_name + "] missing voltage_v or soc_percent array";
        return false;
    }
    if (voltage_arr->empty() || soc_arr->empty() || voltage_arr->size() != soc_arr->size()) {
        error = "section [" + section_name + "] has invalid array sizes";
        return false;
    }

    out.clear();
    out.reserve(voltage_arr->size());
    double last_voltage = -1e9;
    for (size_t i = 0; i < voltage_arr->size(); ++i) {
        const auto v = (*voltage_arr)[i].value<double>();
        const auto s = (*soc_arr)[i].value<double>();
        if (!v || !s) {
            error = "section [" + section_name + "] has non-numeric entries";
            return false;
        }
        if (i > 0 && *v <= last_voltage) {
            error = "section [" + section_name + "] voltage_v must be strictly increasing";
            return false;
        }
        if (*s < 0.0 || *s > 100.0) {
            error = "section [" + section_name + "] soc_percent must be in [0,100]";
            return false;
        }
        out.push_back({.voltage_v = *v, .soc_percent = *s});
        last_voltage = *v;
    }
    return true;
}

}  // namespace

BatterySocEstimator::BatterySocEstimator(const BatteryOptions &options) : options_(options) {
    ocv_discharge_ = make_default_discharge_table();
    ocv_charge_ = make_default_charge_table();
    if (options_.ocv_table_file_path.empty()) options_.ocv_table_file_path = default_ocv_table_path();
    table_loaded_from_file_ = load_static_ocv_table();
    if (options_.state_file_path.empty()) options_.state_file_path = default_state_path();
    (void)load_state();
}

bool BatterySocEstimator::load_static_ocv_table() {
    try {
        const toml::table root = toml::parse_file(options_.ocv_table_file_path);
        std::vector<OcvTablePoint> loaded_discharge;
        std::vector<OcvTablePoint> loaded_charge;
        std::string error;
        if (!parse_ocv_section(root, "discharge", loaded_discharge, error)) {
            table_load_status_ = "fallback_default: " + error;
            return false;
        }
        if (!parse_ocv_section(root, "charge", loaded_charge, error)) {
            table_load_status_ = "fallback_default: " + error;
            return false;
        }
        ocv_discharge_ = std::move(loaded_discharge);
        ocv_charge_ = std::move(loaded_charge);
        table_load_status_ = "loaded_from_file";
        return true;
    } catch (const std::exception &e) {
        table_load_status_ = std::string("fallback_default: ") + e.what();
        return false;
    }
}

double BatterySocEstimator::lookup_ocv_soc(double voltage_v, BatteryFlowDirection direction) const {
    const std::vector<OcvTablePoint> &table =
        (direction == BatteryFlowDirection::Charge) ? ocv_charge_ : ocv_discharge_;
    if (table.empty()) return 50.0;
    if (voltage_v <= table.front().voltage_v) return table.front().soc_percent;
    if (voltage_v >= table.back().voltage_v) return table.back().soc_percent;
    for (size_t i = 1; i < table.size(); ++i) {
        if (voltage_v > table[i].voltage_v) continue;
        const OcvTablePoint &lo = table[i - 1];
        const OcvTablePoint &hi = table[i];
        const double span = hi.voltage_v - lo.voltage_v;
        if (span <= 1e-9) return hi.soc_percent;
        const double t = (voltage_v - lo.voltage_v) / span;
        return lo.soc_percent + t * (hi.soc_percent - lo.soc_percent);
    }
    return table.back().soc_percent;
}

BatteryReading BatterySocEstimator::update(const BatterySample &sample) {
    if (!sample.valid) {
        invalid_streak_++;
        const bool valid = invalid_streak_ <= std::max(0, options_.invalid_read_grace);
        return {.percent = std::clamp(display_percent_, 0.0, 100.0),
                .valid = valid && has_estimate_};
    }

    invalid_streak_ = 0;
    const auto now = sample.timestamp;
    double dt_sec = 0.0;
    const bool had_previous_sample = has_last_sample_;
    if (had_previous_sample) {
        dt_sec = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_timestamp_)
                     .count();
        dt_sec = std::clamp(dt_sec, 0.0, 10.0);
    }
    last_timestamp_ = now;
    has_last_sample_ = true;

    const bool discharge_positive = options_.discharge_current_positive;
    const double signed_discharge_current_a =
        discharge_positive ? sample.current_a : -sample.current_a;
    last_direction_ = signed_discharge_current_a >= 0.0 ? BatteryFlowDirection::Discharge
                                                        : BatteryFlowDirection::Charge;

    if (!has_estimate_) {
        soc_percent_ = lookup_ocv_soc(sample.bus_voltage_v, last_direction_);
        display_percent_ = soc_percent_;
        has_estimate_ = true;
    }

    const double capacity_ah = std::max(0.1, options_.battery_capacity_ah);
    if (dt_sec > 0.0) {
        const double dt_hours = dt_sec / 3600.0;
        const double delta_soc = -(signed_discharge_current_a * dt_hours / capacity_ah) * 100.0;
        soc_percent_ = std::clamp(soc_percent_ + delta_soc, 0.0, 100.0);
    }

    const double rest_current_threshold = std::max(0.0, options_.rest_current_threshold_a);
    const double rest_stability_threshold = std::max(0.0, options_.rest_stability_delta_v);
    const bool current_is_rest_like =
        std::abs(signed_discharge_current_a) <= rest_current_threshold;
    const bool voltage_is_stable =
        !had_previous_sample ||
        std::abs(sample.bus_voltage_v - last_voltage_v_) <= rest_stability_threshold;
    if (current_is_rest_like && voltage_is_stable) {
        near_rest_sec_ += dt_sec;
    } else {
        near_rest_sec_ = 0.0;
    }
    last_voltage_v_ = sample.bus_voltage_v;

    const double rest_window_sec = std::max(0.0, options_.rest_window_sec);
    if (near_rest_sec_ >= rest_window_sec) {
        const double ocv_soc = lookup_ocv_soc(sample.bus_voltage_v, last_direction_);
        const double gain = std::clamp(options_.ocv_correction_gain, 0.0, 1.0);
        soc_percent_ = std::clamp(soc_percent_ + (ocv_soc - soc_percent_) * gain, 0.0, 100.0);
    }

    if (dt_sec > 0.0) {
        const double slew_rate = std::max(0.1, options_.display_slew_pct_per_sec);
        const double max_step = slew_rate * dt_sec;
        const double step = std::clamp(soc_percent_ - display_percent_, -max_step, max_step);
        display_percent_ = std::clamp(display_percent_ + step, 0.0, 100.0);
    } else {
        display_percent_ = soc_percent_;
    }

    if (options_.persist_interval_sec > 0.0 &&
        (last_persist_ == std::chrono::steady_clock::time_point::min() ||
         std::chrono::duration_cast<std::chrono::duration<double>>(now - last_persist_).count() >=
             options_.persist_interval_sec)) {
        if (save_state()) last_persist_ = now;
    }

    return {.percent = std::clamp(display_percent_, 0.0, 100.0), .valid = true};
}

void BatterySocEstimator::log_diagnostics(const BatterySample &sample,
                                          const BatteryReading &reading,
                                          const BatteryOptions &source_options,
                                          bool read_ok) const {
    if (!auto_battlebot::DiagnosticsLogger::is_initialized()) return;
    static std::shared_ptr<auto_battlebot::DiagnosticsModuleLogger> logger =
        auto_battlebot::DiagnosticsLogger::get_logger("battery_ina219");
    if (!logger) return;

    DiagnosticsData measurement_data;
    measurement_data["read_ok"] = read_ok ? 1 : 0;
    measurement_data["sample_valid"] = sample.valid ? 1 : 0;
    measurement_data["i2c_bus"] = source_options.i2c_bus;
    measurement_data["i2c_address"] = source_options.i2c_address;
    measurement_data["raw_bus_voltage"] = static_cast<int>(sample.raw_bus_voltage);
    measurement_data["raw_current"] = static_cast<int>(sample.raw_current);
    measurement_data["raw_power"] = static_cast<int>(sample.raw_power);
    measurement_data["bus_voltage_v"] = sample.bus_voltage_v;
    measurement_data["current_a"] = sample.current_a;
    measurement_data["power_w"] = sample.power_w;

    DiagnosticsData derived_data;
    derived_data["ui_percent"] = reading.percent;
    derived_data["ui_valid"] = reading.valid ? 1 : 0;
    derived_data["soc_percent"] = soc_percent_;
    derived_data["display_percent"] = display_percent_;
    derived_data["has_estimate"] = has_estimate_ ? 1 : 0;
    derived_data["has_last_sample"] = has_last_sample_ ? 1 : 0;
    derived_data["invalid_streak"] = invalid_streak_;
    derived_data["near_rest_sec"] = near_rest_sec_;
    derived_data["last_voltage_v"] = last_voltage_v_;
    derived_data["last_direction_discharge"] =
        (last_direction_ == BatteryFlowDirection::Discharge) ? 1 : 0;
    derived_data["table_loaded_from_file"] = table_loaded_from_file_ ? 1 : 0;
    derived_data["table_load_status"] = table_load_status_;
    derived_data["table_path"] = options_.ocv_table_file_path;

    std::vector<double> discharge_voltage;
    std::vector<double> discharge_soc;
    for (const auto &point : ocv_discharge_) {
        discharge_voltage.push_back(point.voltage_v);
        discharge_soc.push_back(point.soc_percent);
    }
    derived_data["ocv_discharge_voltage_v"] = discharge_voltage;
    derived_data["ocv_discharge_soc_percent"] = discharge_soc;

    std::vector<double> charge_voltage;
    std::vector<double> charge_soc;
    for (const auto &point : ocv_charge_) {
        charge_voltage.push_back(point.voltage_v);
        charge_soc.push_back(point.soc_percent);
    }
    derived_data["ocv_charge_voltage_v"] = charge_voltage;
    derived_data["ocv_charge_soc_percent"] = charge_soc;

    if (read_ok && reading.valid) {
        logger->debug("measurement", measurement_data);
        logger->debug("derived", derived_data);
    } else {
        logger->warning("measurement", measurement_data, "INA219 read failed");
        logger->warning("derived", derived_data,
                        "Battery estimate degraded due to sensor read failure");
    }
}

double BatterySocEstimator::debug_soc_percent() const { return soc_percent_; }

double BatterySocEstimator::debug_table_soc(size_t idx, BatteryFlowDirection direction) const {
    const std::vector<OcvTablePoint> &table =
        (direction == BatteryFlowDirection::Charge) ? ocv_charge_ : ocv_discharge_;
    if (idx >= table.size()) return -1.0;
    return table[idx].soc_percent;
}

bool BatterySocEstimator::load_state() {
    std::ifstream in(options_.state_file_path);
    if (!in.is_open()) return false;
    std::unordered_map<std::string, std::string> kv;
    std::string line;
    while (std::getline(in, line)) {
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim_copy(line.substr(0, eq));
        const std::string val = trim_copy(line.substr(eq + 1));
        if (!key.empty()) kv[key] = val;
    }
    static const std::unordered_map<std::string, bool> allowed_keys = {
        {"version", true},
        {"soc_percent", true},
        {"display_percent", true},
    };
    for (const auto &[key, _] : kv) {
        if (allowed_keys.find(key) == allowed_keys.end()) return false;
    }
    if (parse_int_or(kv, "version", -1) != kStateVersion) return false;

    soc_percent_ = std::clamp(parse_double_or(kv, "soc_percent", soc_percent_), 0.0, 100.0);
    display_percent_ = std::clamp(parse_double_or(kv, "display_percent", soc_percent_), 0.0, 100.0);
    has_estimate_ = true;
    return true;
}

bool BatterySocEstimator::save_state() const {
    const std::filesystem::path state_path(options_.state_file_path);
    if (state_path.empty()) return false;
    std::error_code ec;
    const auto parent = state_path.parent_path();
    if (!parent.empty()) std::filesystem::create_directories(parent, ec);

    std::ofstream out(state_path, std::ios::trunc);
    if (!out.is_open()) return false;

    out << "version=" << kStateVersion << "\n";
    out << "soc_percent=" << soc_percent_ << "\n";
    out << "display_percent=" << display_percent_ << "\n";
    return true;
}

}  // namespace auto_battlebot::ui_internal
