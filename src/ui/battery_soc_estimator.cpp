#include "battery_soc_estimator.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include "diagnostics_logger/diagnostics_backend_interface.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"
#include "directories.hpp"

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

}  // namespace

BatterySocEstimator::BatterySocEstimator(const BatteryOptions &options) : options_(options) {
    ocv_discharge_ = make_default_discharge_table();
    ocv_charge_ = make_default_charge_table();
    if (options_.state_file_path.empty()) options_.state_file_path = default_state_path();
    (void)load_state();
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

void BatterySocEstimator::maybe_self_tune(double voltage_v, BatteryFlowDirection direction) {
    if (!options_.self_tune_enabled) return;
    std::vector<OcvTablePoint> &table =
        (direction == BatteryFlowDirection::Charge) ? ocv_charge_ : ocv_discharge_;
    if (table.empty()) return;

    auto nearest = std::min_element(
        table.begin(), table.end(), [voltage_v](const OcvTablePoint &a, const OcvTablePoint &b) {
            return std::abs(a.voltage_v - voltage_v) < std::abs(b.voltage_v - voltage_v);
        });
    if (nearest == table.end()) return;

    const double error = soc_percent_ - nearest->soc_percent;
    const double max_delta = std::max(0.0, options_.self_tune_max_delta_pct);
    const double gain = std::max(0.0, options_.self_tune_learning_rate);
    const double delta = std::clamp(error * gain, -max_delta, max_delta);

    if (nearest->learned_samples >= options_.self_tune_min_samples || std::abs(error) < 25.0) {
        nearest->soc_percent = std::clamp(nearest->soc_percent + delta, 0.0, 100.0);
    }
    nearest->learned_samples += 1.0;
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
        maybe_self_tune(sample.bus_voltage_v, last_direction_);
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

    std::vector<double> discharge_voltage;
    std::vector<double> discharge_soc;
    std::vector<double> discharge_samples;
    for (const auto &point : ocv_discharge_) {
        discharge_voltage.push_back(point.voltage_v);
        discharge_soc.push_back(point.soc_percent);
        discharge_samples.push_back(point.learned_samples);
    }
    derived_data["ocv_discharge_voltage_v"] = discharge_voltage;
    derived_data["ocv_discharge_soc_percent"] = discharge_soc;
    derived_data["ocv_discharge_learned_samples"] = discharge_samples;

    std::vector<double> charge_voltage;
    std::vector<double> charge_soc;
    std::vector<double> charge_samples;
    for (const auto &point : ocv_charge_) {
        charge_voltage.push_back(point.voltage_v);
        charge_soc.push_back(point.soc_percent);
        charge_samples.push_back(point.learned_samples);
    }
    derived_data["ocv_charge_voltage_v"] = charge_voltage;
    derived_data["ocv_charge_soc_percent"] = charge_soc;
    derived_data["ocv_charge_learned_samples"] = charge_samples;

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
    if (parse_int_or(kv, "version", -1) != kStateVersion) return false;

    soc_percent_ = std::clamp(parse_double_or(kv, "soc_percent", soc_percent_), 0.0, 100.0);
    display_percent_ = std::clamp(parse_double_or(kv, "display_percent", soc_percent_), 0.0, 100.0);
    has_estimate_ = true;

    for (size_t i = 0; i < ocv_discharge_.size(); ++i) {
        const std::string idx = std::to_string(i);
        ocv_discharge_[i].soc_percent = std::clamp(
            parse_double_or(kv, "discharge_soc_" + idx, ocv_discharge_[i].soc_percent), 0.0, 100.0);
        ocv_discharge_[i].learned_samples =
            std::max(0.0, parse_double_or(kv, "discharge_samples_" + idx, 0.0));
    }
    for (size_t i = 0; i < ocv_charge_.size(); ++i) {
        const std::string idx = std::to_string(i);
        ocv_charge_[i].soc_percent = std::clamp(
            parse_double_or(kv, "charge_soc_" + idx, ocv_charge_[i].soc_percent), 0.0, 100.0);
        ocv_charge_[i].learned_samples =
            std::max(0.0, parse_double_or(kv, "charge_samples_" + idx, 0.0));
    }
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
    for (size_t i = 0; i < ocv_discharge_.size(); ++i) {
        out << "discharge_soc_" << i << "=" << ocv_discharge_[i].soc_percent << "\n";
        out << "discharge_samples_" << i << "=" << ocv_discharge_[i].learned_samples << "\n";
    }
    for (size_t i = 0; i < ocv_charge_.size(); ++i) {
        out << "charge_soc_" << i << "=" << ocv_charge_[i].soc_percent << "\n";
        out << "charge_samples_" << i << "=" << ocv_charge_[i].learned_samples << "\n";
    }
    return true;
}

}  // namespace auto_battlebot::ui_internal
