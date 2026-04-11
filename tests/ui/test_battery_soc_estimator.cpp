#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string>

#include "battery_soc_estimator.hpp"

namespace auto_battlebot::ui_internal {
namespace {

BatterySample make_sample(std::chrono::steady_clock::time_point ts, double voltage_v, double current_a,
                         bool valid = true) {
    BatterySample sample;
    sample.timestamp = ts;
    sample.bus_voltage_v = voltage_v;
    sample.current_a = current_a;
    sample.valid = valid;
    return sample;
}

std::string make_temp_state_file(const std::string &name) {
    const auto path = std::filesystem::temp_directory_path() / ("auto_battlebot_" + name + ".state");
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return path.string();
}

TEST(BatterySocEstimatorTest, CoulombCountingMonotonicDischarge) {
    BatteryOptions options;
    options.battery_capacity_ah = 1.0;
    options.ocv_correction_gain = 0.0;
    options.self_tune_enabled = false;
    options.state_file_path = make_temp_state_file("soc_discharge");

    BatterySocEstimator estimator(options);
    const auto t0 = std::chrono::steady_clock::time_point{};
    auto reading = estimator.update(make_sample(t0, 12.4, 0.0));
    ASSERT_TRUE(reading.valid);
    const double start_soc = reading.percent;

    for (int i = 1; i <= 120; ++i) {
        const auto ts = t0 + std::chrono::seconds(i * 10);
        reading = estimator.update(make_sample(ts, 12.0, 1.0));
    }

    EXPECT_LT(reading.percent, start_soc - 25.0);
}

TEST(BatterySocEstimatorTest, OcvCorrectionConvergesAfterRestWindow) {
    BatteryOptions options;
    options.battery_capacity_ah = 1.0;
    options.ocv_correction_gain = 1.0;
    options.rest_window_sec = 5.0;
    options.rest_current_threshold_a = 0.2;
    options.rest_stability_delta_v = 1.0;
    options.self_tune_enabled = false;
    options.state_file_path = make_temp_state_file("soc_ocv_rest");

    BatterySocEstimator estimator(options);
    const auto t0 = std::chrono::steady_clock::time_point{};
    auto reading = estimator.update(make_sample(t0, 12.6, 0.0));
    ASSERT_TRUE(reading.valid);
    EXPECT_GT(reading.percent, 90.0);

    for (int i = 1; i <= 20; ++i) {
        const auto ts = t0 + std::chrono::seconds(i * 10);
        reading = estimator.update(make_sample(ts, 12.6, 5.0));
    }
    EXPECT_LT(reading.percent, 80.0);

    reading = estimator.update(make_sample(t0 + std::chrono::seconds(203), 12.6, 0.0));
    EXPECT_LT(reading.percent, 80.0);

    reading = estimator.update(make_sample(t0 + std::chrono::seconds(206), 12.6, 0.0));
    EXPECT_GT(reading.percent, 90.0);
}

TEST(BatterySocEstimatorTest, SelfTuneDoesNotUpdateDuringDynamicLoad) {
    BatteryOptions options;
    options.self_tune_enabled = true;
    options.self_tune_learning_rate = 0.5;
    options.rest_window_sec = 5.0;
    options.rest_current_threshold_a = 0.05;
    options.ocv_correction_gain = 0.0;
    options.state_file_path = make_temp_state_file("soc_no_dynamic_tune");

    BatterySocEstimator estimator(options);
    const auto t0 = std::chrono::steady_clock::time_point{};
    estimator.update(make_sample(t0, 11.2, 0.8));
    const double before = estimator.debug_table_soc(3, BatteryFlowDirection::Discharge);

    for (int i = 1; i <= 10; ++i) {
        const auto ts = t0 + std::chrono::seconds(i);
        estimator.update(make_sample(ts, 11.2 + 0.02 * i, 0.8));
    }

    const double after = estimator.debug_table_soc(3, BatteryFlowDirection::Discharge);
    EXPECT_DOUBLE_EQ(before, after);
}

TEST(BatterySocEstimatorTest, CorruptVersionFallsBackToDefaultTable) {
    const std::string state_file = make_temp_state_file("soc_version_fallback");

    BatteryOptions options;
    options.self_tune_enabled = false;
    options.persist_interval_sec = 1.0;
    options.state_file_path = state_file;

    {
        BatterySocEstimator estimator(options);
        estimator.update(make_sample(std::chrono::steady_clock::time_point{}, 9.0, 0.0));
        ASSERT_TRUE(std::filesystem::exists(state_file));
    }

    {
        std::ofstream out(state_file, std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "version=999\n";
        out << "soc_percent=5\n";
        out << "display_percent=5\n";
    }

    BatterySocEstimator estimator(options);
    const auto reading = estimator.update(make_sample(std::chrono::steady_clock::time_point{}, 12.6, 0.0));
    EXPECT_GT(reading.percent, 90.0);
}

}  // namespace
}  // namespace auto_battlebot::ui_internal
