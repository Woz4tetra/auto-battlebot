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

std::string make_temp_table_file(const std::string &name) {
    const auto path =
        std::filesystem::temp_directory_path() / ("auto_battlebot_" + name + "_ocv.toml");
    std::error_code ec;
    std::filesystem::remove(path, ec);
    return path.string();
}

void write_ocv_table_file(const std::string &path, double discharge_mid_soc = 45.0,
                          double charge_mid_soc = 50.0) {
    std::ofstream out(path, std::ios::trunc);
    ASSERT_TRUE(out.is_open());
    out << "[discharge]\n";
    out << "voltage_v = [9.0, 10.8, 12.6]\n";
    out << "soc_percent = [0.0, " << discharge_mid_soc << ", 100.0]\n";
    out << "[charge]\n";
    out << "voltage_v = [9.2, 10.9, 12.7]\n";
    out << "soc_percent = [0.0, " << charge_mid_soc << ", 100.0]\n";
}

TEST(BatterySocEstimatorTest, CoulombCountingMonotonicDischarge) {
    BatteryOptions options;
    options.battery_capacity_ah = 1.0;
    options.ocv_correction_gain = 0.0;
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

TEST(BatterySocEstimatorTest, StaticTableDoesNotMutateDuringRuntime) {
    BatteryOptions options;
    options.rest_window_sec = 5.0;
    options.rest_current_threshold_a = 3.0;
    options.ocv_correction_gain = 1.0;
    options.state_file_path = make_temp_state_file("soc_static_runtime");
    options.ocv_table_file_path = make_temp_table_file("soc_static_runtime");
    write_ocv_table_file(options.ocv_table_file_path, 47.0, 52.0);

    BatterySocEstimator estimator(options);
    const auto t0 = std::chrono::steady_clock::time_point{};
    estimator.update(make_sample(t0, 10.8, 0.0));
    const double before = estimator.debug_table_soc(1, BatteryFlowDirection::Discharge);

    for (int i = 1; i <= 10; ++i) {
        const auto ts = t0 + std::chrono::seconds(i);
        estimator.update(make_sample(ts, 10.8, 0.0));
    }

    const double after = estimator.debug_table_soc(1, BatteryFlowDirection::Discharge);
    EXPECT_DOUBLE_EQ(before, after);
}

TEST(BatterySocEstimatorTest, CorruptVersionFallsBackToDefaultTable) {
    const std::string state_file = make_temp_state_file("soc_version_fallback");

    BatteryOptions options;
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

TEST(BatterySocEstimatorTest, LoadsStaticTableFromFile) {
    BatteryOptions options;
    options.state_file_path = make_temp_state_file("soc_table_file");
    options.ocv_table_file_path = make_temp_table_file("soc_table_file");
    write_ocv_table_file(options.ocv_table_file_path, 42.0, 58.0);

    BatterySocEstimator estimator(options);
    EXPECT_DOUBLE_EQ(estimator.debug_table_soc(1, BatteryFlowDirection::Discharge), 42.0);
    EXPECT_DOUBLE_EQ(estimator.debug_table_soc(1, BatteryFlowDirection::Charge), 58.0);
}

TEST(BatterySocEstimatorTest, InvalidTableFallsBackToBuiltInDefaults) {
    BatteryOptions options;
    options.state_file_path = make_temp_state_file("soc_bad_table");
    options.ocv_table_file_path = make_temp_table_file("soc_bad_table");
    {
        std::ofstream out(options.ocv_table_file_path, std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "[discharge]\nvoltage_v = [9.0, 12.0]\nsoc_percent = [0.0]\n";
    }

    BatterySocEstimator estimator(options);
    EXPECT_DOUBLE_EQ(estimator.debug_table_soc(1, BatteryFlowDirection::Discharge), 15.0);
}

TEST(BatterySocEstimatorTest, LegacyStateKeysInvalidateStateLoad) {
    BatteryOptions options;
    options.state_file_path = make_temp_state_file("soc_legacy_state");
    options.ocv_table_file_path = make_temp_table_file("soc_legacy_state");
    write_ocv_table_file(options.ocv_table_file_path, 40.0, 60.0);

    {
        std::ofstream out(options.state_file_path, std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "version=1\n";
        out << "soc_percent=33\n";
        out << "display_percent=33\n";
        out << "discharge_soc_1=5\n";
        out << "charge_soc_1=7\n";
    }

    BatterySocEstimator estimator(options);
    const auto reading =
        estimator.update(make_sample(std::chrono::steady_clock::time_point{}, 12.6, 0.0));
    EXPECT_GT(reading.percent, 90.0);
}

}  // namespace
}  // namespace auto_battlebot::ui_internal
