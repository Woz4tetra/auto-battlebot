#include <gtest/gtest.h>
#include <toml++/toml.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>

#include "battery/battery_monitor.hpp"
#include "battery/config.hpp"
#include "config/config_parser.hpp"

namespace auto_battlebot {
namespace {

using namespace std::chrono_literals;

constexpr auto kPeriod = 5ms;

BatteryOptions temp_options(const std::string &name) {
    BatteryOptions options;
    const auto path = std::filesystem::temp_directory_path() /
                      ("battery_monitor_test_" + name + "_" + std::to_string(::getpid()));
    std::error_code ec;
    std::filesystem::remove(path, ec);
    options.state_file_path = path.string();
    return options;
}

BatteryMonitor::SampleReader fixed_reader(double voltage_v, double current_a,
                                          std::atomic<int> *reads = nullptr) {
    return [=](BatterySample &sample) {
        if (reads) reads->fetch_add(1);
        sample.timestamp = std::chrono::steady_clock::now();
        sample.bus_voltage_v = voltage_v;
        sample.current_a = current_a;
        sample.valid = true;
        return true;
    };
}

/** Polls until `done` holds or a second passes. */
bool wait_for(const std::function<bool()> &done) {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (std::chrono::steady_clock::now() < deadline) {
        if (done()) return true;
        std::this_thread::sleep_for(1ms);
    }
    return done();
}

TEST(BatteryMonitorTest, PublishesTheFirstReading) {
    BatteryMonitor monitor(temp_options("first"), fixed_reader(12.2, 0.5), nullptr, kPeriod);
    ASSERT_TRUE(wait_for([&] { return monitor.latest().has_value(); }));
    const auto message = *monitor.latest();
    EXPECT_TRUE(message.valid);
    EXPECT_GE(message.percent, 0.0);
    EXPECT_LE(message.percent, 100.0);
    ASSERT_TRUE(message.bus_voltage_v.has_value());
    EXPECT_DOUBLE_EQ(*message.bus_voltage_v, 12.2);
    ASSERT_TRUE(message.current_a.has_value());
    EXPECT_DOUBLE_EQ(*message.current_a, 0.5);
    EXPECT_FALSE(message.charging);
}

TEST(BatteryMonitorTest, CurrentIntoTheBatteryIsCharging) {
    BatteryMonitor monitor(temp_options("charging"), fixed_reader(12.2, -1.0), nullptr, kPeriod);
    ASSERT_TRUE(wait_for([&] { return monitor.latest().has_value(); }));
    EXPECT_TRUE(monitor.latest()->charging);
}

TEST(BatteryMonitorTest, ChargingFollowsTheConfiguredCurrentSign) {
    BatteryOptions options = temp_options("sign");
    options.discharge_current_positive = false;
    BatteryMonitor monitor(options, fixed_reader(12.2, 1.0), nullptr, kPeriod);
    ASSERT_TRUE(wait_for([&] { return monitor.latest().has_value(); }));
    EXPECT_TRUE(monitor.latest()->charging);
}

TEST(BatteryMonitorTest, FailedReadsGoInvalidAfterTheGrace) {
    BatteryOptions options = temp_options("grace");
    options.invalid_read_grace = 2;
    std::atomic<int> reads{0};
    auto reader = [&reads](BatterySample &sample) {
        // One good read, then the bus stops answering.
        if (reads.fetch_add(1) > 0) return false;
        sample.timestamp = std::chrono::steady_clock::now();
        sample.bus_voltage_v = 12.0;
        sample.valid = true;
        return true;
    };
    BatteryMonitor monitor(options, reader, nullptr, kPeriod);

    ASSERT_TRUE(wait_for([&] { return reads.load() >= 2; }));
    // Inside the grace: still valid, but the failed read carries no measurements.
    ASSERT_TRUE(wait_for([&] {
        const auto m = monitor.latest();
        return m && !m->bus_voltage_v.has_value();
    }));

    ASSERT_TRUE(wait_for([&] { return reads.load() >= 5; }));
    ASSERT_TRUE(wait_for([&] { return !monitor.latest()->valid; }));
    EXPECT_FALSE(monitor.latest()->bus_voltage_v.has_value());
}

TEST(BatteryMonitorTest, StopsPromptlyWithALongPeriod) {
    std::atomic<int> reads{0};
    const auto start = std::chrono::steady_clock::now();
    {
        BatteryMonitor monitor(temp_options("stop"), fixed_reader(12.2, 0.5, &reads), nullptr,
                               std::chrono::milliseconds(60'000));
        ASSERT_TRUE(wait_for([&] { return reads.load() == 1; }));
    }
    EXPECT_LT(std::chrono::steady_clock::now() - start, 1s);
}

TEST(BatteryMonitorTest, NoneMakesNoMonitor) {
    BatteryConfiguration config;
    config.source = BatterySource::NONE;
    EXPECT_EQ(make_battery_monitor(config, nullptr), nullptr);
}

BatteryConfiguration load(const char *text) {
    toml::table table = toml::parse(text);
    std::vector<std::string> sections;
    return load_battery_from_toml(table, sections);
}

TEST(BatteryConfigTest, MissingSectionIsNone) { EXPECT_EQ(load("").source, BatterySource::NONE); }

TEST(BatteryConfigTest, ReadsSourceAndEstimatorFields) {
    const auto config = load(R"(
[battery]
source = "WAVESHARE_UPS"
i2c_bus = 1
battery_capacity_ah = 2.5
discharge_current_positive = false
)");
    EXPECT_EQ(config.source, BatterySource::WAVESHARE_UPS);
    EXPECT_EQ(config.options.i2c_bus, 1);
    EXPECT_DOUBLE_EQ(config.options.battery_capacity_ah, 2.5);
    EXPECT_FALSE(config.options.discharge_current_positive);
    EXPECT_EQ(config.options.i2c_address, BatteryOptions{}.i2c_address);
}

TEST(BatteryConfigTest, RejectsAnUnknownSource) {
    EXPECT_THROW(load("[battery]\nsource = \"Waveshare UPS\"\n"), std::invalid_argument);
}

TEST(BatteryConfigTest, RejectsAnUnknownField) {
    EXPECT_THROW(load("[battery]\nsource = \"NONE\"\nbattery_source = \"x\"\n"),
                 ConfigValidationError);
}

}  // namespace
}  // namespace auto_battlebot
