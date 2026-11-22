#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <string>
#include <vector>

#include "config/config_parser.hpp"
#include "health/performance_monitor_config.hpp"

namespace auto_battlebot {
namespace {

PerformanceMonitorConfiguration load(const std::string &text, std::vector<std::string> &sections) {
    return load_performance_monitor_from_toml(toml::parse(text), sections);
}

TEST(PerformanceMonitorConfigTest, MissingSectionKeepsTheDefaults) {
    std::vector<std::string> sections;
    const auto config = load("", sections);
    EXPECT_TRUE(config.enable);
    EXPECT_DOUBLE_EQ(config.window_s, 5.0);
    EXPECT_DOUBLE_EQ(config.slew_difference, 0.2);
    EXPECT_DOUBLE_EQ(config.tracking_coverage_min, 0.85);
    EXPECT_TRUE(sections.empty());
}

TEST(PerformanceMonitorConfigTest, ProfileOverridesAThreshold) {
    std::vector<std::string> sections;
    const auto config = load(
        "[performance_monitor]\ncrsf_telemetry_max_age_s = 4.0\nyaw_min_pairs = 40\n", sections);
    EXPECT_DOUBLE_EQ(config.crsf_telemetry_max_age_s, 4.0);
    EXPECT_EQ(config.yaw_min_pairs, 40);
    EXPECT_EQ(sections, std::vector<std::string>{"performance_monitor"});
}

TEST(PerformanceMonitorConfigTest, TypoedKeyIsRefused) {
    std::vector<std::string> sections;
    EXPECT_THROW(load("[performance_monitor]\nslew_diference = 0.3\n", sections),
                 ConfigValidationError);
}

TEST(PerformanceMonitorConfigTest, NonPositiveWindowIsRefused) {
    std::vector<std::string> sections;
    EXPECT_THROW(load("[performance_monitor]\nwindow_s = 0.0\n", sections), ConfigValidationError);
}

}  // namespace
}  // namespace auto_battlebot
