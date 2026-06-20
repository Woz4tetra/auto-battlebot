#include <gtest/gtest.h>

#include <chrono>

#include "rgbd_camera/grab_health_monitor.hpp"

namespace auto_battlebot {

using namespace std::chrono_literals;

namespace {
constexpr auto kWindow = 10s;
constexpr double kShutdownRatio = 0.70;
const std::chrono::steady_clock::time_point kBase{};
}  // namespace

TEST(GrabHealthMonitorTest, FreshMonitorIsHealthy) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    EXPECT_FALSE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 0.0);
    EXPECT_FALSE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, SingleSampleDoesNotFillWindow) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
    // One sample of age 0 cannot span the window, so no shutdown despite a 100% error ratio.
    EXPECT_FALSE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 1.0);
    EXPECT_FALSE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, FullWindowOfErrorsTriggersShutdown) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
    // Oldest sample is now exactly one window old (age == window is retained, not trimmed).
    monitor.record(true, kBase + kWindow);
    EXPECT_TRUE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 1.0);
    EXPECT_TRUE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, FullWindowBelowThresholdDoesNotShutdown) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(false, kBase);
    monitor.record(true, kBase + kWindow);
    EXPECT_TRUE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 0.5);  // 0.5 < 0.70
    EXPECT_FALSE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, AgedOutSamplesAreDropped) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
    // Age of the first sample is 11s > window, so it is trimmed along with its error count.
    monitor.record(false, kBase + 11s);
    EXPECT_FALSE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 0.0);
    EXPECT_FALSE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, ResetClearsState) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
    monitor.record(true, kBase + kWindow);
    ASSERT_TRUE(monitor.should_shutdown());

    monitor.reset();
    EXPECT_FALSE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 0.0);
    EXPECT_FALSE(monitor.should_shutdown());
}

}  // namespace auto_battlebot
