#include <gtest/gtest.h>

#include <chrono>
#include <thread>

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

TEST(GrabHealthMonitorTest, SingleSampleCannotSpanTheWindowSoNeverShutsDown) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
    EXPECT_FALSE(monitor.window_full());
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 1.0);
    EXPECT_FALSE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, SampleAgedExactlyOneWindowIsRetainedAndTriggersShutdown) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
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
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 0.5);
    EXPECT_LT(monitor.error_ratio(), kShutdownRatio);
    EXPECT_FALSE(monitor.should_shutdown());
}

TEST(GrabHealthMonitorTest, AgedOutSamplesAreDropped) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(true, kBase);
    monitor.record(false, kBase + kWindow + 1s);
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

TEST(GrabHealthMonitorTest, RecordingFromASecondThreadAssertsInDebug) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(false, kBase);
    EXPECT_DEBUG_DEATH(
        {
            std::thread other([&] { monitor.record(false, kBase + 1s); });
            other.join();
        },
        "");
}

TEST(GrabHealthMonitorTest, ResetLetsAnotherThreadTakeOverRecording) {
    GrabHealthMonitor monitor(kWindow, kShutdownRatio);
    monitor.record(false, kBase);
    monitor.reset();
    std::thread other([&] { monitor.record(false, kBase + 1s); });
    other.join();
    EXPECT_DOUBLE_EQ(monitor.error_ratio(), 0.0);
}

}  // namespace auto_battlebot
