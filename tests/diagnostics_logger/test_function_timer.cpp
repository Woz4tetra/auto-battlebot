#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "diagnostics_logger/function_timer.hpp"

namespace auto_battlebot {

TEST(FunctionTimerTest, StopThenDestroyLogsOnce) {
    auto logger = std::make_shared<DiagnosticsModuleLogger>("timer_test");
    {
        FunctionTimer timer(logger, "work");
        timer.stop();
        logger->clear();
    }
    const auto snapshots = logger->get_snapshots();
    ASSERT_EQ(snapshots.size(), 1u);
    EXPECT_TRUE(snapshots[0].is_repeat_of_earlier_message);
}

TEST(FunctionTimerTest, NoThresholdNeverWarns) {
    auto logger = std::make_shared<DiagnosticsModuleLogger>("timer_test");
    { FunctionTimer timer(logger, "work"); }
    EXPECT_EQ(logger->get_snapshots().at(0).level, DiagnosticLevel::OK);
}

TEST(FunctionTimerTest, ThresholdBelowElapsedWarns) {
    auto logger = std::make_shared<DiagnosticsModuleLogger>("timer_test");
    {
        FunctionTimer timer(logger, "work", 1.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_EQ(logger->get_snapshots().at(0).level, DiagnosticLevel::WARN);
}

}  // namespace auto_battlebot
