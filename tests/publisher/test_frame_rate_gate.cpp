#include <gtest/gtest.h>

#include <chrono>

#include "publisher/frame_rate_gate.hpp"

namespace auto_battlebot {
namespace {

using Clock = FrameRateGate::Clock;
using std::chrono::duration;
using std::chrono::duration_cast;

int admitted(FrameRateGate &gate, double source_hz, double seconds, Clock::time_point start = {}) {
    int count = 0;
    const int frames = static_cast<int>(source_hz * seconds);
    for (int i = 0; i < frames; ++i) {
        const auto t = start + duration_cast<Clock::duration>(duration<double>(i / source_hz));
        if (gate.admit(t)) ++count;
    }
    return count;
}

TEST(FrameRateGateTest, SourceJustFasterThanTheCapGetsTheCap) {
    constexpr double kHandheldCameraHz = 31.0;
    constexpr double kPreviewCapHz = 30.0;
    FrameRateGate gate(kPreviewCapHz);
    const int count = admitted(gate, kHandheldCameraHz, 10.0);
    EXPECT_GE(count, 298);
    EXPECT_LE(count, 301);
}

TEST(FrameRateGateTest, FastSourceIsCapped) {
    FrameRateGate gate(30.0);
    const int count = admitted(gate, 60.0, 10.0);
    EXPECT_GE(count, 299);
    EXPECT_LE(count, 302);
}

TEST(FrameRateGateTest, SlowSourcePassesEveryFrame) {
    FrameRateGate gate(30.0);
    EXPECT_EQ(admitted(gate, 20.0, 10.0), 200);
}

TEST(FrameRateGateTest, JitteredSourceJustFasterThanTheCapGetsTheCap) {
    constexpr auto kShortGap = std::chrono::milliseconds(27);
    constexpr auto kLongGap = std::chrono::milliseconds(37);
    constexpr int kFrames = 312;
    constexpr int kSteady30FpsFrames = 299;
    FrameRateGate gate(30.0);
    int count = 0;
    Clock::time_point t{};
    for (int i = 0; i < kFrames; ++i) {
        if (gate.admit(t)) ++count;
        t += i % 2 == 0 ? kShortGap : kLongGap;
    }
    EXPECT_GE(count, kSteady30FpsFrames - 3);
    EXPECT_LE(count, kSteady30FpsFrames + 2);
}

TEST(FrameRateGateTest, NoBurstAfterAStall) {
    FrameRateGate gate(30.0);
    Clock::time_point t{};
    EXPECT_TRUE(gate.admit(t));
    t += std::chrono::seconds(2);
    EXPECT_TRUE(gate.admit(t));
    constexpr auto kInsideRestartedPeriod = std::chrono::milliseconds(10);
    EXPECT_FALSE(gate.admit(t + kInsideRestartedPeriod)) << "cadence restarts at the stall's end";
    EXPECT_TRUE(gate.admit(t + std::chrono::milliseconds(34)));
}

TEST(FrameRateGateTest, ZeroRateLetsEverythingThrough) {
    FrameRateGate gate(0.0);
    EXPECT_EQ(admitted(gate, 100.0, 1.0), 100);
}

}  // namespace
}  // namespace auto_battlebot
