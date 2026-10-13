#include <gtest/gtest.h>

#include <chrono>

#include "publisher/frame_rate_gate.hpp"

namespace auto_battlebot {
namespace {

using Clock = FrameRateGate::Clock;
using std::chrono::duration;
using std::chrono::duration_cast;

/** Frames let through out of `seconds` of a source at `source_hz`. */
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
    // The handheld: a 31 Hz camera under the 30 Hz preview cap. The old rule sent about 20 fps.
    FrameRateGate gate(30.0);
    const int count = admitted(gate, 31.0, 10.0);
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
    // 32 ms on average, alternating 27 and 37 ms.
    FrameRateGate gate(30.0);
    int count = 0;
    Clock::time_point t{};
    for (int i = 0; i < 312; ++i) {
        if (gate.admit(t)) ++count;
        t += std::chrono::milliseconds(i % 2 == 0 ? 27 : 37);
    }
    // 312 frames over 9.98 s: 299 at a steady 30 fps.
    EXPECT_GE(count, 296);
    EXPECT_LE(count, 301);
}

TEST(FrameRateGateTest, NoBurstAfterAStall) {
    FrameRateGate gate(30.0);
    Clock::time_point t{};
    EXPECT_TRUE(gate.admit(t));
    t += std::chrono::seconds(2);
    EXPECT_TRUE(gate.admit(t));
    // The cadence restarted at the stall's end, so a frame 10 ms later waits.
    EXPECT_FALSE(gate.admit(t + std::chrono::milliseconds(10)));
    EXPECT_TRUE(gate.admit(t + std::chrono::milliseconds(34)));
}

TEST(FrameRateGateTest, ZeroRateLetsEverythingThrough) {
    FrameRateGate gate(0.0);
    EXPECT_EQ(admitted(gate, 100.0, 1.0), 100);
}

}  // namespace
}  // namespace auto_battlebot
