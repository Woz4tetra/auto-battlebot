#include <drive_gate.h>
#include <gtest/gtest.h>
#include <motor_guard.h>

using namespace drive_gate;

TEST(LoopMode, RadioLossBeatsEverything) {
    EXPECT_EQ(loop_mode(false, true, false), LoopMode::RADIO_LOST);
    EXPECT_EQ(loop_mode(false, false, true), LoopMode::RADIO_LOST);
}

TEST(LoopMode, DisarmedOrTuningStopsTheEscs) {
    EXPECT_EQ(loop_mode(true, false, false), LoopMode::STOPPED);
    EXPECT_EQ(loop_mode(true, true, true), LoopMode::STOPPED);
    EXPECT_EQ(loop_mode(true, true, false), LoopMode::DRIVE);
}

TEST(LoopMode, TripClearsOnlyWhileDisarmedWithRadio) {
    EXPECT_TRUE(clear_guard_trip(LoopMode::STOPPED, false));
    EXPECT_FALSE(clear_guard_trip(LoopMode::STOPPED, true));
    EXPECT_FALSE(clear_guard_trip(LoopMode::RADIO_LOST, false));
    EXPECT_FALSE(clear_guard_trip(LoopMode::DRIVE, false));
}

TEST(LoopGates, ArmedNeedsTheRadio) {
    EXPECT_TRUE(armed_now(true, true));
    EXPECT_FALSE(armed_now(false, true));
    EXPECT_FALSE(armed_now(true, false));
}

TEST(LoopGates, CrashDumpEraseRefusedWhileArmedAllowedWithoutRadio) {
    EXPECT_FALSE(erase_dump_allowed(true, true));
    EXPECT_TRUE(erase_dump_allowed(true, false));
    EXPECT_TRUE(erase_dump_allowed(false, true));
}

TEST(LoopGates, OtaHandlerSkippedWhileTuningOwnsTheEscs) {
    EXPECT_TRUE(run_ota_handler(false));
    EXPECT_FALSE(run_ota_handler(true));
}

TEST(LoopGates, EspNowSeesTuningAsArmed) {
    EXPECT_TRUE(espnow_arm_input(true, true, false));
    EXPECT_TRUE(espnow_arm_input(false, false, true));
    EXPECT_FALSE(espnow_arm_input(true, false, false));
    EXPECT_FALSE(espnow_arm_input(false, true, false));
}

TEST(Substitution, NeedsEveryConditionIncludingTheAutonomySwitch) {
    EXPECT_TRUE(substitution_active(true, true, true, true));
    EXPECT_FALSE(substitution_active(false, true, true, true));
    EXPECT_FALSE(substitution_active(true, false, true, true));
    EXPECT_FALSE(substitution_active(true, true, false, true));
    EXPECT_FALSE(substitution_active(true, true, true, false));
}

TEST(Orientation, UpAndMiddleIgnoreTheSensor) {
    for (bool sensed : {false, true}) {
        EXPECT_TRUE(resolve_orientation(FlipSwitch::UP, sensed).upside_down);
        EXPECT_FALSE(resolve_orientation(FlipSwitch::UP, sensed).auto_steer);
        EXPECT_FALSE(resolve_orientation(FlipSwitch::MIDDLE, sensed).upside_down);
        EXPECT_FALSE(resolve_orientation(FlipSwitch::MIDDLE, sensed).auto_steer);
    }
}

TEST(Orientation, DownFollowsTheSensorAndEnablesAutoSteer) {
    EXPECT_TRUE(resolve_orientation(FlipSwitch::DOWN, true).upside_down);
    EXPECT_FALSE(resolve_orientation(FlipSwitch::DOWN, false).upside_down);
    EXPECT_TRUE(resolve_orientation(FlipSwitch::DOWN, false).auto_steer);
}

TEST(Orientation, UnknownSwitchValueIsRightSideUpManual) {
    const Orientation facing = resolve_orientation(static_cast<FlipSwitch>(7), true);
    EXPECT_FALSE(facing.upside_down);
    EXPECT_FALSE(facing.auto_steer);
}

TEST(ThrottleSign, StickBackIsPositiveAndDrivesBothSidesNegativeRightSideUp) {
    const Pair pair = mix_pair(flipped_throttle(40.0f, false), 0.0f);
    EXPECT_FLOAT_EQ(pair.left, -40.0f);
    EXPECT_FLOAT_EQ(pair.right, -40.0f);
}

TEST(ThrottleSign, UpsideDownReversesBothSides) {
    const Pair pair = mix_pair(flipped_throttle(40.0f, true), 0.0f);
    EXPECT_FLOAT_EQ(pair.left, 40.0f);
    EXPECT_FLOAT_EQ(pair.right, 40.0f);
}

TEST(ThrottleSign, TailFirstThrottleIsPositiveForStickBackInBothOrientations) {
    EXPECT_FLOAT_EQ(tail_first_throttle(flipped_throttle(40.0f, false), false), 40.0f);
    EXPECT_FLOAT_EQ(tail_first_throttle(flipped_throttle(40.0f, true), true), 40.0f);
}

TEST(ThrottleSign, PullingTheStickBackReadsPositiveAndDrivesTailFirstInEitherOrientation) {
    const float a_percent = throttle_from_channel_percent(-50.0f);
    EXPECT_FLOAT_EQ(a_percent, 50.0f);
    const Pair upright = mix_pair(flipped_throttle(a_percent, false), 0.0f);
    EXPECT_LT(upright.left, 0.0f);
    EXPECT_LT(upright.right, 0.0f);
    const Pair inverted = mix_pair(flipped_throttle(a_percent, true), 0.0f);
    EXPECT_GT(inverted.left, 0.0f);
    EXPECT_GT(inverted.right, 0.0f);
}

TEST(Mix, SteerDifferentialSplitsLeftPositiveRightNegative) {
    const Pair pair = mix_pair(0.0f, 10.0f);
    EXPECT_FLOAT_EQ(pair.left, 10.0f);
    EXPECT_FLOAT_EQ(pair.right, -10.0f);
}

TEST(Mix, SaturationScalesBothSidesKeepingTheRatio) {
    const Pair pair = mix_pair(80.0f, 40.0f);
    EXPECT_FLOAT_EQ(pair.left, -40.0f / 120.0f * 100.0f);
    EXPECT_FLOAT_EQ(pair.right, -120.0f / 120.0f * 100.0f);
}

struct RecordingGuard {
    mutable float seen_left = 0;
    mutable float seen_right = 0;
    void apply(float &left, float &right) const {
        seen_left = left;
        seen_right = right;
        left = 0.5f * left;
        right = 0.5f * right;
    }
};

TEST(GuardAfterMixer, GuardSeesMixedCommandsAndItsOutputIsReturned) {
    RecordingGuard guard;
    const Pair pair = guarded_pair(20.0f, 10.0f, guard);
    EXPECT_FLOAT_EQ(guard.seen_left, -10.0f);
    EXPECT_FLOAT_EQ(guard.seen_right, -30.0f);
    EXPECT_FLOAT_EQ(pair.left, -5.0f);
    EXPECT_FLOAT_EQ(pair.right, -15.0f);
}

TEST(GuardAfterMixer, TrippedMotorGuardCutsTheMixedDrive) {
    motor_guard::MotorGuard guard{motor_guard::Config()};
    Pair untouched = mix_pair(60.0f, 0.0f);
    Pair guarded = guarded_pair(60.0f, 0.0f, guard);
    EXPECT_FLOAT_EQ(guarded.left, untouched.left * guard.scale());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
