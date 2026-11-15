#include <gtest/gtest.h>
#include <yaw_control.h>

using yaw_control::Config;
using yaw_control::YawController;

namespace {
const float kTol = 1e-3f;
const float kDt = 0.01f;
const float kNoseFirstThrottle = -50.0f;

Config p_only() {
    Config config;
    config.ki = 0.0f;
    return config;
}
}  // namespace

TEST(WrapDegrees, WrapsToHalfOpenRange) {
    EXPECT_NEAR(yaw_control::wrap_degrees(350.0f), -10.0f, kTol);
    EXPECT_NEAR(yaw_control::wrap_degrees(-350.0f), 10.0f, kTol);
    EXPECT_NEAR(yaw_control::wrap_degrees(180.0f), -180.0f, kTol);
    EXPECT_NEAR(yaw_control::wrap_degrees(10.0f), 10.0f, kTol);
}

TEST(YawController, HeadingHoldHasNoFeedforward) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(90.0f);
    float out = yaw.update(0.0f, kNoseFirstThrottle, 80.0f, 0.0f, kDt);
    float command = config.k_heading * 10.0f;
    EXPECT_NEAR(yaw.rate_command(), command, kTol);
    EXPECT_NEAR(out, config.kp * command, kTol);
}

TEST(YawController, IdlesStandingStillWithStickCentered) {
    Config config;
    YawController yaw(config);
    yaw.reset(90.0f);
    float out = yaw.update(0.5f, config.idle_throttle - 0.5f, 60.0f, 200.0f, kDt);
    EXPECT_EQ(out, 0.0f);
    EXPECT_EQ(yaw.rate_command(), 0.0f);
    EXPECT_NEAR(yaw.setpoint(), 60.0f, kTol);
    yaw.update(0.0f, kNoseFirstThrottle, 60.0f, 0.0f, kDt);
    EXPECT_NEAR(yaw.rate_command(), 0.0f, kTol);
}

TEST(YawController, TurnsInPlaceStandingStill) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(0.0f);
    float out = yaw.update(25.0f, 0.0f, 0.0f, 0.0f, kDt);
    float command = 0.25f * config.max_rate;
    EXPECT_NEAR(out, (config.feedforward + config.kp) * command, kTol);
}

TEST(YawController, HeadingHoldWrapsAcrossNorth) {
    YawController yaw(p_only());
    yaw.reset(355.0f);
    yaw.update(0.0f, kNoseFirstThrottle, 5.0f, 0.0f, kDt);
    EXPECT_LT(yaw.rate_command(), 0.0f);
}

TEST(YawController, HeadingHoldRateIsCapped) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(0.0f);
    yaw.update(0.0f, kNoseFirstThrottle, 170.0f, 0.0f, kDt);
    EXPECT_NEAR(yaw.rate_command(), -config.hold_rate_max, kTol);
}

TEST(YawController, StickCommandsRate) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(0.0f);
    float out = yaw.update(25.0f, kNoseFirstThrottle, 0.0f, 100.0f, kDt);
    float command = 0.25f * config.max_rate;
    EXPECT_NEAR(yaw.rate_command(), command, kTol);
    EXPECT_NEAR(out, config.feedforward * command + config.kp * (command - 100.0f), kTol);
}

TEST(YawController, ReverseRaisesFeedbackAndLowersFeedforward) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(0.0f);
    float out = yaw.update(25.0f, 100.0f, 0.0f, 100.0f, kDt);
    float command = 0.25f * config.max_rate;
    float expected = config.feedforward * config.reverse_ff_scale * command +
                     config.kp * config.reverse_kp_scale * (command - 100.0f);
    EXPECT_NEAR(out, expected, kTol);
}

TEST(YawController, ReleasedTurnBrakesThenHoldsWhereItStopped) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(0.0f);
    yaw.update(50.0f, kNoseFirstThrottle, 0.0f, 0.0f, kDt);
    yaw.update(0.0f, kNoseFirstThrottle, 40.0f, 300.0f, kDt);
    EXPECT_TRUE(yaw.capturing());
    EXPECT_EQ(yaw.rate_command(), 0.0f);
    EXPECT_LT(yaw.output(), 0.0f);
    yaw.update(0.0f, kNoseFirstThrottle, 52.0f, config.capture_rate - 1.0f, kDt);
    EXPECT_FALSE(yaw.capturing());
    EXPECT_NEAR(yaw.setpoint(), 52.0f, kTol);
}

TEST(YawController, CaptureTimesOut) {
    Config config = p_only();
    YawController yaw(config);
    yaw.reset(0.0f);
    yaw.update(50.0f, kNoseFirstThrottle, 0.0f, 0.0f, kDt);
    float heading = 0.0f;
    int steps = 0;
    while (yaw.update(0.0f, kNoseFirstThrottle, heading, 500.0f, kDt), yaw.capturing()) {
        heading += 5.0f;
        steps++;
        ASSERT_LT(steps, 100);
    }
    EXPECT_NEAR(steps * kDt, config.capture_timeout, 2 * kDt);
    EXPECT_NEAR(yaw.setpoint(), heading, kTol);
}

TEST(YawController, IntegralRemovesSteadyRateError) {
    Config config;
    YawController yaw(config);
    yaw.reset(0.0f);
    float first = yaw.update(0.0f, kNoseFirstThrottle, 0.0f, -10.0f, kDt);
    float later = first;
    for (int i = 0; i < 100; i++) later = yaw.update(0.0f, kNoseFirstThrottle, 0.0f, -10.0f, kDt);
    EXPECT_NEAR(later - first, config.ki * 10.0f * 1.0f, 1e-3f);
}

TEST(YawController, IntegralHeldWhileSaturated) {
    Config config;
    config.ki = 10.0f;
    YawController yaw(config);
    yaw.reset(0.0f);
    for (int i = 0; i < 200; i++) yaw.update(100.0f, kNoseFirstThrottle, 0.0f, 0.0f, kDt);
    EXPECT_NEAR(yaw.output(), 100.0f, kTol);
    float out = yaw.update(100.0f, kNoseFirstThrottle, 0.0f, config.max_rate, kDt);
    EXPECT_NEAR(out, config.feedforward * config.max_rate, kTol);
}

TEST(YawController, IntegralClampedToIMax) {
    Config config;
    config.ki = 1000.0f;
    YawController yaw(config);
    yaw.reset(0.0f);
    for (int i = 0; i < 1000; i++) yaw.update(0.0f, kNoseFirstThrottle, 0.0f, -1.0f, kDt);
    EXPECT_NEAR(yaw.output(), config.kp * 1.0f + config.i_max, kTol);
}

TEST(YawController, ResetClearsState) {
    Config config;
    YawController yaw(config);
    yaw.reset(0.0f);
    for (int i = 0; i < 50; i++) yaw.update(0.0f, kNoseFirstThrottle, 0.0f, -50.0f, kDt);
    yaw.update(40.0f, kNoseFirstThrottle, 0.0f, 0.0f, kDt);
    yaw.reset(123.0f);
    EXPECT_FALSE(yaw.capturing());
    EXPECT_EQ(yaw.output(), 0.0f);
    EXPECT_NEAR(yaw.update(0.0f, kNoseFirstThrottle, 123.0f, 0.0f, kDt), 0.0f, kTol);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

namespace {
const float kDegToRad = 3.14159265358979f / 180.0f;
}  // namespace

TEST(UnwrapGyroRate, PassesInRangeReadingsThrough) {
    EXPECT_NEAR(yaw_control::unwrap_gyro_rate(0.5f, 0.4f), 0.5f, kTol);
    EXPECT_NEAR(yaw_control::unwrap_gyro_rate(-30.0f, -29.0f), -30.0f, kTol);
}

// 2026-10-25 01:44 recording: yaw_rate went from -1950 to +1997 deg/s in one sample while the
// robot kept spinning the same way. The unwrapped rate stays negative.
TEST(UnwrapGyroRate, KeepsTheSignPastTheRange) {
    const float unwrapped =
        yaw_control::unwrap_gyro_rate(1997.0f * kDegToRad, -1950.0f * kDegToRad);
    EXPECT_NEAR(unwrapped / kDegToRad, 1997.0f - 4096.0f, 0.1f);
}

TEST(UnwrapGyroRate, TracksASpinUpPastTheRange) {
    float unwrapped = 0.0f;
    for (float truth = 0.0f; truth >= -3400.0f; truth -= 50.0f) {
        const float raw =
            yaw_control::wrap_degrees(truth * (360.0f / 4096.0f)) * (4096.0f / 360.0f);
        unwrapped = yaw_control::unwrap_gyro_rate(raw * kDegToRad, unwrapped);
        ASSERT_NEAR(unwrapped / kDegToRad, truth, 0.5f);
    }
}
