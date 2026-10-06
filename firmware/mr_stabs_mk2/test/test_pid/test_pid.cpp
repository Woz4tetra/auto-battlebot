#include <gtest/gtest.h>
#include <pid.h>

namespace {
const float kTol = 1e-4f;

// update() with setpoint 0, so the error is -measurement.
float step(pid::Pid &pid, float error, float dt) { return pid.update(0.0f, -error, dt); }
}  // namespace

TEST(Pid, ProportionalOnError) {
    pid::PidConfig config;
    config.kp = 0.5f;
    pid::Pid pid(config);
    EXPECT_NEAR(pid.update(10.0f, 4.0f, 0.01f), 3.0f, kTol);
}

TEST(Pid, NonPositiveDtOutputsZero) {
    pid::PidConfig config;
    config.kp = 1.0f;
    pid::Pid pid(config);
    EXPECT_EQ(step(pid, 5.0f, 0.0f), 0.0f);
    EXPECT_EQ(step(pid, 5.0f, -0.01f), 0.0f);
}

TEST(Pid, FirstUpdateHasNoDerivative) {
    pid::PidConfig config;
    config.kd = 1.0f;
    pid::Pid pid(config);
    EXPECT_EQ(step(pid, 5.0f, 0.01f), 0.0f);
}

TEST(Pid, DerivativeIsErrorChangeOverDt) {
    pid::PidConfig config;
    config.kd = 0.01f;
    pid::Pid pid(config);
    step(pid, 0.0f, 0.01f);
    EXPECT_NEAR(step(pid, 0.5f, 0.01f), 0.01f * 0.5f / 0.01f, kTol);
    EXPECT_NEAR(step(pid, 0.5f, 0.01f), 0.0f, kTol);
}

TEST(Pid, OutputIsZeroInsideToleranceBand) {
    pid::PidConfig config;
    config.kp = 1.0f;
    config.kd = 1.0f;
    config.tolerance = 2.0f;
    pid::Pid pid(config);
    step(pid, 3.0f, 0.01f);
    EXPECT_EQ(step(pid, 1.0f, 0.01f), 0.0f);
    EXPECT_EQ(step(pid, -1.9f, 0.01f), 0.0f);
}

// The old update() returned before the D term inside the band, so prev_error kept the value from
// before the band. Drifting through the band then produced kd * (band width) / dt in one step.
TEST(Pid, LeavingToleranceBandDoesNotKick) {
    pid::PidConfig config;
    config.kd = 0.01f;
    config.tolerance = 2.0f;
    pid::Pid pid(config);
    const float dt = 0.01f;
    step(pid, 2.1f, dt);
    step(pid, 0.0f, dt);
    // Only the change since the last in-band sample counts: -2.1, not -4.2.
    EXPECT_NEAR(step(pid, -2.1f, dt), 0.01f * -2.1f / dt, kTol);
}

TEST(Pid, IntegralIsErrorTimesSeconds) {
    pid::PidConfig config;
    config.ki = 2.0f;
    pid::Pid pid(config);
    float output = 0.0f;
    for (int i = 0; i < 100; i++) output = step(pid, 3.0f, 0.01f);
    EXPECT_NEAR(output, 2.0f * 3.0f * 1.0f, 1e-3f);
}

// The old integral summed errors per call and scaled by the latest dt, so one long step
// multiplied everything accumulated so far.
TEST(Pid, IntegralFollowsTimeWithUnevenDt) {
    pid::PidConfig config;
    config.ki = 1.0f;
    pid::Pid pid(config);
    for (int i = 0; i < 50; i++) step(pid, 5.0f, 0.01f);
    EXPECT_NEAR(step(pid, 5.0f, 0.1f), 5.0f * 0.6f, 1e-3f);
}

TEST(Pid, IntegralOutputClampedToIMax) {
    pid::PidConfig config;
    config.ki = 1.0f;
    config.i_max = 2.0f;
    pid::Pid pid(config);
    for (int i = 0; i < 1000; i++) step(pid, 5.0f, 0.01f);
    EXPECT_NEAR(step(pid, 5.0f, 0.01f), 2.0f, kTol);
    // Unwinds from the clamp, not from the unclamped total
    for (int i = 0; i < 100; i++) step(pid, -5.0f, 0.01f);
    EXPECT_NEAR(step(pid, -5.0f, 0.01f), -2.0f, kTol);
}

TEST(Pid, IntegralOnlyAccumulatesInsideIZone) {
    pid::PidConfig config;
    config.ki = 1.0f;
    config.i_zone = 4.0f;
    pid::Pid pid(config);
    for (int i = 0; i < 100; i++) step(pid, 10.0f, 0.01f);
    EXPECT_NEAR(step(pid, 10.0f, 0.01f), 0.0f, kTol);
    EXPECT_NEAR(step(pid, 2.0f, 0.5f), 1.0f, kTol);
}

TEST(Pid, ResetClearsIntegralAndDerivative) {
    pid::PidConfig config;
    config.ki = 1.0f;
    config.kd = 1.0f;
    pid::Pid pid(config);
    for (int i = 0; i < 10; i++) step(pid, 5.0f, 0.1f);
    pid.reset();
    EXPECT_NEAR(step(pid, 1.0f, 0.1f), 0.1f, kTol);  // integral only, no D on the first update
}

TEST(Pid, ContinuousWrapsErrorAcross180) {
    pid::PidConfig config;
    config.kp = 1.0f;
    config.continuous = true;
    pid::Pid pid(config);
    EXPECT_NEAR(pid.update(170.0f, -170.0f, 0.01f), -20.0f, kTol);
    EXPECT_NEAR(pid.update(-170.0f, 170.0f, 0.01f), 20.0f, kTol);
}

TEST(Pid, ContinuousDerivativeAcrossWrap) {
    pid::PidConfig config;
    config.kd = 1.0f;
    config.continuous = true;
    pid::Pid pid(config);
    pid.update(0.0f, -179.0f, 0.01f);  // error 179
    // Error wraps to -179, a change of 2 degrees, not -358
    EXPECT_NEAR(pid.update(0.0f, 179.0f, 0.01f), 2.0f / 0.01f, 1e-2f);
}

TEST(Pid, PdScaleMultipliesOnlyProportionalAndDerivative) {
    pid::PidConfig config;
    config.kp = 1.0f;
    config.ki = 1.0f;
    config.kd = 0.1f;
    pid::Pid scaled(config);
    pid::Pid plain(config);
    step(scaled, 0.0f, 0.1f);
    step(plain, 0.0f, 0.1f);
    // error 2 over 0.1 s: P 2, I 0.2, D 0.1 * 2 / 0.1 = 2
    EXPECT_NEAR(scaled.update(0.0f, -2.0f, 0.1f, 3.0f), 3.0f * (2.0f + 2.0f) + 0.2f, kTol);
    EXPECT_NEAR(step(plain, 2.0f, 0.1f), 2.0f + 2.0f + 0.2f, kTol);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
