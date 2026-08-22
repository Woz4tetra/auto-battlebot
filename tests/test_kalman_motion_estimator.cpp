#include <gtest/gtest.h>

#include <cmath>

#include "robot_filter/dead_reckoning_motion_estimator.hpp"
#include "robot_filter/kalman_motion_estimator.hpp"

namespace auto_battlebot {
namespace {

constexpr double kDt = 1.0 / 30.0;
constexpr double kStartTime = 100.0;

FieldDescription make_field() {
    FieldDescription field;
    field.child_frame_id = FrameId::FIELD;
    field.size.size.x = 10.0;
    field.size.size.y = 10.0;
    return field;
}

RobotDescription make_opponent_measurement(double x, double y,
                                           FrameId frame_id = FrameId::THEIR_ROBOT_1) {
    RobotDescription description;
    description.frame_id = frame_id;
    description.label = Label::OPPONENT;
    description.group = Group::THEIRS;
    description.pose.position = Position{x, y, 0.05};
    description.pose.rotation = Rotation{1.0, 0.0, 0.0, 0.0};
    description.size = Size{0.3, 0.3, 0.1};
    return description;
}

RobotDescription make_our_measurement(double x, double y) {
    RobotDescription description;
    description.frame_id = FrameId::OUR_ROBOT_1;
    description.label = Label::MR_STABS_MK1;
    description.group = Group::OURS;
    description.pose.position = Position{x, y, 0.0};
    description.pose.rotation = Rotation{1.0, 0.0, 0.0, 0.0};
    description.size = Size{0.25, 0.25, 0.1};
    return description;
}

/** Drives one opponent at constant velocity through `steps` measured frames at 30 Hz.
 * Returns the time of the last measurement; `x` and `y` end at the last measured position. */
double run_constant_velocity(KalmanMotionEstimator &estimator, FrameIdAssigner &assigner,
                             const FieldDescription &field, const MotionEstimatorContext &context,
                             int steps, double &x, double &y, double vx, double vy) {
    double t = kStartTime;
    for (int i = 0; i < steps; ++i) {
        estimator.predict(t, CommandFeedback{});
        estimator.update({make_opponent_measurement(x, y)}, t, assigner, field, context);
        if (i + 1 < steps) {
            t += kDt;
            x += vx * kDt;
            y += vy * kDt;
        }
    }
    return t;
}

TEST(KalmanMotionEstimatorTest, ConvergesToConstantVelocityAndPredictsAhead) {
    KalmanMotionEstimator estimator{KalmanMotionEstimatorConfiguration{}};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 1.5;
    const double vy = -0.75;
    double x = 1.0;
    double y = 2.0;
    const double last_t =
        run_constant_velocity(estimator, assigner, field, context, 60, x, y, vx, vy);

    estimator.predict(last_t, CommandFeedback{});
    auto coasted = estimator.coast(last_t);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    EXPECT_FALSE((*coasted)[0].is_stale);
    EXPECT_NEAR((*coasted)[0].velocity.vx, vx, 0.2);
    EXPECT_NEAR((*coasted)[0].velocity.vy, vy, 0.2);
    EXPECT_NEAR((*coasted)[0].pose.position.x, x, 0.05);
    EXPECT_NEAR((*coasted)[0].pose.position.y, y, 0.05);

    // Coasting 100 ms ahead of the last measurement tracks the constant-velocity truth.
    auto ahead = estimator.coast(last_t + 0.1);
    ASSERT_TRUE(ahead.has_value());
    ASSERT_EQ(ahead->size(), 1u);
    EXPECT_NEAR((*ahead)[0].pose.position.x, x + vx * 0.1, 0.06);
    EXPECT_NEAR((*ahead)[0].pose.position.y, y + vy * 0.1, 0.06);
}

TEST(KalmanMotionEstimatorTest, CoastHoldsPositionPastMaxCoast) {
    KalmanMotionEstimatorConfiguration config;
    config.max_coast_s = 0.4;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 1.5;
    double x = 1.0;
    double y = 2.0;
    const double last_t =
        run_constant_velocity(estimator, assigner, field, context, 60, x, y, vx, 0.0);

    // Advance in control-rate steps so the coast horizon is crossed mid-sequence.
    double position_at_half_second = 0.0;
    double final_position = 0.0;
    for (double offset = 0.05; offset <= 1.2 + 1e-9; offset += 0.05) {
        auto coasted = estimator.coast(last_t + offset);
        ASSERT_TRUE(coasted.has_value());
        ASSERT_EQ(coasted->size(), 1u);
        const double position = (*coasted)[0].pose.position.x;
        if (std::abs(offset - 0.5) < 1e-9) position_at_half_second = position;
        final_position = position;
        if (offset < 0.1) {
            // Between-frame coasting right after a measurement still reads as fresh.
            EXPECT_FALSE((*coasted)[0].is_stale) << "offset " << offset;
        } else if (offset > 0.15) {
            // A track unmeasured for several frame periods reads as stale even though
            // correct() never ran to say so.
            EXPECT_TRUE((*coasted)[0].is_stale) << "offset " << offset;
        }
    }

    // Position moved with the estimated velocity up to the horizon, then froze.
    EXPECT_GT(position_at_half_second, x + vx * 0.3);
    EXPECT_NEAR(final_position, position_at_half_second, 1e-9);
}

TEST(KalmanMotionEstimatorTest, GateRejectsJumpThenReinitializes) {
    KalmanMotionEstimatorConfiguration config;
    config.reinit_after_rejects = 3;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    double x = 1.0;
    double y = 2.0;
    double t = run_constant_velocity(estimator, assigner, field, context, 60, x, y, 0.5, 0.0);

    const double jump_x = x + 3.0;
    for (int reject = 1; reject <= 2; ++reject) {
        t += kDt;
        estimator.predict(t, CommandFeedback{});
        auto outputs =
            estimator.update({make_opponent_measurement(jump_x, y)}, t, assigner, field, context);
        ASSERT_EQ(outputs.size(), 1u);
        EXPECT_TRUE(outputs[0].is_stale) << "gated frame " << reject;
        EXPECT_LT(std::abs(outputs[0].pose.position.x - x), 0.5) << "gated frame " << reject;
    }

    // Third consecutive reject crosses reinit_after_rejects: the world moved, follow it.
    t += kDt;
    estimator.predict(t, CommandFeedback{});
    auto outputs =
        estimator.update({make_opponent_measurement(jump_x, y)}, t, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_FALSE(outputs[0].is_stale);
    EXPECT_NEAR(outputs[0].pose.position.x, jump_x, 1e-6);
}

TEST(KalmanMotionEstimatorTest, LateMeasurementRetrodictsInsteadOfCorruptingTrack) {
    KalmanMotionEstimator estimator{KalmanMotionEstimatorConfiguration{}};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 1.5;
    double x = 1.0;
    double y = 2.0;
    const double last_t =
        run_constant_velocity(estimator, assigner, field, context, 60, x, y, vx, 0.0);

    // The control loop runs ahead of the camera: the track coasts to last_t + 50 ms, then a
    // measurement stamped at last_t + 33 ms arrives.
    estimator.predict(last_t + 0.05, CommandFeedback{});
    estimator.coast(last_t + 0.05);
    const double measurement_t = last_t + kDt;
    const double measurement_x = x + vx * kDt;
    auto outputs = estimator.update({make_opponent_measurement(measurement_x, y)}, measurement_t,
                                    assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_FALSE(outputs[0].is_stale);
    EXPECT_TRUE(std::isfinite(outputs[0].pose.position.x));
    EXPECT_NEAR(outputs[0].pose.position.x, measurement_x, 0.05);
    EXPECT_NEAR(outputs[0].pose.position.y, y, 0.05);

    // The next coast picks up from the corrected state and keeps tracking.
    auto coasted = estimator.coast(last_t + 0.1);
    ASSERT_TRUE(coasted.has_value());
    EXPECT_NEAR((*coasted)[0].pose.position.x, x + vx * 0.1, 0.06);
}

TEST(KalmanMotionEstimatorTest, OurRobotMatchesDeadReckoningEstimator) {
    KalmanMotionEstimator kalman{KalmanMotionEstimatorConfiguration{}};
    DeadReckoningMotionEstimator dead_reckoning;
    FrameIdAssigner kalman_assigner(10.0, 5);
    FrameIdAssigner dead_reckoning_assigner(10.0, 5);
    const FieldDescription field = make_field();
    MotionEstimatorContext context;
    context.our_robot_hold_window_s = 10.0;

    CommandFeedback feedback;
    feedback.commands[FrameId::OUR_ROBOT_1] = VelocityCommand{1.0, 0.0, 0.5};

    const double t0 = kStartTime;
    kalman.predict(t0, feedback);
    dead_reckoning.predict(t0, feedback);
    auto kalman_out =
        kalman.update({make_our_measurement(0.0, 0.0)}, t0, kalman_assigner, field, context);
    auto dead_out = dead_reckoning.update({make_our_measurement(0.0, 0.0)}, t0,
                                          dead_reckoning_assigner, field, context);
    ASSERT_EQ(kalman_out.size(), dead_out.size());

    // Two unmeasured frames: both estimators dead-reckon our robot with the same command.
    for (int i = 1; i <= 2; ++i) {
        const double t = t0 + i * kDt;
        kalman.predict(t, feedback);
        dead_reckoning.predict(t, feedback);
        kalman_out = kalman.update({}, t, kalman_assigner, field, context);
        dead_out = dead_reckoning.update({}, t, dead_reckoning_assigner, field, context);
        ASSERT_EQ(kalman_out.size(), 1u);
        ASSERT_EQ(dead_out.size(), 1u);
        EXPECT_NEAR(kalman_out[0].pose.position.x, dead_out[0].pose.position.x, 1e-12);
        EXPECT_NEAR(kalman_out[0].pose.position.y, dead_out[0].pose.position.y, 1e-12);
        EXPECT_NEAR(kalman_out[0].pose.rotation.w, dead_out[0].pose.rotation.w, 1e-12);
        EXPECT_NEAR(kalman_out[0].pose.rotation.z, dead_out[0].pose.rotation.z, 1e-12);
        EXPECT_EQ(kalman_out[0].is_stale, dead_out[0].is_stale);
    }
}

TEST(KalmanMotionEstimatorTest, OurRobotDropsAfterHoldWindow) {
    KalmanMotionEstimator estimator{KalmanMotionEstimatorConfiguration{}};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    MotionEstimatorContext context;
    context.our_robot_hold_window_s = 0.15;

    const double t0 = kStartTime;
    estimator.predict(t0, CommandFeedback{});
    auto outputs = estimator.update({make_our_measurement(0.0, 0.0)}, t0, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);

    outputs = estimator.update({}, t0 + 0.1, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_TRUE(outputs[0].is_stale);

    // Past the hold window the stale our-robot track decays away instead of being held forever.
    outputs = estimator.update({}, t0 + 0.2, assigner, field, context);
    EXPECT_TRUE(outputs.empty());
}

TEST(KalmanMotionEstimatorTest, ResetClearsTracks) {
    KalmanMotionEstimator estimator{KalmanMotionEstimatorConfiguration{}};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    estimator.update({make_opponent_measurement(1.0, 1.0)}, kStartTime, assigner, field, context);
    estimator.reset();
    auto outputs = estimator.update({}, kStartTime + kDt, assigner, field, context);
    EXPECT_TRUE(outputs.empty());
}

}  // namespace
}  // namespace auto_battlebot
