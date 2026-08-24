#include <gtest/gtest.h>

#include <cmath>

#include "plant/mrs_buff_mk3_params.hpp"
#include "robot_filter/dead_reckoning_motion_estimator.hpp"
#include "robot_filter/kalman_motion_estimator.hpp"
#include "transform_utils.hpp"

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

/** Opponent-KF tests pin KALMAN explicitly: the config default is HOLD (opponent
 * prediction disabled), which would pin every opponent at its last measured pose. */
KalmanMotionEstimatorConfiguration kalman_opponents_config() {
    KalmanMotionEstimatorConfiguration config;
    config.opponent_mode = OpponentMode::KALMAN;
    config.plant = mrs_buff_mk3_plant();
    return config;
}

/** Our-robot DEAD_RECKONING config with the plant the estimator now needs to give a normalized
 *  stick command a physical scale. Config loading requires the table in both modes. */
KalmanMotionEstimatorConfiguration dead_reckoning_config() {
    KalmanMotionEstimatorConfiguration config;
    config.plant = mrs_buff_mk3_plant();
    return config;
}

DeadReckoningMotionEstimatorConfiguration dr_estimator_config() {
    DeadReckoningMotionEstimatorConfiguration config;
    config.plant = mrs_buff_mk3_plant();
    return config;
}

TEST(KalmanMotionEstimatorTest, ConvergesToConstantVelocityAndPredictsAhead) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
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
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
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
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
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
    KalmanMotionEstimator estimator{kalman_opponents_config()};
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
    KalmanMotionEstimator kalman{dead_reckoning_config()};
    DeadReckoningMotionEstimator dead_reckoning{dr_estimator_config()};
    FrameIdAssigner kalman_assigner(10.0, 5);
    FrameIdAssigner dead_reckoning_assigner(10.0, 5);
    const FieldDescription field = make_field();
    MotionEstimatorContext context;
    context.our_robot_hold_window_s = 10.0;

    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = VelocityCommand{1.0, 0.0, 0.5};

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

/** The dead-reckoning arm scales the normalized stick command to body velocity with the fitted
 * gains. Before [plant] became the single source of physical scaling this factor came from
 * transmitter-local wheel geometry, which disagreed with the fit by 24% forward and 21% in yaw,
 * and every robot inherited Mrs Buff Mk3's numbers. */
TEST(KalmanMotionEstimatorTest, OurRobotDeadReckoningScalesStickByPlantGains) {
    KalmanMotionEstimatorConfiguration config;
    JigPlantParams plant;  // deliberately asymmetric and non-unit
    plant.k_fwd = 4.0;
    plant.k_rev = 2.0;
    plant.k_ang = 10.0;
    config.plant = plant;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    MotionEstimatorContext context;
    context.our_robot_hold_window_s = 10.0;

    // Half stick forward, no yaw: one unmeasured frame advances x by 0.5 * k_fwd * dt.
    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = VelocityCommand{0.5, 0.0, 0.0};
    const double t0 = kStartTime;
    estimator.predict(t0, feedback);
    estimator.update({make_our_measurement(0.0, 0.0)}, t0, assigner, field, context);

    estimator.predict(t0 + kDt, feedback);
    auto outputs = estimator.update({}, t0 + kDt, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(outputs[0].pose.position.x, 0.5 * 4.0 * kDt, 1e-9);
}

/** Reverse stick selects k_rev, not k_fwd. A naive `k_fwd * u` gets every backward coast wrong,
 * and the fitted gains are asymmetric (4.88 forward vs 4.36 reverse on Mrs Buff Mk3). */
TEST(KalmanMotionEstimatorTest, OurRobotDeadReckoningUsesReverseGainBackward) {
    KalmanMotionEstimatorConfiguration config;
    JigPlantParams plant;
    plant.k_fwd = 4.0;
    plant.k_rev = 2.0;
    plant.k_ang = 10.0;
    config.plant = plant;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    MotionEstimatorContext context;
    context.our_robot_hold_window_s = 10.0;

    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = VelocityCommand{-0.5, 0.0, 0.0};
    const double t0 = kStartTime;
    estimator.predict(t0, feedback);
    estimator.update({make_our_measurement(0.0, 0.0)}, t0, assigner, field, context);

    estimator.predict(t0 + kDt, feedback);
    auto outputs = estimator.update({}, t0 + kDt, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(outputs[0].pose.position.x, -0.5 * 2.0 * kDt, 1e-9);
}

TEST(KalmanMotionEstimatorTest, OurRobotDropsAfterHoldWindow) {
    KalmanMotionEstimator estimator{dead_reckoning_config()};
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

/** Stage A fit (plant_stageA.toml, 2026-08-23), same values as the config chain carries. */
JigPlantParams stage_a_params() {
    JigPlantParams params;
    params.dz_lin_fwd = 0.0109768;
    params.dz_lin_rev = 0.0253396;
    params.dz_ang_l = 0.016061;
    params.dz_ang_r = 0.0240734;
    params.k_fwd = 4.88002;
    params.k_rev = 4.3546;
    params.k_ang = 31.7062;
    params.tau_lin_a = 0.14921;
    params.tau_lin_d = 0.123461;
    params.tau_ang_a = 0.173867;
    params.tau_ang_d = 0.0878998;
    params.delay_s = 0.0522094;
    params.c_sb = 2.70197;
    params.c_ad = 0.463423;
    return params;
}

KalmanMotionEstimatorConfiguration ekf_config() {
    KalmanMotionEstimatorConfiguration config;
    config.our_robot_mode = OurRobotMode::EKF;
    config.plant = stage_a_params();
    return config;
}

/** A keypoint-derived our-robot pose measurement: keypoints non-empty marks it as carrying a
 * trustworthy heading, matching what the front/back converter emits. */
RobotDescription make_our_pose_measurement(const PlantState &truth) {
    RobotDescription description;
    description.frame_id = FrameId::OUR_ROBOT_1;
    description.label = Label::MR_STABS_MK1;
    description.group = Group::OURS;
    description.pose = pose2d_to_pose(Pose2D{truth.x, truth.y, truth.theta});
    description.size = Size{0.25, 0.25, 0.1};
    description.keypoints.push_back(Position{truth.x, truth.y, 0.0});
    return description;
}

/** Runs `steps` measured frames at 30 Hz with a constant command, generating the measurement
 * from a truth plant driven by the same command history the estimator sees. */
double run_ekf_measured_phase(KalmanMotionEstimator &estimator, JigPlantModel &truth_model,
                              PlantState &truth, std::vector<TimedCommand> &truth_history,
                              const VelocityCommand &command, FrameIdAssigner &assigner,
                              const FieldDescription &field, const MotionEstimatorContext &context,
                              int steps) {
    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = command;
    double t = kStartTime;
    for (int i = 0; i < steps; ++i) {
        estimator.predict(t, feedback);
        truth_history.push_back(TimedCommand{t, command});
        if (i > 0) {
            truth = truth_model.propagate_unwrapped(truth, truth_history, t - kDt, t);
        }
        estimator.update({make_our_pose_measurement(truth)}, t, assigner, field, context);
        if (i + 1 < steps) t += kDt;
    }
    return t;
}

TEST(KalmanMotionEstimatorTest, OurRobotEkfTracksPlantTruthThroughDropout) {
    KalmanMotionEstimator estimator{ekf_config()};
    JigPlantModel truth_model(stage_a_params());
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    // A gentle arc: v about 1.3 m/s, w about 0.95 rad/s under the stage A gains.
    const VelocityCommand command{0.3, 0.0, 0.05};
    PlantState truth;
    std::vector<TimedCommand> truth_history;
    double t = run_ekf_measured_phase(estimator, truth_model, truth, truth_history, command,
                                      assigner, field, context, 60);
    ASSERT_GT(truth.v, 1.0);

    // 300 ms of dropout, inside the 400 ms coast horizon: the filter propagates through the
    // plant model with the held command and stays on the truth it was converged to.
    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = command;
    for (int i = 0; i < 9; ++i) {
        const double prev_t = t;
        t += kDt;
        estimator.predict(t, feedback);
        truth_history.push_back(TimedCommand{t, command});
        truth = truth_model.propagate_unwrapped(truth, truth_history, prev_t, t);
        auto outputs = estimator.update({}, t, assigner, field, context);
        ASSERT_EQ(outputs.size(), 1u);
        EXPECT_TRUE(outputs[0].is_stale);
        EXPECT_NEAR(outputs[0].pose.position.x, truth.x, 0.03) << "dropout frame " << i;
        EXPECT_NEAR(outputs[0].pose.position.y, truth.y, 0.03) << "dropout frame " << i;
        const double yaw = pose_to_pose2d(outputs[0].pose).yaw;
        EXPECT_NEAR(std::remainder(yaw - truth.theta, 2.0 * M_PI), 0.0, 0.05)
            << "dropout frame " << i;
    }

    // Past the coast horizon the pose freezes instead of integrating fiction.
    std::vector<RobotDescription> frozen;
    for (int i = 0; i < 9; ++i) {
        t += kDt;
        estimator.predict(t, feedback);
        frozen = estimator.update({}, t, assigner, field, context);
        ASSERT_EQ(frozen.size(), 1u);
    }
    const double frozen_x = frozen[0].pose.position.x;
    t += kDt;
    estimator.predict(t, feedback);
    frozen = estimator.update({}, t, assigner, field, context);
    ASSERT_EQ(frozen.size(), 1u);
    EXPECT_NEAR(frozen[0].pose.position.x, frozen_x, 1e-9);
}

TEST(KalmanMotionEstimatorTest, OurRobotEkfHeadingFlipCorrectsPositionOnly) {
    KalmanMotionEstimator estimator{ekf_config()};
    JigPlantModel truth_model(stage_a_params());
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const VelocityCommand command{0.3, 0.0, 0.0};
    PlantState truth;
    std::vector<TimedCommand> truth_history;
    double t = run_ekf_measured_phase(estimator, truth_model, truth, truth_history, command,
                                      assigner, field, context, 40);

    // The front/back converter mislabels: heading arrives flipped by pi, position correct.
    t += kDt;
    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = command;
    estimator.predict(t, feedback);
    truth_history.push_back(TimedCommand{t, command});
    truth = truth_model.propagate_unwrapped(truth, truth_history, t - kDt, t);
    PlantState flipped = truth;
    flipped.theta = truth.theta + M_PI;
    auto outputs =
        estimator.update({make_our_pose_measurement(flipped)}, t, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    // Position folded in, heading kept: the estimate must not spin toward the flip.
    EXPECT_FALSE(outputs[0].is_stale);
    EXPECT_NEAR(outputs[0].pose.position.x, truth.x, 0.03);
    const double yaw = pose_to_pose2d(outputs[0].pose).yaw;
    EXPECT_NEAR(std::remainder(yaw - truth.theta, 2.0 * M_PI), 0.0, 0.1);
}

TEST(KalmanMotionEstimatorTest, HoldModePinsOpponentAtLastMeasurement) {
    KalmanMotionEstimatorConfiguration config;
    config.opponent_mode = OpponentMode::HOLD;
    config.plant = mrs_buff_mk3_plant();
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    double x = 1.0;
    double y = 2.0;
    const double last_t =
        run_constant_velocity(estimator, assigner, field, context, 30, x, y, 1.5, 0.0);

    // Dropout: the output stays pinned at the last measured pose, no extrapolation, and
    // coast() holds the same pose between frames.
    double t = last_t;
    for (int i = 0; i < 6; ++i) {
        t += kDt;
        estimator.predict(t, CommandFeedback{});
        auto outputs = estimator.update({}, t, assigner, field, context);
        ASSERT_EQ(outputs.size(), 1u);
        EXPECT_TRUE(outputs[0].is_stale);
        EXPECT_DOUBLE_EQ(outputs[0].pose.position.x, x);
        EXPECT_DOUBLE_EQ(outputs[0].pose.position.y, y);
    }
    auto coasted = estimator.coast(t + 0.05);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    EXPECT_DOUBLE_EQ((*coasted)[0].pose.position.x, x);
    EXPECT_TRUE((*coasted)[0].is_stale);
}

TEST(KalmanMotionEstimatorTest, ResetClearsTracks) {
    KalmanMotionEstimator estimator{dead_reckoning_config()};
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
