#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <magic_enum.hpp>
#include <sstream>

#include "diagnostics_logger/test_diagnostics_logger.hpp"
#include "plant/mrs_buff_mk3_params.hpp"
#include "robot_filter/kalman_motion_estimator.hpp"
#include "robot_filter/opponent_filter_golden_data.hpp"
#include "transform_utils.hpp"

namespace auto_battlebot {
namespace {

constexpr double kDt = 1.0 / 30.0;
constexpr double kStartTime = 100.0;
constexpr double kCoastLongerThanAnyTestDropoutS = 0.4;
constexpr double kCoastLeadS = plant_golden::kParam_delay_s;

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

struct ConstantVelocityTrack {
    double last_measurement_t;
    double x;
    double y;
};

ConstantVelocityTrack run_constant_velocity(KalmanMotionEstimator &estimator,
                                            FrameIdAssigner &assigner,
                                            const FieldDescription &field,
                                            const MotionEstimatorContext &context, int steps,
                                            double x, double y, double vx, double vy) {
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
    return {t, x, y};
}

KalmanMotionEstimatorConfiguration kalman_opponents_config() {
    KalmanMotionEstimatorConfiguration config;
    config.plant = mrs_buff_mk3_plant();
    config.max_coast_s = kCoastLongerThanAnyTestDropoutS;
    return config;
}

KalmanMotionEstimatorConfiguration default_config() {
    KalmanMotionEstimatorConfiguration config;
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
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, vx, vy);

    estimator.predict(last_t, CommandFeedback{});
    auto coasted = estimator.coast(last_t);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    EXPECT_NEAR((*coasted)[0].velocity.vx_mps, vx, 0.2);
    EXPECT_NEAR((*coasted)[0].velocity.vy_mps, vy, 0.2);
    EXPECT_NEAR((*coasted)[0].pose.position.x, x + vx * kCoastLeadS, 0.05);
    EXPECT_NEAR((*coasted)[0].pose.position.y, y + vy * kCoastLeadS, 0.05);

    constexpr double kCoastAheadS = 0.1;
    auto ahead = estimator.coast(last_t + kCoastAheadS);
    ASSERT_TRUE(ahead.has_value());
    ASSERT_EQ(ahead->size(), 1u);
    EXPECT_NEAR((*ahead)[0].pose.position.x, x + vx * (kCoastAheadS + kCoastLeadS), 0.06);
    EXPECT_NEAR((*ahead)[0].pose.position.y, y + vy * (kCoastAheadS + kCoastLeadS), 0.06);
}

TEST(KalmanMotionEstimatorTest, CoastLeadsTheMeasurementByTheTransportDelay) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 2.0;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 0.0, 0.0, vx, 0.0);

    estimator.predict(last_t, CommandFeedback{});
    auto outputs =
        estimator.update({make_opponent_measurement(x, y)}, last_t, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(outputs[0].pose.position.x, x, 0.02) << "update() renders at the shutter stamp";

    auto coasted = estimator.coast(last_t);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    const double lead_distance = (*coasted)[0].pose.position.x - outputs[0].pose.position.x;
    EXPECT_NEAR(lead_distance, vx * kCoastLeadS, 0.02);
    EXPECT_GT(lead_distance, 0.0);
}

TEST(KalmanMotionEstimatorTest, CoastHoldsPositionPastMaxCoast) {
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.max_coast_s = kCoastLongerThanAnyTestDropoutS;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 1.5;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, vx, 0.0);

    double position_at_half_second = 0.0;
    double final_position = 0.0;
    for (double offset = 0.05; offset <= 3.0 * config.max_coast_s + 1e-9; offset += 0.05) {
        auto coasted = estimator.coast(last_t + offset);
        ASSERT_TRUE(coasted.has_value());
        ASSERT_EQ(coasted->size(), 1u);
        const double position = (*coasted)[0].pose.position.x;
        if (std::abs(offset - 0.5) < 1e-9) position_at_half_second = position;
        final_position = position;
        const double rendered_age = offset + kCoastLeadS;
        const double velocity = (*coasted)[0].velocity.vx_mps;
        if (rendered_age < config.max_coast_s - 1e-9) {
            EXPECT_GT(velocity, 0.5 * vx) << "offset " << offset;
        } else if (rendered_age > config.max_coast_s + 1e-9) {
            EXPECT_EQ(velocity, 0.0) << "offset " << offset << ": a frozen mean reports motion";
        }
    }

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

    auto [t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, 0.5, 0.0);

    const double jump_x = x + 3.0;
    for (int reject = 1; reject < config.reinit_after_rejects; ++reject) {
        t += kDt;
        estimator.predict(t, CommandFeedback{});
        auto outputs =
            estimator.update({make_opponent_measurement(jump_x, y)}, t, assigner, field, context);
        ASSERT_EQ(outputs.size(), 1u);
        EXPECT_LT(std::abs(outputs[0].pose.position.x - x), 0.5) << "gated frame " << reject;
    }

    t += kDt;
    estimator.predict(t, CommandFeedback{});
    auto outputs =
        estimator.update({make_opponent_measurement(jump_x, y)}, t, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(outputs[0].pose.position.x, jump_x, 1e-6);
}

TEST(KalmanMotionEstimatorTest, LateMeasurementRetrodictsInsteadOfCorruptingTrack) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 1.5;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, vx, 0.0);

    estimator.predict(last_t + 0.05, CommandFeedback{});
    estimator.coast(last_t + 0.05);
    const double measurement_t = last_t + kDt;
    const double measurement_x = x + vx * kDt;
    auto outputs = estimator.update({make_opponent_measurement(measurement_x, y)}, measurement_t,
                                    assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_TRUE(std::isfinite(outputs[0].pose.position.x));
    EXPECT_NEAR(outputs[0].pose.position.x, measurement_x, 0.05);
    EXPECT_NEAR(outputs[0].pose.position.y, y, 0.05);

    auto coasted = estimator.coast(last_t + 0.1);
    ASSERT_TRUE(coasted.has_value());
    EXPECT_NEAR((*coasted)[0].pose.position.x, x + vx * (0.1 + kCoastLeadS), 0.06);
}

double sigma_of(const RobotDescription &description) {
    return description.uncertainty.position_sigma_major_axis_m.value_or(0.0);
}

double first_unmeasured_stamp_without(KalmanMotionEstimator &estimator, FrameIdAssigner &assigner,
                                      const RobotDescription &measurement, FrameId frame_id,
                                      double &last_sigma) {
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    double t = kStartTime;
    estimator.predict(t, CommandFeedback{});
    estimator.update({measurement}, t, assigner, field, context);
    for (int frame = 1; frame < 300; ++frame) {
        t = kStartTime + frame * kDt;
        estimator.predict(t, CommandFeedback{});
        const auto outputs = estimator.update({}, t, assigner, field, context);
        const auto found =
            std::find_if(outputs.begin(), outputs.end(),
                         [&](const RobotDescription &d) { return d.frame_id == frame_id; });
        if (found == outputs.end()) return t;
        last_sigma = sigma_of(*found);
    }
    return std::numeric_limits<double>::infinity();
}

TEST(KalmanMotionEstimatorTest, AnUnmeasuredTrackExpiresOnceItsPositionSigmaPassesTheLimit) {
    constexpr double kLimitM = 0.3;
    for (const RobotDescription &measurement :
         {make_our_measurement(0.0, 0.0), make_opponent_measurement(1.0, 1.0)}) {
        KalmanMotionEstimatorConfiguration config = default_config();
        config.max_position_sigma_m = kLimitM;
        KalmanMotionEstimator estimator{config};
        FrameIdAssigner assigner(10.0, 5);
        double last_sigma = 0.0;
        const double expired_at = first_unmeasured_stamp_without(estimator, assigner, measurement,
                                                                 measurement.frame_id, last_sigma);

        ASSERT_TRUE(std::isfinite(expired_at)) << magic_enum::enum_name(measurement.frame_id);
        EXPECT_GT(expired_at, kStartTime + kDt);
        EXPECT_LE(last_sigma, kLimitM);
        EXPECT_GT(last_sigma, 0.5 * kLimitM);
        EXPECT_EQ(assigner.last_positions().count(measurement.frame_id), 0u)
            << "an expired track must not keep gating association";
    }
}

TEST(KalmanMotionEstimatorTest, EveryTrackCarriesItsPositionCovariance) {
    KalmanMotionEstimator estimator{default_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    estimator.predict(kStartTime, CommandFeedback{});
    estimator.update({make_our_measurement(0.0, 0.0), make_opponent_measurement(1.0, 1.0)},
                     kStartTime, assigner, field, context);
    estimator.predict(kStartTime + 0.2, CommandFeedback{});
    const auto outputs = estimator.update({}, kStartTime + 0.2, assigner, field, context);
    ASSERT_EQ(outputs.size(), 2u);
    for (const RobotDescription &output : outputs) {
        ASSERT_TRUE(output.uncertainty.position_covariance_m2.has_value());
        const PositionCovariance &c = *output.uncertainty.position_covariance_m2;
        const double trace = c.xx + c.yy;
        const double det = c.xx * c.yy - c.xy * c.xy;
        const double largest = 0.5 * trace + std::sqrt(std::max(0.0, 0.25 * trace * trace - det));
        EXPECT_GT(det, 0.0);
        EXPECT_NEAR(std::sqrt(largest), *output.uncertainty.position_sigma_major_axis_m, 1e-9);
    }
}

TEST(KalmanMotionEstimatorTest, AMeasuredTrackNeverExpires) {
    KalmanMotionEstimatorConfiguration config = default_config();
    config.max_position_sigma_m = 0.05;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    const auto [t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 0.0, 0.0, 0.5, 0.0);
    const auto outputs =
        estimator.update({make_opponent_measurement(x, y)}, t + kDt, assigner, field, context);
    EXPECT_EQ(outputs.size(), 1u);
}

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
    config.plant = stage_a_params();
    return config;
}

RobotDescription make_our_pose_measurement(const PlantState &truth) {
    RobotDescription description;
    description.frame_id = FrameId::OUR_ROBOT_1;
    description.label = Label::MR_STABS_MK1;
    description.group = Group::OURS;
    description.pose = pose2d_to_pose(Pose2D{truth.x, truth.y, truth.theta});
    description.size = Size{0.25, 0.25, 0.1};
    return description;
}

double run_ekf_measured_phase(KalmanMotionEstimator &estimator, JigPlantModel &truth_model,
                              PlantState &truth, std::vector<TimedCommand> &truth_history,
                              const StickCommand &command, FrameIdAssigner &assigner,
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

    const StickCommand command{0.3, 0.0, 0.05};
    PlantState truth;
    std::vector<TimedCommand> truth_history;
    double t = run_ekf_measured_phase(estimator, truth_model, truth, truth_history, command,
                                      assigner, field, context, 60);
    ASSERT_GT(truth.v, 1.0);

    const double max_coast_s = ekf_config().max_coast_s;
    const int dropout_frames = static_cast<int>(0.75 * max_coast_s / kDt);
    CommandFeedback feedback;
    feedback.stick_commands[FrameId::OUR_ROBOT_1] = command;
    for (int i = 0; i < dropout_frames; ++i) {
        const double prev_t = t;
        t += kDt;
        estimator.predict(t, feedback);
        truth_history.push_back(TimedCommand{t, command});
        truth = truth_model.propagate_unwrapped(truth, truth_history, prev_t, t);
        auto outputs = estimator.update({}, t, assigner, field, context);
        ASSERT_EQ(outputs.size(), 1u);
        EXPECT_NEAR(outputs[0].pose.position.x, truth.x, 0.03) << "dropout frame " << i;
        EXPECT_NEAR(outputs[0].pose.position.y, truth.y, 0.03) << "dropout frame " << i;
        const double yaw = pose_to_pose2d(outputs[0].pose).yaw;
        EXPECT_NEAR(std::remainder(yaw - truth.theta, 2.0 * M_PI), 0.0, 0.05)
            << "dropout frame " << i;
    }

    const int frames_past_horizon = static_cast<int>(std::ceil(max_coast_s / kDt));
    std::vector<RobotDescription> frozen;
    for (int i = 0; i < frames_past_horizon; ++i) {
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
    EXPECT_EQ(frozen[0].velocity.vx_mps, 0.0) << "a frozen mean reports motion";
    EXPECT_EQ(frozen[0].velocity.vy_mps, 0.0);
    EXPECT_EQ(frozen[0].velocity.omega_radps, 0.0);

    auto coasted = estimator.coast(t);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    EXPECT_EQ((*coasted)[0].velocity.vx_mps, 0.0);
    EXPECT_EQ((*coasted)[0].velocity.omega_radps, 0.0);
}

TEST(KalmanMotionEstimatorTest, OurRobotEkfHeadingFlipCorrectsPositionOnly) {
    KalmanMotionEstimator estimator{ekf_config()};
    JigPlantModel truth_model(stage_a_params());
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const StickCommand command{0.3, 0.0, 0.0};
    PlantState truth;
    std::vector<TimedCommand> truth_history;
    double t = run_ekf_measured_phase(estimator, truth_model, truth, truth_history, command,
                                      assigner, field, context, 40);

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
    EXPECT_NEAR(outputs[0].pose.position.x, truth.x, 0.03);
    const double yaw = pose_to_pose2d(outputs[0].pose).yaw;
    EXPECT_NEAR(std::remainder(yaw - truth.theta, 2.0 * M_PI), 0.0, 0.1);
}

RobotDescription make_opponent_pose_measurement(double x, double y, double yaw,
                                                Label label = Label::OPPONENT) {
    RobotDescription description = make_opponent_measurement(x, y);
    description.label = label;
    description.pose = pose2d_to_pose(Pose2D{x, y, yaw});
    description.pose.position.z = 0.05;
    return description;
}

double yaw_of(const RobotDescription &description) { return pose_to_pose2d(description.pose).yaw; }

double angle_error(double a, double b) { return std::remainder(a - b, 2.0 * M_PI); }

struct TurningTruth {
    double x = 1.0;
    double y = 2.0;
    double yaw = 0.3;
    double vx = 0.8;
    double vy = -0.4;
    double omega = 2.0;
    void step(double dt) {
        x += vx * dt;
        y += vy * dt;
        yaw = std::remainder(yaw + omega * dt, 2.0 * M_PI);
    }
};

struct TurningRun {
    double last_measurement_t;
    TurningTruth truth;
};

TurningRun run_turning(KalmanMotionEstimator &estimator, FrameIdAssigner &assigner,
                       const FieldDescription &field, const MotionEstimatorContext &context,
                       int steps, TurningTruth truth, Label label = Label::OPPONENT) {
    double t = kStartTime;
    for (int i = 0; i < steps; ++i) {
        estimator.predict(t, CommandFeedback{});
        estimator.update({make_opponent_pose_measurement(truth.x, truth.y, truth.yaw, label)}, t,
                         assigner, field, context);
        if (i + 1 < steps) {
            t += kDt;
            truth.step(kDt);
        }
    }
    return {t, truth};
}

TEST(KalmanMotionEstimatorTest, OpponentHeadingAndYawRateConvergeOnConstantTurn) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    const auto [last_t, truth] = run_turning(estimator, assigner, field, context, 60, initial);

    auto coasted = estimator.coast(last_t);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    const RobotDescription &out = (*coasted)[0];
    EXPECT_NEAR(out.velocity.omega_radps, truth.omega, 0.1);
    EXPECT_NEAR(angle_error(yaw_of(out), truth.yaw + truth.omega * kCoastLeadS), 0.0, 0.03);
    const KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    EXPECT_TRUE(out.uncertainty.heading_sigma_rad.has_value());
    EXPECT_LT(*out.uncertainty.heading_sigma_rad, 2.0 * config.opponent_heading_sigma_rad);
    EXPECT_LT(*out.uncertainty.yaw_rate_sigma_rad_s, 0.25 * config.initial_yaw_rate_sigma);
}

TEST(KalmanMotionEstimatorTest, OpponentSingleHeadingFlipCorrectsPositionOnly) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    initial.omega = 0.0;
    auto [t, truth] = run_turning(estimator, assigner, field, context, 40, initial);

    constexpr double kSwapPositionErrorM = 0.05;
    t += kDt;
    truth.step(kDt);
    estimator.predict(t, CommandFeedback{});
    auto outputs = estimator.update(
        {make_opponent_pose_measurement(truth.x + kSwapPositionErrorM, truth.y, truth.yaw + M_PI)},
        t, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(angle_error(yaw_of(outputs[0]), truth.yaw), 0.0, 0.02)
        << "the flipped heading must not move the estimate";
    EXPECT_GT(outputs[0].pose.position.x, truth.x + 0.01) << "position still corrected";

    t += kDt;
    truth.step(kDt);
    estimator.predict(t, CommandFeedback{});
    outputs = estimator.update({make_opponent_pose_measurement(truth.x, truth.y, truth.yaw)}, t,
                               assigner, field, context);
    EXPECT_NEAR(angle_error(yaw_of(outputs[0]), truth.yaw), 0.0, 0.02);
}

TEST(KalmanMotionEstimatorTest, OpponentConsecutiveFlipsReinitializeHeadingOnly) {
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.heading_flip_reinit_count = 3;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    initial.omega = 0.0;
    auto [t, truth] = run_turning(estimator, assigner, field, context, 40, initial);

    std::vector<RobotDescription> outputs;
    for (int flip = 1; flip <= 3; ++flip) {
        t += kDt;
        truth.step(kDt);
        estimator.predict(t, CommandFeedback{});
        outputs =
            estimator.update({make_opponent_pose_measurement(truth.x, truth.y, truth.yaw + M_PI)},
                             t, assigner, field, context);
        ASSERT_EQ(outputs.size(), 1u);
        EXPECT_NEAR(outputs[0].pose.position.x, truth.x, 0.01) << "flip " << flip;
        EXPECT_NEAR(outputs[0].pose.position.y, truth.y, 0.01) << "flip " << flip;
        if (flip < 3) {
            EXPECT_NEAR(angle_error(yaw_of(outputs[0]), truth.yaw), 0.0, 0.02) << "flip " << flip;
        }
    }
    EXPECT_NEAR(angle_error(yaw_of(outputs[0]), truth.yaw + M_PI), 0.0, 1e-9);
    EXPECT_NEAR(outputs[0].velocity.omega_radps, 0.0, 1e-9);
    EXPECT_NEAR(*outputs[0].uncertainty.heading_sigma_rad, config.opponent_heading_sigma_rad, 1e-9);
    EXPECT_NEAR(outputs[0].velocity.vx_mps, truth.vx, 0.2);
}

TEST(KalmanMotionEstimatorTest, OpponentHeadingAfterLongDropoutIsNotCalledAFlip) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    initial.omega = 0.0;
    auto [t, truth] = run_turning(estimator, assigner, field, context, 40, initial);

    for (double offset = 0.004; offset <= 1.0; offset += 0.004) estimator.coast(t + offset);
    t += 1.0;
    truth.step(1.0);
    const double turned = truth.yaw + 2.0 * M_PI / 3.0;
    estimator.predict(t, CommandFeedback{});
    auto outputs = estimator.update({make_opponent_pose_measurement(truth.x, truth.y, turned)}, t,
                                    assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(angle_error(yaw_of(outputs[0]), turned), 0.0, 0.05);
}

TEST(KalmanMotionEstimatorTest, OpponentLateMeasurementRewindRestoresHeading) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    const auto [last_t, truth] = run_turning(estimator, assigner, field, context, 60, initial);

    constexpr double kCoastAheadS = 0.05;
    estimator.predict(last_t + kCoastAheadS, CommandFeedback{});
    estimator.coast(last_t + kCoastAheadS);
    TurningTruth late = truth;
    late.step(kDt);
    auto outputs = estimator.update({make_opponent_pose_measurement(late.x, late.y, late.yaw)},
                                    last_t + kDt, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_NEAR(angle_error(yaw_of(outputs[0]), late.yaw), 0.0, 0.03);
    EXPECT_NEAR(outputs[0].velocity.omega_radps, truth.omega, 0.1);
    EXPECT_NEAR(outputs[0].pose.position.x, late.x, 0.02);

    auto coasted = estimator.coast(last_t + 0.1);
    ASSERT_TRUE(coasted.has_value());
    EXPECT_NEAR(angle_error(yaw_of((*coasted)[0]), truth.yaw + truth.omega * (0.1 + kCoastLeadS)),
                0.0, 0.05);
}

TEST(KalmanMotionEstimatorTest, OpponentPastMaxCoastHoldsPositionHeadingAndYawRate) {
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.max_coast_s = 0.2;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    const auto [last_t, truth] = run_turning(estimator, assigner, field, context, 60, initial);

    RobotDescription at_one_second;
    RobotDescription at_two_seconds;
    for (double offset = 0.004; offset <= 2.0 + 1e-9; offset += 0.004) {
        auto coasted = estimator.coast(last_t + offset);
        ASSERT_TRUE(coasted.has_value());
        if (std::abs(offset - 1.0) < 2e-3) at_one_second = (*coasted)[0];
        at_two_seconds = (*coasted)[0];
    }
    EXPECT_NEAR(at_two_seconds.pose.position.x, at_one_second.pose.position.x, 1e-9);
    EXPECT_NEAR(at_two_seconds.pose.position.y, at_one_second.pose.position.y, 1e-9);
    EXPECT_NEAR(angle_error(yaw_of(at_two_seconds), yaw_of(at_one_second)), 0.0, 1e-9);
    EXPECT_NEAR(angle_error(yaw_of(at_two_seconds), truth.yaw + truth.omega * config.max_coast_s),
                0.0, 0.1);
}

TEST(KalmanMotionEstimatorTest, OpponentSigmasShrinkWhenMeasuredAndGrowWhileCoasting) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    TurningTruth initial;
    auto [t, truth] = run_turning(estimator, assigner, field, context, 2, initial);
    const PoseUncertainty early = (*estimator.coast(t))[0].uncertainty;
    TurningTruth rest = truth;
    for (int i = 0; i < 40; ++i) {
        t += kDt;
        rest.step(kDt);
        estimator.predict(t, CommandFeedback{});
        estimator.update({make_opponent_pose_measurement(rest.x, rest.y, rest.yaw)}, t, assigner,
                         field, context);
    }
    const PoseUncertainty converged = (*estimator.coast(t))[0].uncertainty;
    EXPECT_LT(*converged.position_sigma_major_axis_m, *early.position_sigma_major_axis_m);
    EXPECT_LT(*converged.heading_sigma_rad, *early.heading_sigma_rad);
    EXPECT_LT(*converged.yaw_rate_sigma_rad_s, *early.yaw_rate_sigma_rad_s);

    const PoseUncertainty coasting = (*estimator.coast(t + 0.15))[0].uncertainty;
    EXPECT_GT(*coasting.position_sigma_major_axis_m, *converged.position_sigma_major_axis_m);
    EXPECT_GT(*coasting.heading_sigma_rad, *converged.heading_sigma_rad);
    EXPECT_GT(*coasting.yaw_rate_sigma_rad_s, *converged.yaw_rate_sigma_rad_s);
}

TEST(KalmanMotionEstimatorTest, OpponentCoastingIntoWallRendersStopped) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    FieldDescription field = make_field();
    field.size.size.x = 2.0;
    field.size.size.y = 2.0;
    const MotionEstimatorContext context;

    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 15, 0.2, 0.0, 1.5, 0.3);
    auto coasted = estimator.coast(last_t + 0.15);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    EXPECT_DOUBLE_EQ((*coasted)[0].pose.position.x, 1.0);
    EXPECT_EQ((*coasted)[0].velocity.vx_mps, 0.0) << "no velocity into the wall";
    EXPECT_NEAR((*coasted)[0].velocity.vy_mps, 0.3, 0.1) << "velocity along the wall survives";
}

TEST(KalmanMotionEstimatorTest, OpponentArmMatchesPythonMirrorGoldenFixture) {
    namespace golden = opponent_filter_golden;
    constexpr double kTol = 1e-6;
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.max_coast_s = golden::kMaxCoastS;
    KalmanMotionEstimator estimator{config};
    ASSERT_NEAR(estimator.render_lead_s(), golden::kRenderLeadS, 1e-12);
    FrameIdAssigner assigner(10.0, 5);
    FieldDescription field = make_field();
    field.size.size.x = golden::kFieldSize;
    field.size.size.y = golden::kFieldSize;
    MotionEstimatorContext context;
    context.field_bounds_margin_meters = golden::kMargin;

    const auto expect_value = [&](double actual, double expected, const std::string &what) {
        if (std::isinf(expected)) {
            EXPECT_TRUE(std::isinf(actual)) << what;
        } else {
            EXPECT_NEAR(actual, expected, kTol) << what;
        }
    };

    int record = 0;
    int state_record = 0;
    for (int i = 0; i < golden::kNumEvents; ++i) {
        const int kind = golden::kEventKind[i];
        const double time = golden::kEventTime[i];
        std::vector<RobotDescription> outputs;
        if (kind == golden::kKindCoast) {
            outputs = *estimator.coast(time);
        } else {
            std::vector<RobotDescription> measurements;
            if (kind == golden::kKindMeasured) {
                measurements.push_back(make_opponent_pose_measurement(
                    golden::kMeasX[i], golden::kMeasY[i], golden::kMeasYaw[i]));
            }
            outputs = estimator.update(measurements, time, assigner, field, context);
        }
        if (!golden::kRecorded[i]) continue;
        ASSERT_EQ(outputs.size(), 1u) << "event " << i;
        ASSERT_LT(record, golden::kNumRecorded);
        const RobotDescription &out = outputs[0];
        const double *row = &golden::kRender[record * golden::kRenderWidth];
        const std::string at = "event " + std::to_string(i) + " field ";
        expect_value(out.pose.position.x, row[0], at + "x");
        expect_value(out.pose.position.y, row[1], at + "y");
        EXPECT_NEAR(angle_error(yaw_of(out), row[2]), 0.0, kTol) << at << "yaw";
        expect_value(out.velocity.vx_mps, row[3], at + "vx");
        expect_value(out.velocity.vy_mps, row[4], at + "vy");
        expect_value(out.velocity.omega_radps, row[5], at + "omega");
        expect_value(out.uncertainty.position_sigma_major_axis_m.value_or(
                         std::numeric_limits<double>::infinity()),
                     row[6], at + "position_sigma_major_axis_m");
        expect_value(
            out.uncertainty.heading_sigma_rad.value_or(std::numeric_limits<double>::infinity()),
            row[7], at + "heading_sigma_rad");
        expect_value(
            out.uncertainty.yaw_rate_sigma_rad_s.value_or(std::numeric_limits<double>::infinity()),
            row[8], at + "yaw_rate_sigma_rad_s");
        ++record;

        if (kind == golden::kKindCoast) continue;
        const auto state = estimator.opponent_filter_state(FrameId::THEIR_ROBOT_1);
        ASSERT_TRUE(state.has_value());
        const double *expected = &golden::kState[state_record * golden::kStateWidth];
        for (int k = 0; k < 6; ++k) {
            if (k == 4) {
                EXPECT_NEAR(angle_error(state->state(k), expected[k]), 0.0, kTol)
                    << "event " << i << " theta";
            } else {
                EXPECT_NEAR(state->state(k), expected[k], kTol) << "event " << i << " state " << k;
            }
        }
        int column = 6;
        for (int r = 0; r < 6; ++r) {
            for (int c = r; c < 6; ++c) {
                EXPECT_NEAR(state->covariance(r, c), expected[column++], kTol)
                    << "event " << i << " P(" << r << "," << c << ")";
            }
        }
        EXPECT_NEAR(state->stamp, expected[column++], 1e-9) << "event " << i << " stamp";
        EXPECT_EQ(state->heading_initialized ? 1.0 : 0.0, expected[column]) << "event " << i;
        ++state_record;
    }
    EXPECT_EQ(record, golden::kNumRecorded);
    EXPECT_EQ(state_record, golden::kNumStateRecords);
}

TEST(KalmanMotionEstimatorTest, ResetClearsTracks) {
    KalmanMotionEstimator estimator{default_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    estimator.update({make_opponent_measurement(1.0, 1.0)}, kStartTime, assigner, field, context);
    estimator.reset();
    auto outputs = estimator.update({}, kStartTime + kDt, assigner, field, context);
    EXPECT_TRUE(outputs.empty());
}

double position_sigma_after_coasting(KalmanMotionEstimator &estimator, double now) {
    const auto coasted = estimator.coast(now);
    EXPECT_TRUE(coasted.has_value());
    return *(*coasted)[0].uncertainty.position_sigma_major_axis_m;
}

TEST(KalmanMotionEstimatorTest, OpponentPositionSigmaKeepsGrowingPastTheCoastHorizon) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, 0.5, 0.0);
    const double measured_sigma = position_sigma_after_coasting(estimator, last_t);
    const double held_sigma = position_sigma_after_coasting(estimator, last_t + 3.0);
    EXPECT_GT(held_sigma, 3.0 * measured_sigma);
}

TEST(KalmanMotionEstimatorTest, OpponentPositionSigmaOfALongUnseenTrackGrowsNoFasterThanLinearly) {
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.max_position_sigma_m = std::numeric_limits<double>::infinity();
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, 0.5, 0.0);
    double t = last_t;
    for (int i = 0; i < 600; ++i) {
        t += 0.5;
        estimator.predict(t, CommandFeedback{});
        estimator.update({}, t, assigner, field, context);
    }
    const double sigma = position_sigma_after_coasting(estimator, t);
    EXPECT_LT(sigma, 2.0 * config.initial_velocity_sigma * (t - last_t));
}

TEST(KalmanMotionEstimatorTest, OpponentPositionFixSurvivesAKeypointHeadingOff60Degrees) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    TurningTruth initial;
    initial.omega = 0.0;
    initial.vx = 0.0;
    initial.vy = 0.0;
    const auto [last_t, truth] = run_turning(estimator, assigner, field, context, 90, initial);
    const double t = last_t + kDt;
    estimator.predict(t, CommandFeedback{});
    const auto outputs = estimator.update(
        {make_opponent_pose_measurement(truth.x + 0.05, truth.y, truth.yaw + M_PI / 3.0)}, t,
        assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_GT(outputs[0].pose.position.x, truth.x + 0.01);
}

TEST(KalmanMotionEstimatorTest, OurRobotSpeedProjectsOntoTheMidBaselineHeading) {
    KalmanMotionEstimatorConfiguration config = ekf_config();
    config.keypoint_heading_sigma_rad = 2.0;
    config.speed_sigma_floor_mps = 0.01;
    config.speed_position_sigma_m = 0.001;
    config.speed_gate_nis = 1e9;
    config.gate_nis = 1e9;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    const double speed = 1.0;
    const double omega = 10.0;
    const double dt = 0.1;
    const double turned = omega * dt;
    const double chord = 2.0 * speed / omega * std::sin(turned / 2.0);

    PlantState start;
    estimator.predict(kStartTime, CommandFeedback{});
    estimator.update({make_our_pose_measurement(start)}, kStartTime, assigner, field, context);

    PlantState end;
    end.theta = turned;
    end.x = chord * std::cos(turned / 2.0);
    end.y = chord * std::sin(turned / 2.0);
    estimator.predict(kStartTime + dt, CommandFeedback{});
    const auto outputs = estimator.update({make_our_pose_measurement(end)}, kStartTime + dt,
                                          assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    const double measured_speed =
        std::hypot(outputs[0].velocity.vx_mps, outputs[0].velocity.vy_mps);
    EXPECT_NEAR(measured_speed, chord / dt, 0.03);
}

TEST(KalmanMotionEstimatorTest, NoisySpeedDoesNotRejectPosition) {
    KalmanMotionEstimatorConfiguration config = ekf_config();
    config.keypoint_heading_sigma_rad = 2.0;
    config.gate_nis = 1e9;
    config.speed_gate_nis = 1e-12;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    PlantState start;
    estimator.predict(kStartTime, CommandFeedback{});
    estimator.update({make_our_pose_measurement(start)}, kStartTime, assigner, field, context);

    PlantState jumped;
    jumped.x = 0.3;
    estimator.predict(kStartTime + 0.1, CommandFeedback{});
    const auto outputs = estimator.update({make_our_pose_measurement(jumped)}, kStartTime + 0.1,
                                          assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    EXPECT_GT(outputs[0].pose.position.x, 0.05);
}

TEST(KalmanMotionEstimatorTest, RenderLeadEqualsPlantDelay) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    EXPECT_DOUBLE_EQ(estimator.render_lead_s(), kCoastLeadS);
}

TEST(KalmanMotionEstimatorTest, ConstructionWithoutPlantThrows) {
    KalmanMotionEstimatorConfiguration config;
    EXPECT_THROW(KalmanMotionEstimator{config}, std::invalid_argument);
}

TEST(KalmanMotionEstimatorTest, TooOldMeasurementFoldsInAtCurrentState) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const ConstantVelocityTrack track =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, 0.0, 0.0);
    const double x = track.x;
    const double y = track.y;
    const double before = estimator.opponent_filter_state(FrameId::THEIR_ROBOT_1)->state(0);
    ASSERT_NEAR(before, x, 1e-3);

    const double older_than_every_snapshot = kStartTime - 10.0;
    auto outputs = estimator.update({make_opponent_measurement(x + 0.05, y)},
                                    older_than_every_snapshot, assigner, field, context);
    ASSERT_EQ(outputs.size(), 1u);
    const double after = estimator.opponent_filter_state(FrameId::THEIR_ROBOT_1)->state(0);
    EXPECT_GT(after, before + 0.005) << "the measurement was dropped instead of folded in";
    EXPECT_LT(after, x + 0.05);
}

size_t count_clock_gap_warnings(KalmanMotionEstimator &estimator, const std::vector<double> &nows) {
    std::ostringstream captured;
    auto logger = std::make_shared<spdlog::logger>(
        "kalman_clock_gap_test", std::make_shared<spdlog::sinks::ostream_sink_mt>(captured));
    const auto previous = spdlog::default_logger();
    spdlog::set_default_logger(logger);
    for (double now : nows) estimator.coast(now);
    spdlog::set_default_logger(previous);
    const std::string log = captured.str();
    size_t count = 0;
    for (size_t at = log.find("Coast span"); at != std::string::npos;
         at = log.find("Coast span", at + 1)) {
        ++count;
    }
    return count;
}

TEST(KalmanMotionEstimatorTest, LargeClockGapMovesNoFurtherThanMaxCoast) {
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.max_coast_s = 0.2;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;

    const double vx = 1.5;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, vx, 0.0);

    auto coasted = estimator.coast(last_t + 5.0);
    ASSERT_TRUE(coasted.has_value());
    ASSERT_EQ(coasted->size(), 1u);
    EXPECT_NEAR((*coasted)[0].pose.position.x, x + vx * config.max_coast_s, 0.05)
        << "a 5 s clock gap extrapolates no further than max_coast_s";
}

TEST(KalmanMotionEstimatorTest, OpponentCoastLeavesTheFilterStateAlone) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, 1.5, 0.0);
    const auto before = estimator.opponent_filter_state(FrameId::THEIR_ROBOT_1);
    ASSERT_TRUE(before.has_value());

    const auto ahead = estimator.coast(last_t + 0.02);
    const auto behind = estimator.coast(last_t + 0.01);
    ASSERT_TRUE(ahead.has_value());
    ASSERT_TRUE(behind.has_value());

    const auto after = estimator.opponent_filter_state(FrameId::THEIR_ROBOT_1);
    ASSERT_TRUE(after.has_value());
    EXPECT_EQ(after->stamp, before->stamp);
    EXPECT_EQ(after->state, before->state);
    EXPECT_EQ(after->covariance, before->covariance);
    EXPECT_LT((*behind)[0].pose.position.x, (*ahead)[0].pose.position.x)
        << "a coast behind the previous one renders behind it";
}

TEST(KalmanMotionEstimatorTest, ClockSteppingBackBetweenCoastsDoesNotWarn) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    const auto [last_t, x, y] =
        run_constant_velocity(estimator, assigner, field, context, 60, 1.0, 2.0, 1.5, 0.0);

    EXPECT_EQ(count_clock_gap_warnings(estimator, {last_t + 0.04, last_t + 0.032, last_t + 0.036}),
              0u);
}

TEST(KalmanMotionEstimatorTest, FirstClockGapWarningFiresAtEpochZero) {
    KalmanMotionEstimator estimator{kalman_opponents_config()};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    estimator.predict(0.0, CommandFeedback{});
    estimator.update({make_opponent_measurement(1.0, 2.0)}, 0.0, assigner, field, context);

    EXPECT_EQ(count_clock_gap_warnings(estimator, {2.0}), 1u);
}

TEST(KalmanMotionEstimatorTest, ClockGapWarningIsThrottledByConfiguredPeriod) {
    KalmanMotionEstimatorConfiguration config = kalman_opponents_config();
    config.clock_gap_warn_period_s = 3.0;
    KalmanMotionEstimator estimator{config};
    FrameIdAssigner assigner(10.0, 5);
    const FieldDescription field = make_field();
    const MotionEstimatorContext context;
    estimator.predict(0.0, CommandFeedback{});
    estimator.update({make_opponent_measurement(1.0, 2.0)}, 0.0, assigner, field, context);

    EXPECT_EQ(count_clock_gap_warnings(estimator, {2.0, 4.0, 7.0}), 2u);
}

}  // namespace
}  // namespace auto_battlebot
