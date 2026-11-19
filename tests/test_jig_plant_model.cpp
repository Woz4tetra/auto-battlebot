#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "plant/jig_plant_model.hpp"
#include "plant/mrs_buff_mk3_params.hpp"
#include "plant/plant_golden_data.hpp"
#include "robot_filter/command_ring_buffer.hpp"

namespace auto_battlebot {
namespace {

// Both sides run identical double arithmetic at the same 2 ms substep; the only drift is
// last-ulp libm differences accumulating over 1200 steps, well under this.
constexpr double kGoldenTol = 1e-9;
constexpr double kDesignHorizonS = 0.4;

double wrap_angle(double angle) {
    double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped - M_PI;
}

JigPlantParams golden_params() { return mrs_buff_mk3_plant(); }

std::vector<TimedCommand> golden_commands() {
    std::vector<TimedCommand> commands;
    commands.reserve(plant_golden::kNumCommands);
    for (int i = 0; i < plant_golden::kNumCommands; ++i) {
        commands.push_back({plant_golden::kCmdStamp[i],
                            {plant_golden::kCmdLin[i], 0.0, plant_golden::kCmdAng[i]}});
    }
    return commands;
}

void expect_matches_record(const PlantState &state, int record, const char *label) {
    EXPECT_NEAR(state.x, plant_golden::kExpectedX[record], kGoldenTol) << label << record;
    EXPECT_NEAR(state.y, plant_golden::kExpectedY[record], kGoldenTol) << label << record;
    EXPECT_NEAR(wrap_angle(state.theta - plant_golden::kExpectedTheta[record]), 0.0, kGoldenTol)
        << label << record;
    EXPECT_NEAR(state.v, plant_golden::kExpectedV[record], kGoldenTol) << label << record;
    EXPECT_NEAR(state.w, plant_golden::kExpectedW[record], kGoldenTol) << label << record;
}

TEST(JigPlantModelTest, GoldenTickByTickMatchesPythonReference) {
    JigPlantModel model(golden_params());
    const std::vector<TimedCommand> commands = golden_commands();
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    for (int step = 1; step <= plant_golden::kSteps; ++step) {
        const double t0 = static_cast<double>(step - 1) * plant_golden::kDt;
        const double t1 = static_cast<double>(step) * plant_golden::kDt;
        state = model.propagate(state, commands, t0, t1, jacobian);
        if (step % plant_golden::kRecordStride == 0) {
            const int record = step / plant_golden::kRecordStride - 1;
            expect_matches_record(state, record, "tick record ");
        }
    }
}

TEST(JigPlantModelTest, GoldenChunkedPropagateMatchesPythonReference) {
    JigPlantModel model(golden_params());
    const std::vector<TimedCommand> commands = golden_commands();
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    constexpr double kEkfFrameGapS = 0.1;
    constexpr int kChunkSteps = static_cast<int>(kEkfFrameGapS / plant_golden::kDt + 0.5);
    for (int step = kChunkSteps; step <= plant_golden::kSteps; step += kChunkSteps) {
        const double t0 = static_cast<double>(step - kChunkSteps) * plant_golden::kDt;
        const double t1 = static_cast<double>(step) * plant_golden::kDt;
        state = model.propagate(state, commands, t0, t1, jacobian);
        const int record = step / plant_golden::kRecordStride - 1;
        expect_matches_record(state, record, "chunk record ");
    }
}

TEST(JigPlantModelTest, GatheredRingBufferSpanMatchesFullHistory) {
    JigPlantModel model(golden_params());
    const std::vector<TimedCommand> commands = golden_commands();
    CommandRingBuffer buffer;
    for (const TimedCommand &command : commands) {
        buffer.push(command);
    }
    const double t0 = 1.0;
    const double t1 = 1.1;
    std::vector<TimedCommand> gathered;
    buffer.gather(t0 - model.params().delay_s, t1, gathered);

    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.v = 1.0;
    state.w = 2.0;
    const PlantState from_full = model.propagate(state, commands, t0, t1, jacobian);
    const PlantState from_gathered = model.propagate(state, gathered, t0, t1, jacobian);
    EXPECT_DOUBLE_EQ(from_full.x, from_gathered.x);
    EXPECT_DOUBLE_EQ(from_full.y, from_gathered.y);
    EXPECT_DOUBLE_EQ(from_full.theta, from_gathered.theta);
    EXPECT_DOUBLE_EQ(from_full.v, from_gathered.v);
    EXPECT_DOUBLE_EQ(from_full.w, from_gathered.w);
}

TEST(JigPlantModelTest, ZeroSpanReturnsStateAndIdentityJacobian) {
    JigPlantModel model(golden_params());
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.x = 1.0;
    state.theta = 2.0;
    state.v = -0.5;
    const PlantState out = model.propagate(state, {}, 3.0, 3.0, jacobian);
    EXPECT_DOUBLE_EQ(out.x, state.x);
    EXPECT_DOUBLE_EQ(out.v, state.v);
    EXPECT_TRUE(jacobian.isApprox(Eigen::Matrix<double, 5, 5>::Identity(), 1e-12));
}

TEST(JigPlantModelTest, JacobianSingleStepStraightLine) {
    JigPlantModel model(golden_params());
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.v = 1.0;
    const double dt = 0.002;
    model.propagate(state, {}, 0.0, dt, jacobian);

    EXPECT_NEAR(jacobian(0, 3), dt, 1e-6);
    EXPECT_NEAR(jacobian(1, 3), 0.0, 1e-6);
    EXPECT_NEAR(jacobian(2, 4), dt, 1e-6);
    EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(jacobian(1, 1), 1.0, 1e-9);
    EXPECT_NEAR(jacobian(2, 2), 1.0, 1e-9);
    EXPECT_NEAR(jacobian(3, 3), std::exp(-dt / model.params().tau_lin_d), 1e-6);
}

TEST(JigPlantModelTest, JacobianHeadingRotatesVelocityIntoCross) {
    JigPlantModel model(golden_params());
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.theta = M_PI / 2.0;
    state.v = 1.0;
    const double dt = 0.002;
    model.propagate(state, {}, 0.0, dt, jacobian);
    EXPECT_NEAR(jacobian(1, 3), dt, 1e-6);
    EXPECT_NEAR(jacobian(0, 3), 0.0, 1e-6);
    EXPECT_NEAR(jacobian(0, 2), -state.v * dt * std::sin(state.theta), 1e-6);
}

TEST(JigPlantModelTest, TransportDelayGatesCommandOnset) {
    JigPlantModel model(golden_params());
    constexpr double kInsideDelayS = 0.05;
    ASSERT_GT(model.params().delay_s, kInsideDelayS);
    const std::vector<TimedCommand> commands = {{0.0, {1.0, 0.0, 0.0}}};
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state = model.propagate(state, commands, 0.0, kInsideDelayS, jacobian);
    EXPECT_DOUBLE_EQ(state.v, 0.0);
    EXPECT_DOUBLE_EQ(state.x, 0.0);
    state = model.propagate(state, commands, kInsideDelayS, kInsideDelayS + 0.1, jacobian);
    EXPECT_GT(state.v, 0.5);
    EXPECT_GT(state.x, 0.0);
}

TEST(JigPlantModelTest, PropagateWrapsThetaToPlusMinusPi) {
    JigPlantModel model(golden_params());
    const std::vector<TimedCommand> commands = {{0.0, {0.0, 0.0, 1.0}}};
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.theta = 3.0;
    for (int step = 0; step < 40; ++step) {
        const double t0 = 0.05 * step;
        state = model.propagate(state, commands, t0, t0 + 0.05, jacobian);
        EXPECT_GT(state.theta, -M_PI);
        EXPECT_LE(state.theta, M_PI);
    }
}

TEST(JigPlantModelTest, CommandEdgeInsideSpanAppliesAtItsStamp) {
    JigPlantParams params = golden_params();
    params.delay_s = 0.1;
    JigPlantModel model(params);
    const std::vector<TimedCommand> commands = {{0.0, {0.0, 0.0, 0.0}}, {0.5, {1.0, 0.0, 0.0}}};
    Eigen::Matrix<double, 5, 5> jacobian;

    const PlantState one_span = model.propagate(PlantState{}, commands, 0.0, 1.0, jacobian);
    PlantState chunked = model.propagate(PlantState{}, commands, 0.0, 0.5, jacobian);
    chunked = model.propagate(chunked, commands, 0.5, 1.0, jacobian);

    EXPECT_GT(one_span.x, 0.0) << "command edge inside the span was ignored";
    EXPECT_NEAR(one_span.x, chunked.x, 1e-9);
}

TEST(JigPlantModelTest, JacobianMatchesAnalyticSlopeNearUnitState) {
    JigPlantModel model(golden_params());
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.x = 1.3;
    model.propagate(state, {}, 0.0, 0.05, jacobian);
    EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-12);
}

TEST(JigPlantModelTest, JacobianThetaColumnFiniteNearPi) {
    JigPlantModel model(golden_params());
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state.theta = M_PI - 1e-7;
    model.propagate(state, {}, 0.0, 0.05, jacobian);
    EXPECT_NEAR(jacobian(2, 2), 1.0, 1e-6);
}

TEST(JigPlantModelTest, ZeroCommandBeforeFirstEntry) {
    JigPlantModel model(golden_params());
    const std::vector<TimedCommand> commands = {{1.0, {1.0, 0.0, 0.0}}};
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state = model.propagate(state, commands, 0.0, 1.0, jacobian);
    EXPECT_DOUBLE_EQ(state.v, 0.0);
    EXPECT_DOUBLE_EQ(state.x, 0.0);
}

TEST(JigPlantModelTest, EffectiveCommandDeadzoneAndRescale) {
    const JigPlantParams params = golden_params();
    EXPECT_DOUBLE_EQ(plant_effective_command(0.0, params.dz_lin_fwd, params.dz_lin_rev), 0.0);
    EXPECT_DOUBLE_EQ(
        plant_effective_command(params.dz_lin_fwd * 0.9, params.dz_lin_fwd, params.dz_lin_rev),
        0.0);
    constexpr double kClippedDeadzone = 0.95;
    EXPECT_DOUBLE_EQ(plant_effective_command(0.97, 0.99, 0.99),
                     (0.97 - kClippedDeadzone) / (1.0 - kClippedDeadzone));
    EXPECT_DOUBLE_EQ(plant_effective_command(1.0, params.dz_lin_fwd, params.dz_lin_rev), 1.0);
    EXPECT_DOUBLE_EQ(plant_effective_command(-1.0, params.dz_lin_fwd, params.dz_lin_rev), -1.0);
    const double expected = (0.5 - params.dz_lin_fwd) / (1.0 - params.dz_lin_fwd);
    EXPECT_DOUBLE_EQ(plant_effective_command(0.5, params.dz_lin_fwd, params.dz_lin_rev), expected);
    ASSERT_GT(params.dz_lin_rev, params.dz_lin_fwd);
    EXPECT_LT(std::abs(plant_effective_command(-0.5, params.dz_lin_fwd, params.dz_lin_rev)),
              expected);
}

TEST(JigPlantModelTest, SteadyStateSteerBrakeClampsToZero) {
    const JigPlantParams params = golden_params();
    ASSERT_GT(params.c_sb, 2.0);
    double v_target = 0.0;
    double w_target = 0.0;
    plant_steady_state(0.5, 0.5, params, v_target, w_target);
    EXPECT_DOUBLE_EQ(v_target, 0.0);
    EXPECT_GT(w_target, 1.0);
    plant_steady_state(0.5, 0.0, params, v_target, w_target);
    const double lin_eff = plant_effective_command(0.5, params.dz_lin_fwd, params.dz_lin_rev);
    EXPECT_DOUBLE_EQ(v_target, params.k_fwd * lin_eff);
}

TEST(JigPlantModelTest, ProcessNoiseRotatesPositionBlockWithHeading) {
    JigPlantNoiseParams noise;
    JigPlantModel model(golden_params(), noise);
    const double dt = 0.1;
    PlantState state;
    state.v = 1.5;
    state.w = 2.0;
    const double sf_v = noise.scale_factor * state.v;
    const double sf_w = noise.heading_scale_factor * state.w;
    const double jitter = noise.delay_jitter_s * state.v;
    const double p_along = noise.q_along * dt * dt / 3.0 + sf_v * sf_v * kDesignHorizonS +
                           jitter * jitter / kDesignHorizonS;
    const double p_cross = noise.q_cross * kDesignHorizonS * kDesignHorizonS / 3.0;
    const double p_theta =
        noise.heading_random_walk + sf_w * sf_w * kDesignHorizonS + noise.q_heading * dt * dt / 3.0;

    Eigen::Matrix<double, 5, 5> q_forward = model.process_noise(state, {}, dt);
    EXPECT_DOUBLE_EQ(q_forward(0, 0), p_along * dt);
    EXPECT_DOUBLE_EQ(q_forward(1, 1), p_cross * dt);
    EXPECT_DOUBLE_EQ(q_forward(2, 2), p_theta * dt);
    EXPECT_DOUBLE_EQ(q_forward(3, 3), noise.q_along * dt);
    EXPECT_DOUBLE_EQ(q_forward(4, 4), noise.q_heading * dt);
    EXPECT_DOUBLE_EQ(q_forward(0, 3), noise.q_along * dt * dt / 2.0);
    EXPECT_DOUBLE_EQ(q_forward(1, 3), 0.0);

    state.theta = M_PI / 2.0;
    Eigen::Matrix<double, 5, 5> q_sideways = model.process_noise(state, {}, dt);
    EXPECT_NEAR(q_sideways(0, 0), p_cross * dt, 1e-12);
    EXPECT_NEAR(q_sideways(1, 1), p_along * dt, 1e-12);
    EXPECT_NEAR(q_sideways(1, 3), noise.q_along * dt * dt / 2.0, 1e-12);
    EXPECT_NEAR(q_sideways(0, 3), 0.0, 1e-12);
    EXPECT_TRUE(q_sideways.isApprox(q_sideways.transpose(), 1e-12));
}

TEST(JigPlantModelTest, ProcessNoiseMatchesGrowthLawAtDesignHorizon) {
    const JigPlantParams params = golden_params();
    JigPlantModel model(params, JigPlantNoiseParams{});

    const double u_lin = 0.10983;
    const double u_ang = 0.04294;
    PlantState state;
    plant_steady_state(u_lin, u_ang, params, state.v, state.w);
    constexpr double kMedianSpeedMps = 0.47;
    constexpr double kMedianYawRateRadps = 0.83;
    ASSERT_NEAR(state.v, kMedianSpeedMps, 0.02);
    ASSERT_NEAR(state.w, kMedianYawRateRadps, 0.03);
    const std::vector<TimedCommand> commands{{-1.0, {u_lin, 0.0, u_ang}}};

    Eigen::Matrix<double, 5, 5> covariance = Eigen::Matrix<double, 5, 5>::Zero();
    Eigen::Matrix<double, 5, 5> jacobian;
    const double dt = 0.025;
    const int steps = static_cast<int>(kDesignHorizonS / dt + 0.5);
    for (int step = 0; step < steps; ++step) {
        const double t0 = static_cast<double>(step) * dt;
        state = model.propagate(state, commands, t0, t0 + dt, jacobian);
        covariance =
            jacobian * covariance * jacobian.transpose() + model.process_noise(state, commands, dt);
    }

    const double sigma_along = std::sqrt(covariance(0, 0));
    const double sigma_cross = std::sqrt(covariance(1, 1));
    const double sigma_heading = std::sqrt(covariance(2, 2));
    constexpr double kFittedAlongM = 0.135;
    constexpr double kFittedCrossM = 0.0866;
    constexpr double kFittedHeadingRad = 0.613;
    constexpr double kBandLow = 0.6;
    constexpr double kBandHigh = 1.5;
    EXPECT_GT(sigma_along, kBandLow * kFittedAlongM);
    EXPECT_LT(sigma_along, kBandHigh * kFittedAlongM);
    EXPECT_GT(sigma_cross, kBandLow * kFittedCrossM);
    EXPECT_LT(sigma_cross, kBandHigh * kFittedCrossM);
    EXPECT_GT(sigma_heading, kBandLow * kFittedHeadingRad);
    EXPECT_LT(sigma_heading, kBandHigh * kFittedHeadingRad);
}

TEST(PlantStickToBodyVelocityTest, AppliesGainsPerDirection) {
    JigPlantParams params;
    params.k_fwd = 4.0;
    params.k_rev = 2.0;
    params.k_ang = 10.0;

    const VelocityCommand forward = plant_stick_to_body_velocity({0.5, 0.0, 0.0}, params);
    EXPECT_NEAR(forward.forward_mps, 2.0, 1e-12);
    EXPECT_NEAR(forward.yaw_rate_radps, 0.0, 1e-12);

    const VelocityCommand reverse = plant_stick_to_body_velocity({-0.5, 0.0, 0.0}, params);
    EXPECT_NEAR(reverse.forward_mps, -1.0, 1e-12);

    const VelocityCommand yaw = plant_stick_to_body_velocity({0.0, 0.0, -0.25}, params);
    EXPECT_NEAR(yaw.yaw_rate_radps, -2.5, 1e-12);

    EXPECT_NEAR(plant_stick_to_body_velocity({0.5, 0.7, 0.0}, params).lateral_mps, 0.0, 1e-12);
}

TEST(PlantStickToBodyVelocityTest, IgnoresDeadzones) {
    JigPlantParams params = mrs_buff_mk3_plant();
    const double inside_deadzone = params.dz_lin_fwd * 0.5;
    ASSERT_GT(inside_deadzone, 0.0);

    const VelocityCommand converted =
        plant_stick_to_body_velocity({inside_deadzone, 0.0, 0.0}, params);
    EXPECT_NEAR(converted.forward_mps, params.k_fwd * inside_deadzone, 1e-12);

    double v_target = 0.0;
    double w_target = 0.0;
    plant_steady_state(inside_deadzone, 0.0, params, v_target, w_target);
    EXPECT_NEAR(v_target, 0.0, 1e-12);
}

}  // namespace
}  // namespace auto_battlebot
