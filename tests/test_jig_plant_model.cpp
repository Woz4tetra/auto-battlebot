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
    // The fixture's theta is unwrapped while propagate wraps, so compare the wrapped
    // difference. The dynamics only see theta through sin/cos, making the trajectories
    // identical either way.
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
    // 100 ms per propagate call, the frame-gap scale the EKF will use. The command is
    // re-read every substep, so edges landing inside a chunk take effect at their own time
    // and the result matches the tick-by-tick reference exactly.
    JigPlantModel model(golden_params());
    const std::vector<TimedCommand> commands = golden_commands();
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    constexpr int kChunkSteps = 50;
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

    // Pose integrates the start-of-substep velocity, so over a single substep these are
    // exact: dx/dv = dt at theta 0, dtheta/dw = dt, and the pose rows are identity in
    // themselves.
    EXPECT_NEAR(jacobian(0, 3), dt, 1e-6);
    EXPECT_NEAR(jacobian(1, 3), 0.0, 1e-6);
    EXPECT_NEAR(jacobian(2, 4), dt, 1e-6);
    EXPECT_NEAR(jacobian(0, 0), 1.0, 1e-9);
    EXPECT_NEAR(jacobian(1, 1), 1.0, 1e-9);
    EXPECT_NEAR(jacobian(2, 2), 1.0, 1e-9);
    // No command: the target is zero, both perturbations decelerate, so dv'/dv is the
    // decel lag factor.
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
    // dx/dtheta = -v * dt * sin(theta) = -dt at pi/2.
    EXPECT_NEAR(jacobian(0, 2), -state.v * dt, 1e-6);
}

TEST(JigPlantModelTest, TransportDelayGatesCommandOnset) {
    JigPlantModel model(golden_params());
    ASSERT_GT(model.params().delay_s, 0.05);
    const std::vector<TimedCommand> commands = {{0.0, {1.0, 0.0, 0.0}}};
    Eigen::Matrix<double, 5, 5> jacobian;
    PlantState state;
    state = model.propagate(state, commands, 0.0, 0.05, jacobian);
    // Inside the transport delay the command has not arrived: no motion at all.
    EXPECT_DOUBLE_EQ(state.v, 0.0);
    EXPECT_DOUBLE_EQ(state.x, 0.0);
    state = model.propagate(state, commands, 0.05, 0.15, jacobian);
    EXPECT_GT(state.v, 0.5);
    EXPECT_GT(state.x, 0.0);
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
    // Full command maps to full effect: the rescale keeps the gain equal to the max speed.
    EXPECT_DOUBLE_EQ(plant_effective_command(1.0, params.dz_lin_fwd, params.dz_lin_rev), 1.0);
    EXPECT_DOUBLE_EQ(plant_effective_command(-1.0, params.dz_lin_fwd, params.dz_lin_rev), -1.0);
    const double expected = (0.5 - params.dz_lin_fwd) / (1.0 - params.dz_lin_fwd);
    EXPECT_DOUBLE_EQ(plant_effective_command(0.5, params.dz_lin_fwd, params.dz_lin_rev), expected);
    // The reverse side uses its own, larger deadzone.
    EXPECT_LT(std::abs(plant_effective_command(-0.5, params.dz_lin_fwd, params.dz_lin_rev)),
              expected);
}

TEST(JigPlantModelTest, SteadyStateSteerBrakeClampsToZero) {
    const JigPlantParams params = golden_params();
    ASSERT_GT(params.c_sb, 2.0);
    double v_target = 0.0;
    double w_target = 0.0;
    // At c_sb 2.70 the factor crosses zero near |u_ang_eff| 0.37; a half-stick turn is far
    // past it, so the forward target must clamp to zero, not go negative.
    plant_steady_state(0.5, 0.5, params, v_target, w_target);
    EXPECT_DOUBLE_EQ(v_target, 0.0);
    EXPECT_GT(w_target, 1.0);
    // A pure forward command keeps its full gain.
    plant_steady_state(0.5, 0.0, params, v_target, w_target);
    const double lin_eff = plant_effective_command(0.5, params.dz_lin_fwd, params.dz_lin_rev);
    EXPECT_DOUBLE_EQ(v_target, params.k_fwd * lin_eff);
}

TEST(JigPlantModelTest, ProcessNoiseRotatesPositionBlockWithHeading) {
    JigPlantNoiseParams noise;
    JigPlantModel model(golden_params(), noise);
    const double dt = 0.1;
    constexpr double kDesignHorizonS = 0.4;
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

    // Hold the fit's median operating point (0.47 m/s, 0.83 rad/s): commands whose steady
    // state lands there, with the state started at that steady state so the window matches
    // the conditions the growth law was fit under.
    const double u_lin = 0.10983;
    const double u_ang = 0.04294;
    PlantState state;
    plant_steady_state(u_lin, u_ang, params, state.v, state.w);
    ASSERT_NEAR(state.v, 0.47, 0.02);
    ASSERT_NEAR(state.w, 0.83, 0.03);
    const std::vector<TimedCommand> commands{{-1.0, {u_lin, 0.0, u_ang}}};

    // EKF-style accumulation P = F P F^T + Q per frame over the 400 ms design horizon.
    Eigen::Matrix<double, 5, 5> covariance = Eigen::Matrix<double, 5, 5>::Zero();
    Eigen::Matrix<double, 5, 5> jacobian;
    const double dt = 0.025;
    const int steps = 16;
    for (int step = 0; step < steps; ++step) {
        const double t0 = static_cast<double>(step) * dt;
        state = model.propagate(state, commands, t0, t0 + dt, jacobian);
        covariance =
            jacobian * covariance * jacobian.transpose() + model.process_noise(state, commands, dt);
    }

    // Targets are the fitted growth law at 400 ms (fit_process_noise.py modelled row):
    // along 135 mm, cross 86.6 mm, heading 35.1 deg, decomposed at the start heading
    // (theta0 = 0, so field x is along-track). The band is wide on purpose: the plant's lag
    // damps the q h^3/3 terms below their undamped mapping, and heading noise reaches
    // cross-track through the Jacobian on top of the fitted cross budget. The test pins the
    // wiring (units, rotation, mechanism terms), not the second-order shape.
    const double sigma_along = std::sqrt(covariance(0, 0));
    const double sigma_cross = std::sqrt(covariance(1, 1));
    const double sigma_heading = std::sqrt(covariance(2, 2));
    EXPECT_GT(sigma_along, 0.6 * 0.135);
    EXPECT_LT(sigma_along, 1.5 * 0.135);
    EXPECT_GT(sigma_cross, 0.6 * 0.0866);
    EXPECT_LT(sigma_cross, 1.5 * 0.0866);
    EXPECT_GT(sigma_heading, 0.6 * 0.613);
    EXPECT_LT(sigma_heading, 1.5 * 0.613);
}

/** The dead-reckoning conversion is the gains and nothing else. It is deliberately not
 * plant_steady_state(): a single-frame hold carries no state, so deadzones and coupling would
 * model a response the caller cannot represent. */
TEST(PlantStickToBodyVelocityTest, AppliesGainsPerDirection) {
    JigPlantParams params;
    params.k_fwd = 4.0;
    params.k_rev = 2.0;
    params.k_ang = 10.0;

    const VelocityCommand forward = plant_stick_to_body_velocity({0.5, 0.0, 0.0}, params);
    EXPECT_NEAR(forward.linear_x, 2.0, 1e-12);
    EXPECT_NEAR(forward.angular_z, 0.0, 1e-12);

    const VelocityCommand reverse = plant_stick_to_body_velocity({-0.5, 0.0, 0.0}, params);
    EXPECT_NEAR(reverse.linear_x, -1.0, 1e-12);

    const VelocityCommand yaw = plant_stick_to_body_velocity({0.0, 0.0, -0.25}, params);
    EXPECT_NEAR(yaw.angular_z, -2.5, 1e-12);

    // A differential drive has no lateral axis, so linear_y stays zero whatever comes in.
    EXPECT_NEAR(plant_stick_to_body_velocity({0.5, 0.7, 0.0}, params).linear_y, 0.0, 1e-12);
}

/** No deadzone removal here, unlike plant_steady_state(): a stick inside the fitted deadzone
 * still produces motion. Pinned because the two converters must stay visibly different. */
TEST(PlantStickToBodyVelocityTest, IgnoresDeadzones) {
    JigPlantParams params = mrs_buff_mk3_plant();
    const double inside_deadzone = params.dz_lin_fwd * 0.5;
    ASSERT_GT(inside_deadzone, 0.0);

    const VelocityCommand converted =
        plant_stick_to_body_velocity({inside_deadzone, 0.0, 0.0}, params);
    EXPECT_NEAR(converted.linear_x, params.k_fwd * inside_deadzone, 1e-12);

    double v_target = 0.0;
    double w_target = 0.0;
    plant_steady_state(inside_deadzone, 0.0, params, v_target, w_target);
    EXPECT_NEAR(v_target, 0.0, 1e-12);
}

}  // namespace
}  // namespace auto_battlebot
