#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "robot_filter/ekf_math.hpp"

namespace auto_battlebot {
namespace {

constexpr double kFramePeriod = 1.0 / 30.0;
constexpr double kOddMultipleOfPiRoundingRad = 1e-12;

TEST(EkfMathTest, WrapAngleStaysInHalfOpenPiRange) {
    EXPECT_EQ(ekf::wrap_angle(-M_PI), M_PI);
    for (const double angle : {3.0 * M_PI, -3.0 * M_PI, 5.0 * M_PI, -5.0 * M_PI}) {
        const double wrapped = ekf::wrap_angle(angle);
        EXPECT_GT(wrapped, -M_PI) << angle;
        EXPECT_LE(wrapped, M_PI) << angle;
        EXPECT_NEAR(std::abs(wrapped), M_PI, kOddMultipleOfPiRoundingRad) << angle;
    }
    EXPECT_NEAR(ekf::wrap_angle(0.25), 0.25, 1e-12);
    EXPECT_NEAR(ekf::wrap_angle(M_PI + 0.1), -M_PI + 0.1, 1e-12);
    EXPECT_NEAR(ekf::wrap_angle(-M_PI - 0.1), M_PI - 0.1, 1e-12);
}

TEST(EkfMathTest, PropagateCovarianceGrowsAndStaysSymmetric) {
    ekf::Mat<2, 2> covariance;
    covariance << 0.01, 0.0, 0.0, 0.01;
    ekf::Mat<2, 2> transition;
    transition << 1.0, kFramePeriod, 0.0, 1.0;
    ekf::Mat<2, 2> process_noise;
    process_noise << 1e-4, 1e-5, 1e-5, 1e-3;

    const double initial_position_variance = covariance(0, 0);
    ekf::propagate_covariance<2>(covariance, transition, process_noise);

    EXPECT_GT(covariance(0, 0), initial_position_variance);
    EXPECT_DOUBLE_EQ(covariance(0, 1), covariance(1, 0));
}

TEST(EkfMathTest, UpdateReducesVarianceAndKeepsSymmetry) {
    constexpr double kPriorMean = 1.0;
    constexpr double kMeasured = 1.2;
    constexpr double kPriorVariance = 0.5;
    constexpr double kNoiseVariance = 0.01;
    ekf::Vec<2> state;
    state << kPriorMean, 0.5;
    ekf::Mat<2, 2> covariance = ekf::Mat<2, 2>::Identity() * kPriorVariance;
    ekf::Mat<1, 2> observation;
    observation << 1.0, 0.0;
    const ekf::Vec<1> measurement(kMeasured);
    const ekf::Vec<1> predicted(state(0));
    const ekf::Mat<1, 1> measurement_noise = ekf::Mat<1, 1>::Identity() * kNoiseVariance;

    const auto outcome =
        ekf::ekf_update<2, 1>(state, covariance, measurement, predicted, observation,
                              measurement_noise, 0.0, {false}, {false, false}, 1e-12);

    EXPECT_TRUE(outcome.accepted);
    EXPECT_LT(covariance(0, 0), 0.5);
    EXPECT_DOUBLE_EQ(covariance(0, 1), covariance(1, 0));
    const double gain = kPriorVariance / (kPriorVariance + kNoiseVariance);
    EXPECT_NEAR(state(0), kPriorMean + gain * (kMeasured - kPriorMean), 1e-9);
}

TEST(EkfMathTest, GateRejectsFarMeasurementAndLeavesStateUntouched) {
    ekf::Vec<2> state;
    state << 0.0, 0.0;
    ekf::Mat<2, 2> covariance = ekf::Mat<2, 2>::Identity() * 0.01;
    const ekf::Vec<2> state_before = state;
    const ekf::Mat<2, 2> covariance_before = covariance;

    const ekf::Mat<2, 2> observation = ekf::Mat<2, 2>::Identity();
    const ekf::Vec<2> measurement(5.0, 5.0);
    const ekf::Vec<2> predicted(0.0, 0.0);
    const ekf::Mat<2, 2> measurement_noise = ekf::Mat<2, 2>::Identity() * 0.01;

    const auto outcome =
        ekf::ekf_update<2, 2>(state, covariance, measurement, predicted, observation,
                              measurement_noise, 11.83, {false, false}, {false, false}, 1e-12);

    EXPECT_FALSE(outcome.accepted);
    EXPECT_GT(outcome.nis, 11.83);
    EXPECT_EQ(state, state_before);
    EXPECT_EQ(covariance, covariance_before);
}

TEST(EkfMathTest, DisabledGateAcceptsFarMeasurement) {
    ekf::Vec<2> state = ekf::Vec<2>::Zero();
    ekf::Mat<2, 2> covariance = ekf::Mat<2, 2>::Identity() * 0.01;
    const ekf::Mat<2, 2> observation = ekf::Mat<2, 2>::Identity();
    const ekf::Vec<2> measurement(5.0, 5.0);
    const ekf::Vec<2> predicted = ekf::Vec<2>::Zero();
    const ekf::Mat<2, 2> measurement_noise = ekf::Mat<2, 2>::Identity() * 0.01;

    const auto outcome =
        ekf::ekf_update<2, 2>(state, covariance, measurement, predicted, observation,
                              measurement_noise, 0.0, {false, false}, {false, false}, 1e-12);
    EXPECT_TRUE(outcome.accepted);
}

TEST(EkfMathTest, AngleInnovationWrapsAcrossPi) {
    constexpr double kHalfGapRad = 0.03;
    constexpr double kTrueErrorRad = 2.0 * kHalfGapRad;
    ekf::Vec<1> state(M_PI - kHalfGapRad);
    ekf::Mat<1, 1> covariance = ekf::Mat<1, 1>::Identity() * 0.1;
    const ekf::Mat<1, 1> observation = ekf::Mat<1, 1>::Identity();
    const ekf::Vec<1> measurement(-M_PI + kHalfGapRad);
    const ekf::Vec<1> predicted = state;
    const ekf::Mat<1, 1> measurement_noise = ekf::Mat<1, 1>::Identity() * 0.001;

    const auto outcome =
        ekf::ekf_update<1, 1>(state, covariance, measurement, predicted, observation,
                              measurement_noise, 11.83, {true}, {true}, 1e-12);

    EXPECT_TRUE(outcome.accepted);
    EXPECT_GT(std::abs(state(0)), M_PI - kTrueErrorRad) << "posterior on the pi side";
    EXPECT_LE(std::abs(state(0)), M_PI) << "posterior wrapped into (-pi, pi]";
    EXPECT_LT(outcome.nis, 1.0) << "NIS of the wrapped kTrueErrorRad innovation, not a 2-pi one";
}

TEST(EkfMathTest, FloorDiagonalStopsCollapse) {
    ekf::Mat<2, 2> covariance = ekf::Mat<2, 2>::Identity() * 1e-12;
    ekf::floor_diagonal<2>(covariance, 1e-6);
    EXPECT_DOUBLE_EQ(covariance(0, 0), 1e-6);
    EXPECT_DOUBLE_EQ(covariance(1, 1), 1e-6);
}

TEST(EkfMathTest, NeesConsistencyMonteCarloConstantVelocity) {
    constexpr int kRuns = 100;
    constexpr int kSteps = 60;
    constexpr double kAccelPsd = 2.0;
    constexpr double kMeasurementSigma = 0.02;

    std::mt19937 rng(1234);
    std::normal_distribution<double> randn(0.0, 1.0);

    const double dt = kFramePeriod;
    ekf::Mat<4, 4> transition = ekf::Mat<4, 4>::Identity();
    transition(0, 2) = dt;
    transition(1, 3) = dt;

    ekf::Mat<4, 4> process_noise = ekf::Mat<4, 4>::Zero();
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    process_noise(0, 0) = process_noise(1, 1) = kAccelPsd * dt3 / 3.0;
    process_noise(0, 2) = process_noise(2, 0) = kAccelPsd * dt2 / 2.0;
    process_noise(1, 3) = process_noise(3, 1) = kAccelPsd * dt2 / 2.0;
    process_noise(2, 2) = process_noise(3, 3) = kAccelPsd * dt;
    const ekf::Mat<4, 4> process_noise_sqrt = process_noise.llt().matrixL();

    ekf::Mat<2, 4> observation = ekf::Mat<2, 4>::Zero();
    observation(0, 0) = 1.0;
    observation(1, 1) = 1.0;
    const ekf::Mat<2, 2> measurement_noise =
        ekf::Mat<2, 2>::Identity() * (kMeasurementSigma * kMeasurementSigma);

    ekf::Mat<4, 4> initial_covariance = ekf::Mat<4, 4>::Zero();
    initial_covariance(0, 0) = initial_covariance(1, 1) = 0.01;
    initial_covariance(2, 2) = initial_covariance(3, 3) = 1.0;
    const ekf::Mat<4, 4> initial_covariance_sqrt = initial_covariance.llt().matrixL();

    double nees_sum = 0.0;
    int nees_count = 0;
    for (int run = 0; run < kRuns; ++run) {
        ekf::Vec<4> truth;
        truth << 0.0, 0.0, 1.0, -0.5;

        ekf::Vec<4> noise_seed;
        for (int i = 0; i < 4; ++i) noise_seed(i) = randn(rng);
        ekf::Vec<4> state = truth + initial_covariance_sqrt * noise_seed;
        ekf::Mat<4, 4> covariance = initial_covariance;

        for (int step = 0; step < kSteps; ++step) {
            ekf::Vec<4> process_sample;
            for (int i = 0; i < 4; ++i) process_sample(i) = randn(rng);
            truth = transition * truth + process_noise_sqrt * process_sample;

            state = transition * state;
            ekf::propagate_covariance<4>(covariance, transition, process_noise);

            ekf::Vec<2> measurement(truth(0) + kMeasurementSigma * randn(rng),
                                    truth(1) + kMeasurementSigma * randn(rng));
            const ekf::Vec<2> predicted(state(0), state(1));
            const auto outcome = ekf::ekf_update<4, 2>(
                state, covariance, measurement, predicted, observation, measurement_noise, 0.0,
                {false, false}, {false, false, false, false}, 1e-15);
            ASSERT_TRUE(outcome.accepted);

            const ekf::Vec<4> error = truth - state;
            nees_sum += error.dot(covariance.llt().solve(error));
            ++nees_count;
        }
    }

    const double mean_nees = nees_sum / nees_count;
    EXPECT_GT(mean_nees, 3.5);
    EXPECT_LT(mean_nees, 4.5);
}

}  // namespace
}  // namespace auto_battlebot
