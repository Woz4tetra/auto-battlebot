#include <gtest/gtest.h>

#include <cmath>

#include "navigation/motion_profile_navigation.hpp"
#include "plant/plant_params.hpp"

namespace auto_battlebot {
namespace {

constexpr double kDeadzoneAngLeft = 0.016061;
constexpr double kDeadzoneAngRight = 0.0240734;
constexpr double kSteerBrake = 2.70197;
constexpr double kSteerBrakeFloor = 0.3;

using Nav = MotionProfileNavigation;

TEST(PlantEffectiveCommand, ZeroCommandStaysZero) {
    EXPECT_DOUBLE_EQ(plant_effective_command(0.0, kDeadzoneAngLeft, kDeadzoneAngRight), 0.0);
}

TEST(PlantEffectiveCommand, FullCommandMapsToFullEffect) {
    EXPECT_DOUBLE_EQ(plant_effective_command(1.0, kDeadzoneAngLeft, kDeadzoneAngRight), 1.0);
    EXPECT_DOUBLE_EQ(plant_effective_command(-1.0, kDeadzoneAngLeft, kDeadzoneAngRight), -1.0);
}

TEST(PlantEffectiveCommand, CommandInsideDeadzoneProducesNothing) {
    EXPECT_DOUBLE_EQ(
        plant_effective_command(kDeadzoneAngLeft * 0.5, kDeadzoneAngLeft, kDeadzoneAngRight), 0.0);
    EXPECT_DOUBLE_EQ(
        plant_effective_command(-kDeadzoneAngRight * 0.5, kDeadzoneAngLeft, kDeadzoneAngRight),
        0.0);
}

TEST(PlantEffectiveCommand, DeadzoneIsPerSign) {
    const double magnitude = 0.5 * (kDeadzoneAngLeft + kDeadzoneAngRight);
    const double left = plant_effective_command(magnitude, kDeadzoneAngLeft, kDeadzoneAngRight);
    const double right = plant_effective_command(-magnitude, kDeadzoneAngLeft, kDeadzoneAngRight);
    EXPECT_GT(left, 0.0);
    EXPECT_DOUBLE_EQ(right, 0.0) << "the magnitude sits between the two deadzones";
}

TEST(PlantEffectiveCommand, IsMonotonicAndOddOnASymmetricDeadzone) {
    double previous = -1.0;
    for (double command = 0.0; command <= 1.0; command += 0.05) {
        const double effect = plant_effective_command(command, 0.1, 0.1);
        EXPECT_GE(effect, previous);
        EXPECT_DOUBLE_EQ(plant_effective_command(-command, 0.1, 0.1), -effect);
        previous = effect;
    }
}

TEST(PlantEffectiveCommand, AbsurdDeadzoneIsClampedNotDividedByZero) {
    const double effect = plant_effective_command(1.0, 5.0, 5.0);
    EXPECT_TRUE(std::isfinite(effect));
    EXPECT_DOUBLE_EQ(effect, 1.0);
}

TEST(PlantEffectiveCommand, NoDeadzoneIsIdentity) {
    for (const double command : {-1.0, -0.4, 0.0, 0.25, 1.0}) {
        EXPECT_DOUBLE_EQ(plant_effective_command(command, 0.0, 0.0), command);
    }
}

TEST(MotionProfileCompensateCoupling, DisabledCoefficientIsIdentity) {
    const MotionProfileNavigationConfiguration shipped;
    const double kShippedDroopCoeff = shipped.compensate_angular_droop ? shipped.plant.c_ad : 0.0;
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, 0.5, kShippedDroopCoeff, kSteerBrakeFloor), 0.6);
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, 0.5, -1.0, kSteerBrakeFloor), 0.6);
}

TEST(MotionProfileCompensateCoupling, NoCouplingEffectLeavesCommandAlone) {
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, 0.0, kSteerBrake, kSteerBrakeFloor), 0.6);
}

TEST(MotionProfileCompensateCoupling, UndoesTheModelledLossExactly) {
    const double command = 0.5;
    const double effect = 0.1;
    const double authority = 1.0 - kSteerBrake * effect;
    ASSERT_GT(authority, kSteerBrakeFloor) << "effect too large: the test would check the floor";

    const double compensated =
        Nav::compensate_coupling(command, effect, kSteerBrake, kSteerBrakeFloor);
    EXPECT_GT(compensated, command);
    EXPECT_DOUBLE_EQ(compensated * authority, command);
}

TEST(MotionProfileCompensateCoupling, IsSymmetricInTheSignOfTheEffect) {
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.5, 0.1, kSteerBrake, kSteerBrakeFloor),
                     Nav::compensate_coupling(0.5, -0.1, kSteerBrake, kSteerBrakeFloor));
}

TEST(MotionProfileCompensateCoupling, PreservesTheSignOfTheCommand) {
    const double forward = Nav::compensate_coupling(0.5, 0.1, kSteerBrake, kSteerBrakeFloor);
    const double reverse = Nav::compensate_coupling(-0.5, 0.1, kSteerBrake, kSteerBrakeFloor);
    EXPECT_DOUBLE_EQ(reverse, -forward);
}

TEST(MotionProfileCompensateCoupling, SaturatesAtTheFloorInsteadOfBlowingUp) {
    const double singular = 1.0 / kSteerBrake;
    for (const double effect : {singular - 1e-9, singular, singular + 0.2, 1.0}) {
        const double compensated =
            Nav::compensate_coupling(0.5, effect, kSteerBrake, kSteerBrakeFloor);
        EXPECT_TRUE(std::isfinite(compensated));
        EXPECT_GT(compensated, 0.0) << "sign flipped at effect=" << effect;
        EXPECT_DOUBLE_EQ(compensated, 0.5 / kSteerBrakeFloor);
    }
}

TEST(MotionProfileCompensateCoupling, GrowsMonotonicallyUpToTheFloor) {
    double previous = 0.0;
    for (double effect = 0.0; effect <= 1.0; effect += 0.02) {
        const double compensated =
            Nav::compensate_coupling(0.5, effect, kSteerBrake, kSteerBrakeFloor);
        EXPECT_GE(compensated, previous);
        EXPECT_LE(compensated, 0.5 / kSteerBrakeFloor);
        previous = compensated;
    }
}

TEST(MotionProfilePlantAlgebra, DeadzoneKeepsTinyTurnsFromCostingForwardSpeed) {
    const double tiny_turn = kDeadzoneAngLeft * 0.5;
    const double effect = plant_effective_command(tiny_turn, kDeadzoneAngLeft, kDeadzoneAngRight);
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, effect, kSteerBrake, kSteerBrakeFloor), 0.6);

    EXPECT_GT(Nav::compensate_coupling(0.6, tiny_turn, kSteerBrake, kSteerBrakeFloor), 0.6)
        << "the raw tiny turn would inflate the forward command by about 4%";
}

}  // namespace
}  // namespace auto_battlebot
