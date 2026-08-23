#include <gtest/gtest.h>

#include <cmath>

#include "navigation/motion_profile_navigation.hpp"
#include "plant/plant_params.hpp"

namespace auto_battlebot {
namespace {

// Mrs Buff Mk3, from playground/calibration/out/plant_stageA.toml.
constexpr double kDeadzoneAngLeft = 0.016061;    // dz_ang_l
constexpr double kDeadzoneAngRight = 0.0240734;  // dz_ang_r
constexpr double kSteerBrake = 2.70197;          // c_sb
constexpr double kSteerBrakeFloor = 0.3;

using Nav = MotionProfileNavigation;

// --- plant_effective_command: what the plant sees after its own deadzone eats the low end.
// Shared with JigPlantModel, which applies the same curve forward.

TEST(PlantEffectiveCommand, ZeroCommandStaysZero) {
    EXPECT_DOUBLE_EQ(plant_effective_command(0.0, kDeadzoneAngLeft, kDeadzoneAngRight), 0.0);
}

TEST(PlantEffectiveCommand, FullCommandMapsToFullEffect) {
    // The rescale by 1/(1 - deadzone) exists so saturation still means saturation. Without it a
    // deadzone would quietly cap the plant's reachable authority below 1.
    EXPECT_DOUBLE_EQ(plant_effective_command(1.0, kDeadzoneAngLeft, kDeadzoneAngRight), 1.0);
    EXPECT_DOUBLE_EQ(plant_effective_command(-1.0, kDeadzoneAngLeft, kDeadzoneAngRight), -1.0);
}

TEST(PlantEffectiveCommand, CommandInsideDeadzoneProducesNothing) {
    // Anything under dz_ang_l does not move the robot, so the coupling terms must not be charged
    // for it. Half the deadzone is inside for both signs.
    EXPECT_DOUBLE_EQ(
        plant_effective_command(kDeadzoneAngLeft * 0.5, kDeadzoneAngLeft, kDeadzoneAngRight), 0.0);
    EXPECT_DOUBLE_EQ(
        plant_effective_command(-kDeadzoneAngRight * 0.5, kDeadzoneAngLeft, kDeadzoneAngRight),
        0.0);
}

TEST(PlantEffectiveCommand, DeadzoneIsPerSign) {
    // dz_ang_r is 1.5x dz_ang_l, so the same command magnitude yields less effect turning right.
    // A symmetric deadzone would hide this, and it is the asymmetry the jig actually measured.
    const double magnitude = 0.02;  // between dz_ang_l and dz_ang_r
    const double left = plant_effective_command(magnitude, kDeadzoneAngLeft, kDeadzoneAngRight);
    const double right = plant_effective_command(-magnitude, kDeadzoneAngLeft, kDeadzoneAngRight);
    EXPECT_GT(left, 0.0);
    EXPECT_DOUBLE_EQ(right, 0.0);  // 0.02 is still inside the right deadzone
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
    // Guards the 1/(1 - deadzone) denominator against a bad refit. Clamped at 0.95.
    const double effect = plant_effective_command(1.0, 5.0, 5.0);
    EXPECT_TRUE(std::isfinite(effect));
    EXPECT_DOUBLE_EQ(effect, 1.0);
}

TEST(PlantEffectiveCommand, NoDeadzoneIsIdentity) {
    for (const double command : {-1.0, -0.4, 0.0, 0.25, 1.0}) {
        EXPECT_DOUBLE_EQ(plant_effective_command(command, 0.0, 0.0), command);
    }
}

// --- compensate_coupling: divide out the loss the plant is about to apply ---

TEST(MotionProfileCompensateCoupling, DisabledCoefficientIsIdentity) {
    // angular_droop_coeff ships at 0.0, so this is the shipped path for that channel.
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, 0.5, 0.0, kSteerBrakeFloor), 0.6);
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, 0.5, -1.0, kSteerBrakeFloor), 0.6);
}

TEST(MotionProfileCompensateCoupling, NoCouplingEffectLeavesCommandAlone) {
    // Driving straight: no angular command, so no steer-brake loss to undo.
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, 0.0, kSteerBrake, kSteerBrakeFloor), 0.6);
}

TEST(MotionProfileCompensateCoupling, UndoesTheModelledLossExactly) {
    // The point of the whole term: command x, plant multiplies by (1 - c_sb*|u_ang|), robot gets x.
    const double command = 0.5;
    const double effect = 0.1;
    const double authority = 1.0 - kSteerBrake * effect;
    ASSERT_GT(authority, kSteerBrakeFloor);  // otherwise this test is checking the floor instead

    const double compensated =
        Nav::compensate_coupling(command, effect, kSteerBrake, kSteerBrakeFloor);
    EXPECT_GT(compensated, command);
    EXPECT_DOUBLE_EQ(compensated * authority, command);
}

TEST(MotionProfileCompensateCoupling, IsSymmetricInTheSignOfTheEffect) {
    // Turning left and right cost the same forward speed; the loss is on |u_ang|.
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.5, 0.1, kSteerBrake, kSteerBrakeFloor),
                     Nav::compensate_coupling(0.5, -0.1, kSteerBrake, kSteerBrakeFloor));
}

TEST(MotionProfileCompensateCoupling, PreservesTheSignOfTheCommand) {
    const double forward = Nav::compensate_coupling(0.5, 0.1, kSteerBrake, kSteerBrakeFloor);
    const double reverse = Nav::compensate_coupling(-0.5, 0.1, kSteerBrake, kSteerBrakeFloor);
    EXPECT_DOUBLE_EQ(reverse, -forward);
}

TEST(MotionProfileCompensateCoupling, SaturatesAtTheFloorInsteadOfBlowingUp) {
    // c_sb's fitted linear loss hits zero authority at |u_ang| = 1/2.702 = 0.370. Past that the
    // fit no longer describes the plant, so the floor has to catch it: without it the divisor
    // crosses zero and the command goes infinite, then flips sign.
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

// --- the two composed, which is how compute_command uses them ---

TEST(MotionProfilePlantAlgebra, DeadzoneKeepsTinyTurnsFromCostingForwardSpeed) {
    // A turn command below the plant's angular deadzone produces no yaw, so it must not trigger
    // a steer-brake correction either. Feeding the raw command instead of the effective one would
    // charge forward speed for a turn that never happens.
    const double tiny_turn = kDeadzoneAngLeft * 0.5;
    const double effect = plant_effective_command(tiny_turn, kDeadzoneAngLeft, kDeadzoneAngRight);
    EXPECT_DOUBLE_EQ(Nav::compensate_coupling(0.6, effect, kSteerBrake, kSteerBrakeFloor), 0.6);

    // Same command fed raw would have inflated the forward command by ~4%.
    EXPECT_GT(Nav::compensate_coupling(0.6, tiny_turn, kSteerBrake, kSteerBrakeFloor), 0.6);
}

}  // namespace
}  // namespace auto_battlebot
