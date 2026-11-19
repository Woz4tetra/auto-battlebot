#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "transmitter/differential_drive_processor.hpp"
#include "transmitter/drive_mixing.hpp"
#include "transmitter/drive_processor_interface.hpp"
#include "transmitter/tank_drive_processor.hpp"

using namespace auto_battlebot;

namespace {

std::shared_ptr<DiagnosticsModuleLogger> make_logger() {
    return std::make_shared<DiagnosticsModuleLogger>("test_drive_processor");
}

DriveProcessorInterface::Config make_config(bool reverse_linear, bool reverse_angular) {
    return {
        .velocity_saturation_limit = 1.0,
        .zero_deadzone_percent = 2.0,
        .lifted_deadzone_percent = 10.0,
        .reverse_linear = reverse_linear,
        .reverse_angular = reverse_angular,
    };
}

VelocityCommand command(double linear, double angular) {
    return {.forward_mps = linear, .lateral_mps = 0.0, .yaw_rate_radps = angular};
}

const std::vector<VelocityCommand> kCommands = {
    command(0.0, 0.0),  command(0.6, 0.0),   command(0.0, 0.5),  command(0.4, 0.4),
    command(-0.7, 0.2), command(0.01, 0.01), command(1.5, -0.9),
};

}  // namespace

// The two processors run the same saturation and deadzone stages; tank stops in wheel space while
// differential converts back to body axes. Un-mixing the tank output must land on the differential
// output, including when the reversals are on (each applies them in a different stage).
TEST(DriveProcessorTest, TankUnMixesToDifferentialOutput) {
    for (const bool reverse_linear : {false, true}) {
        for (const bool reverse_angular : {false, true}) {
            const auto config = make_config(reverse_linear, reverse_angular);
            const DifferentialDriveProcessor differential(config, make_logger());
            const TankDriveProcessor tank(config, make_logger());

            for (const auto& cmd : kCommands) {
                const auto body = tank.to_body_velocity(tank.process(cmd));
                const auto differential_body =
                    differential.to_body_velocity(differential.process(cmd));
                EXPECT_NEAR(body.linear, differential_body.linear, 1e-12)
                    << "linear=" << cmd.forward_mps << " angular=" << cmd.yaw_rate_radps
                    << " reverse_linear=" << reverse_linear
                    << " reverse_angular=" << reverse_angular;
                EXPECT_NEAR(body.angular, differential_body.angular, 1e-12)
                    << "linear=" << cmd.forward_mps << " angular=" << cmd.yaw_rate_radps
                    << " reverse_linear=" << reverse_linear
                    << " reverse_angular=" << reverse_angular;
            }
        }
    }
}

// The readback reports the command in the frame process() took it, so the filter predicts motion
// in the direction the navigator asked for. The reversals only change what goes on the wire.
TEST(DriveProcessorTest, ReadbackUndoesTheReversals) {
    for (const bool reverse_linear : {false, true}) {
        for (const bool reverse_angular : {false, true}) {
            const auto config = make_config(reverse_linear, reverse_angular);
            const auto plain_config = make_config(false, false);
            const DifferentialDriveProcessor differential(config, make_logger());
            const TankDriveProcessor tank(config, make_logger());
            const DifferentialDriveProcessor plain(plain_config, make_logger());

            for (const auto& cmd : kCommands) {
                const auto expected = plain.process(cmd);
                for (const DriveProcessorInterface* processor :
                     {static_cast<const DriveProcessorInterface*>(&differential),
                      static_cast<const DriveProcessorInterface*>(&tank)}) {
                    const auto body = processor->to_body_velocity(processor->process(cmd));
                    EXPECT_NEAR(body.linear, expected.channel_a, 1e-12)
                        << "linear=" << cmd.forward_mps << " angular=" << cmd.yaw_rate_radps
                        << " reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                    EXPECT_NEAR(body.angular, expected.channel_b, 1e-12)
                        << "linear=" << cmd.forward_mps << " angular=" << cmd.yaw_rate_radps
                        << " reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                }
            }
        }
    }
}

TEST(DriveProcessorTest, TankSplitsCommandIntoWheelChannels) {
    const TankDriveProcessor tank({.velocity_saturation_limit = 1.0, .zero_deadzone_percent = 0.0},
                                  make_logger());

    const auto straight = tank.process(command(0.5, 0.0));
    EXPECT_NEAR(straight.channel_a, 0.5, 1e-12);
    EXPECT_NEAR(straight.channel_b, 0.5, 1e-12);

    const auto spin = tank.process(command(0.0, 0.4));
    EXPECT_NEAR(spin.channel_a, 0.4, 1e-12);
    EXPECT_NEAR(spin.channel_b, -0.4, 1e-12);
}

TEST(DriveProcessorTest, TankReverseAngularSwapsWheels) {
    const TankDriveProcessor tank({.velocity_saturation_limit = 1.0, .reverse_angular = true},
                                  make_logger());

    const auto spin = tank.process(command(0.0, 0.4));
    EXPECT_NEAR(spin.channel_a, -0.4, 1e-12);
    EXPECT_NEAR(spin.channel_b, 0.4, 1e-12);
}

TEST(DriveProcessorTest, TankAppliesDeadzonesPerWheel) {
    const TankDriveProcessor tank(make_config(false, false), make_logger());

    // Below the zero deadzone both wheels are silent.
    const auto tiny = tank.process(command(0.01, 0.0));
    EXPECT_EQ(tiny.channel_a, 0.0);
    EXPECT_EQ(tiny.channel_b, 0.0);

    // Just above it the wheels jump to the lifted deadzone so they overcome static friction.
    const auto small = tank.process(command(0.021, 0.0));
    EXPECT_GE(std::abs(small.channel_a), 0.1);
    EXPECT_GE(std::abs(small.channel_b), 0.1);
}

TEST(DriveProcessorTest, TankSaturationGivesAngularPriority) {
    const TankDriveProcessor tank({.velocity_saturation_limit = 1.0, .zero_deadzone_percent = 0.0},
                                  make_logger());

    const auto saturated = tank.process(command(1.0, 1.0));
    EXPECT_NEAR(saturated.channel_a, 1.0, 1e-12);
    EXPECT_NEAR(saturated.channel_b, -1.0, 1e-12);
}

TEST(DriveProcessorTest, FactoryBuildsBothProcessorsAndRejectsUnknown) {
    const auto config = make_config(false, false);
    EXPECT_NE(dynamic_cast<TankDriveProcessor*>(
                  make_drive_processor("TankDriveProcessor", config, make_logger()).get()),
              nullptr);
    EXPECT_NE(dynamic_cast<DifferentialDriveProcessor*>(
                  make_drive_processor("DifferentialDriveProcessor", config, make_logger()).get()),
              nullptr);
    EXPECT_THROW(make_drive_processor("NopeDriveProcessor", config, make_logger()),
                 std::invalid_argument);
}

TEST(DriveProcessorTest, InverseRoundTripsWithReversals) {
    const std::vector<VelocityCommand> in_range = {command(0.0, 0.0), command(0.6, 0.0),
                                                   command(0.0, 0.5), command(0.4, 0.4),
                                                   command(-0.7, 0.2)};
    for (const bool reverse_linear : {false, true}) {
        for (const bool reverse_angular : {false, true}) {
            DriveProcessorInterface::Config config = {.velocity_saturation_limit = 1.0,
                                                      .reverse_linear = reverse_linear,
                                                      .reverse_angular = reverse_angular};
            const DifferentialDriveProcessor differential(config, make_logger());
            const TankDriveProcessor tank(config, make_logger());
            for (const DriveProcessorInterface* processor :
                 {static_cast<const DriveProcessorInterface*>(&differential),
                  static_cast<const DriveProcessorInterface*>(&tank)}) {
                for (const auto& cmd : in_range) {
                    const auto body = processor->to_body_velocity(processor->process(cmd));
                    EXPECT_NEAR(body.linear, cmd.forward_mps, 1e-12)
                        << "reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                    EXPECT_NEAR(body.angular, cmd.yaw_rate_radps, 1e-12)
                        << "reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                }
            }
        }
    }
}

TEST(DriveMixingTest, SaturateGivesAngularPriority) {
    const auto both_full = saturate_velocity(1.0, 1.0, 1.0);
    EXPECT_DOUBLE_EQ(both_full.angular, 1.0);
    EXPECT_DOUBLE_EQ(both_full.linear, 0.0);

    const auto partial = saturate_velocity(1.0, 0.4, 1.0);
    EXPECT_DOUBLE_EQ(partial.angular, 0.4);
    EXPECT_NEAR(partial.linear, 0.6, 1e-12);

    const auto reverse = saturate_velocity(-1.0, 0.4, 1.0);
    EXPECT_NEAR(reverse.linear, -0.6, 1e-12);
}

TEST(DriveMixingTest, SaturateLimitZeroClampsEachAxis) {
    const auto clamped = saturate_velocity(1.5, -2.0, 0.0);
    EXPECT_DOUBLE_EQ(clamped.linear, 1.0);
    EXPECT_DOUBLE_EQ(clamped.angular, -1.0);
}

TEST(DriveMixingTest, MixWithinUnitRange) {
    for (const double linear : {-1.5, -1.0, -0.3, 0.0, 0.3, 1.0, 1.5}) {
        for (const double angular : {-1.5, -1.0, -0.3, 0.0, 0.3, 1.0, 1.5}) {
            const auto wheels = mix_to_wheels(saturate_velocity(linear, angular, 1.0));
            EXPECT_LE(std::abs(wheels.left), 1.0 + 1e-12)
                << "linear=" << linear << " angular=" << angular;
            EXPECT_LE(std::abs(wheels.right), 1.0 + 1e-12)
                << "linear=" << linear << " angular=" << angular;
        }
    }
}

TEST(DriveMixingTest, LiftedDeadzoneMapping) {
    EXPECT_EQ(apply_lifted_deadzone(0.02, 0.02, 0.1), 0.0);
    EXPECT_EQ(apply_lifted_deadzone(-0.01, 0.02, 0.1), 0.0);

    const double just_above = apply_lifted_deadzone(0.0201, 0.02, 0.1);
    EXPECT_GE(just_above, 0.1);
    EXPECT_LT(just_above, 0.11);

    EXPECT_NEAR(apply_lifted_deadzone(1.0, 0.02, 0.1), 1.0, 1e-12);
    EXPECT_NEAR(apply_lifted_deadzone(-1.0, 0.02, 0.1), -1.0, 1e-12);
    EXPECT_LT(apply_lifted_deadzone(-0.5, 0.02, 0.1), -0.1);
}
