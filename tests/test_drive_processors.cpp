#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include "command_shaper/command_shaper_interface.hpp"
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
        .zero_deadzone_percent = 2.0,
        .lifted_deadzone_percent = 10.0,
        .reverse_linear = reverse_linear,
        .reverse_angular = reverse_angular,
    };
}

StickCommand command(double linear, double angular) {
    return saturate_stick({.forward = linear, .lateral = 0.0, .yaw = angular}, 1.0);
}

StickCommand raw(double linear, double angular) {
    return {.forward = linear, .lateral = 0.0, .yaw = angular};
}

const std::vector<StickCommand> kCommands = {
    command(0.0, 0.0),  command(0.6, 0.0),   command(0.0, 0.5),  command(0.4, 0.4),
    command(-0.7, 0.2), command(0.01, 0.01), command(1.5, -0.9),
};

}  // namespace

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
                    << "linear=" << cmd.forward << " angular=" << cmd.yaw
                    << " reverse_linear=" << reverse_linear
                    << " reverse_angular=" << reverse_angular;
                EXPECT_NEAR(body.angular, differential_body.angular, 1e-12)
                    << "linear=" << cmd.forward << " angular=" << cmd.yaw
                    << " reverse_linear=" << reverse_linear
                    << " reverse_angular=" << reverse_angular;
            }
        }
    }
}

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
                    EXPECT_NEAR(body.linear, expected.first_output, 1e-12)
                        << "linear=" << cmd.forward << " angular=" << cmd.yaw
                        << " reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                    EXPECT_NEAR(body.angular, expected.second_output, 1e-12)
                        << "linear=" << cmd.forward << " angular=" << cmd.yaw
                        << " reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                }
            }
        }
    }
}

TEST(DriveProcessorTest, OutputsStayInUnitRangeForOversizedCommands) {
    const std::vector<StickCommand> oversized = {raw(5.0, 5.0),   raw(-5.0, 5.0), raw(5.0, -5.0),
                                                 raw(-5.0, -5.0), raw(3.0, 0.0),  raw(0.0, -3.0)};
    for (const double saturation_limit : {0.0, 0.5, 1.0, 4.0}) {
        const auto config = make_config(false, true);
        const DifferentialDriveProcessor differential(config, make_logger());
        const TankDriveProcessor tank(config, make_logger());
        for (const auto& oversized_cmd : oversized) {
            const StickCommand cmd = saturate_stick(oversized_cmd, saturation_limit);
            for (const DriveProcessorInterface* processor :
                 {static_cast<const DriveProcessorInterface*>(&differential),
                  static_cast<const DriveProcessorInterface*>(&tank)}) {
                const auto out = processor->process(cmd);
                EXPECT_LE(std::abs(out.first_output), 1.0)
                    << "limit=" << saturation_limit << " cmd=" << cmd.forward << "," << cmd.yaw;
                EXPECT_LE(std::abs(out.second_output), 1.0)
                    << "limit=" << saturation_limit << " cmd=" << cmd.forward << "," << cmd.yaw;
            }
            EXPECT_LE(std::abs(cmd.forward), 1.0) << "limit=" << saturation_limit;
            EXPECT_LE(std::abs(cmd.yaw), 1.0) << "limit=" << saturation_limit;
        }
    }
}

TEST(DriveProcessorTest, TankSplitsCommandIntoWheelChannels) {
    const TankDriveProcessor tank({.zero_deadzone_percent = 0.0}, make_logger());

    const auto straight = tank.process(command(0.5, 0.0));
    EXPECT_NEAR(straight.first_output, 0.5, 1e-12);
    EXPECT_NEAR(straight.second_output, 0.5, 1e-12);

    const auto spin = tank.process(command(0.0, 0.4));
    EXPECT_NEAR(spin.first_output, 0.4, 1e-12);
    EXPECT_NEAR(spin.second_output, -0.4, 1e-12);
}

TEST(DriveProcessorTest, TankReverseAngularSwapsWheels) {
    const TankDriveProcessor tank({.reverse_angular = true}, make_logger());

    const auto spin = tank.process(command(0.0, 0.4));
    EXPECT_NEAR(spin.first_output, -0.4, 1e-12);
    EXPECT_NEAR(spin.second_output, 0.4, 1e-12);
}

TEST(DriveProcessorTest, TankAppliesDeadzonesPerWheel) {
    const auto config = make_config(false, false);
    const TankDriveProcessor tank(config, make_logger());

    const double zero_deadzone = config.zero_deadzone_percent / 100.0;
    const double lifted_deadzone = config.lifted_deadzone_percent / 100.0;

    const auto tiny = tank.process(command(0.5 * zero_deadzone, 0.0));
    EXPECT_EQ(tiny.first_output, 0.0);
    EXPECT_EQ(tiny.second_output, 0.0);

    const auto small = tank.process(command(1.05 * zero_deadzone, 0.0));
    EXPECT_GE(std::abs(small.first_output), lifted_deadzone);
    EXPECT_GE(std::abs(small.second_output), lifted_deadzone);
}

TEST(DriveProcessorTest, DifferentialSmallSteerWhileDrivingPassesThrough) {
    const DifferentialDriveProcessor differential(make_config(false, false), make_logger());
    const auto steering = differential.process(command(0.8, 0.01));
    EXPECT_NEAR(steering.second_output, 0.01, 2e-3) << "steer lifted to the deadzone floor";
    const auto nudge = differential.process(command(0.01, 0.8));
    EXPECT_NEAR(nudge.first_output, 0.01, 2e-3) << "linear nudge lifted to the deadzone floor";
}

TEST(DriveProcessorTest, TankSaturationGivesAngularPriority) {
    const TankDriveProcessor tank({.zero_deadzone_percent = 0.0}, make_logger());

    const auto saturated = tank.process(command(1.0, 1.0));
    EXPECT_NEAR(saturated.first_output, 1.0, 1e-12);
    EXPECT_NEAR(saturated.second_output, -1.0, 1e-12);
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
    const std::vector<StickCommand> in_range = {command(0.0, 0.0), command(0.6, 0.0),
                                                command(0.0, 0.5), command(0.4, 0.4),
                                                command(-0.7, 0.2)};
    for (const bool reverse_linear : {false, true}) {
        for (const bool reverse_angular : {false, true}) {
            DriveProcessorInterface::Config config = {.reverse_linear = reverse_linear,
                                                      .reverse_angular = reverse_angular};
            const DifferentialDriveProcessor differential(config, make_logger());
            const TankDriveProcessor tank(config, make_logger());
            for (const DriveProcessorInterface* processor :
                 {static_cast<const DriveProcessorInterface*>(&differential),
                  static_cast<const DriveProcessorInterface*>(&tank)}) {
                for (const auto& cmd : in_range) {
                    const auto body = processor->to_body_velocity(processor->process(cmd));
                    EXPECT_NEAR(body.linear, cmd.forward, 1e-12)
                        << "reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                    EXPECT_NEAR(body.angular, cmd.yaw, 1e-12)
                        << "reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                }
            }
        }
    }
}

TEST(DriveMixingTest, SaturateGivesAngularPriority) {
    const auto both_full = saturate_stick(raw(1.0, 1.0), 1.0);
    EXPECT_DOUBLE_EQ(both_full.yaw, 1.0);
    EXPECT_DOUBLE_EQ(both_full.forward, 0.0);

    const auto partial = saturate_stick(raw(1.0, 0.4), 1.0);
    EXPECT_DOUBLE_EQ(partial.yaw, 0.4);
    EXPECT_NEAR(partial.forward, 0.6, 1e-12);

    const auto reverse = saturate_stick(raw(-1.0, 0.4), 1.0);
    EXPECT_NEAR(reverse.forward, -0.6, 1e-12);
}

TEST(DriveMixingTest, SaturateLimitZeroClampsEachAxis) {
    const auto clamped = saturate_stick(raw(1.5, -2.0), 0.0);
    EXPECT_DOUBLE_EQ(clamped.forward, 1.0);
    EXPECT_DOUBLE_EQ(clamped.yaw, -1.0);
}

TEST(DriveMixingTest, MixWithinUnitRange) {
    for (const double linear : {-1.5, -1.0, -0.3, 0.0, 0.3, 1.0, 1.5}) {
        for (const double angular : {-1.5, -1.0, -0.3, 0.0, 0.3, 1.0, 1.5}) {
            const auto saturated = saturate_stick(raw(linear, angular), 1.0);
            const auto wheels =
                mix_to_wheels({.linear = saturated.forward, .angular = saturated.yaw});
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
