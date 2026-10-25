#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include "transmitter/differential_drive_processor.hpp"
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
    return {.linear_x = linear, .linear_y = 0.0, .angular_z = angular};
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
                    << "linear=" << cmd.linear_x << " angular=" << cmd.angular_z
                    << " reverse_linear=" << reverse_linear
                    << " reverse_angular=" << reverse_angular;
                EXPECT_NEAR(body.angular, differential_body.angular, 1e-12)
                    << "linear=" << cmd.linear_x << " angular=" << cmd.angular_z
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
                // Saturation and deadzones still shape the command; only the sign must match.
                const auto expected = plain.process(cmd);
                for (const DriveProcessorInterface* processor :
                     {static_cast<const DriveProcessorInterface*>(&differential),
                      static_cast<const DriveProcessorInterface*>(&tank)}) {
                    const auto body = processor->to_body_velocity(processor->process(cmd));
                    EXPECT_NEAR(body.linear, expected.channel_a, 1e-12)
                        << "linear=" << cmd.linear_x << " angular=" << cmd.angular_z
                        << " reverse_linear=" << reverse_linear
                        << " reverse_angular=" << reverse_angular;
                    EXPECT_NEAR(body.angular, expected.channel_b, 1e-12)
                        << "linear=" << cmd.linear_x << " angular=" << cmd.angular_z
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

    // Spinning in place drives the wheels in opposite directions: left forward, right reverse.
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

// Angular has priority over linear: with the budget at 1.0, a full-scale turn leaves no headroom
// for forward motion, so the wheels end up at +/-1.
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
