#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "time/manual_clock.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot {
namespace {

/** CommandFeedback carries the same command in two spaces: normalized stick for the EKF's
 * plant model and physical body velocity for the dead-reckoning arm. The live bug this
 * pins down: feeding the physical map into the plant applies the fitted gains twice. */
TEST(PlaybackTransmitterTest, FeedbackCarriesStickAndPhysicalCommands) {
    PlaybackTransmitterConfiguration config;
    config.max_motor_rpm = 1500.0;
    config.wheel_diameter = 0.05;
    config.wheel_track_width = 0.195;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    // Within the saturation budget and above the (zero) deadzones, so the drive processor
    // round-trips the command unchanged.
    transmitter.send(VelocityCommand{0.5, 0.0, -0.25});
    const CommandFeedback feedback = transmitter.update();

    ASSERT_EQ(feedback.stick_commands.count(FrameId::OUR_ROBOT_1), 1u);
    ASSERT_EQ(feedback.commands.count(FrameId::OUR_ROBOT_1), 1u);
    const VelocityCommand &stick = feedback.stick_commands.at(FrameId::OUR_ROBOT_1);
    const VelocityCommand &physical = feedback.commands.at(FrameId::OUR_ROBOT_1);
    EXPECT_NEAR(stick.linear_x, 0.5, 1e-9);
    EXPECT_NEAR(stick.angular_z, -0.25, 1e-9);
    const double max_linear_mps = 1500.0 * M_PI * 0.05 / 60.0;
    const double max_angular_radps = 2.0 * max_linear_mps / 0.195;
    EXPECT_NEAR(physical.linear_x, 0.5 * max_linear_mps, 1e-9);
    EXPECT_NEAR(physical.angular_z, -0.25 * max_angular_radps, 1e-9);
}

TEST(PlaybackTransmitterTest, NoFeedbackBeforeFirstSend) {
    PlaybackTransmitterConfiguration config;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    const CommandFeedback feedback = transmitter.update();
    EXPECT_TRUE(feedback.commands.empty());
    EXPECT_TRUE(feedback.stick_commands.empty());
}

/** The sim plant consumes normalized stick directly, so feedback fills only the stick map.
 * Before this stream existed the command-driven prediction path was inert in simulation. */
TEST(SimTransmitterTest, FeedbackReportsLastSentStickCommand) {
    SimTransmitterConfiguration config;
    config.init_delay_seconds = 0.0;
    auto clock = std::make_shared<ManualClock>();
    SimTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());
    transmitter.enable();

    EXPECT_TRUE(transmitter.update().stick_commands.empty());

    transmitter.send(VelocityCommand{0.4, 0.0, 0.2});
    const CommandFeedback feedback = transmitter.update();
    ASSERT_EQ(feedback.stick_commands.count(FrameId::OUR_ROBOT_1), 1u);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).linear_x, 0.4, 1e-9);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).angular_z, 0.2, 1e-9);
    EXPECT_TRUE(feedback.commands.empty());
}

TEST(SimTransmitterTest, DisableClearsFeedback) {
    SimTransmitterConfiguration config;
    config.init_delay_seconds = 0.0;
    auto clock = std::make_shared<ManualClock>();
    SimTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());
    transmitter.enable();
    transmitter.send(VelocityCommand{0.4, 0.0, 0.2});

    transmitter.disable();
    EXPECT_TRUE(transmitter.update().stick_commands.empty());
}

}  // namespace
}  // namespace auto_battlebot
