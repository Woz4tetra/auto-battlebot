#include <gtest/gtest.h>

#include <memory>

#include "time/manual_clock.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot {
namespace {

/** Feedback is normalized stick space only. Physical scaling is the consumer's job, through the
 * fitted [plant] gains: when the transmitter also published a pre-scaled physical map, the EKF
 * read it and applied those gains a second time. */
TEST(PlaybackTransmitterTest, FeedbackReportsStickCommand) {
    PlaybackTransmitterConfiguration config;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    // Within the saturation budget and above the (zero) deadzones, so the drive processor
    // round-trips the command unchanged.
    transmitter.send(VelocityCommand{0.5, 0.0, -0.25});
    const CommandFeedback feedback = transmitter.update();

    ASSERT_EQ(feedback.stick_commands.count(FrameId::OUR_ROBOT_1), 1u);
    const VelocityCommand &stick = feedback.stick_commands.at(FrameId::OUR_ROBOT_1);
    EXPECT_NEAR(stick.linear_x, 0.5, 1e-9);
    EXPECT_NEAR(stick.angular_z, -0.25, 1e-9);
}

TEST(PlaybackTransmitterTest, NoFeedbackBeforeFirstSend) {
    PlaybackTransmitterConfiguration config;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    const CommandFeedback feedback = transmitter.update();
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
