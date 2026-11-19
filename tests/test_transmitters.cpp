#include <gtest/gtest.h>

#include <memory>

#include "time/manual_clock.hpp"
#include "transmitter/drive_mixing.hpp"
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
    const StickCommand &stick = feedback.stick_commands.at(FrameId::OUR_ROBOT_1);
    EXPECT_NEAR(stick.forward, 0.5, 1e-9);
    EXPECT_NEAR(stick.yaw, -0.25, 1e-9);
}

TEST(PlaybackTransmitterTest, NoFeedbackBeforeFirstSend) {
    PlaybackTransmitterConfiguration config;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    const CommandFeedback feedback = transmitter.update();
    EXPECT_TRUE(feedback.stick_commands.empty());
}

TEST(PlaybackTransmitterTest, InitPressOnFixedLogicalTime) {
    PlaybackTransmitterConfiguration config;
    config.init_delay_seconds = 2.0;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    clock->set(100.0);
    transmitter.update();
    EXPECT_FALSE(transmitter.did_init_button_press());

    clock->set(101.9);
    transmitter.update();
    EXPECT_FALSE(transmitter.did_init_button_press());

    clock->set(102.0);
    transmitter.update();
    EXPECT_TRUE(transmitter.did_init_button_press());
    EXPECT_FALSE(transmitter.did_init_button_press());
}

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
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).forward, 0.4, 1e-9);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).yaw, 0.2, 1e-9);
}

TEST(PlaybackTransmitterTest, FeedbackStreamsBeforeInitButton) {
    PlaybackTransmitterConfiguration config;
    config.init_delay_seconds = 10.0;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    clock->set(1.0);
    transmitter.send(VelocityCommand{0.5, 0.0, 0.0});
    const CommandFeedback feedback = transmitter.update();
    EXPECT_FALSE(transmitter.did_init_button_press());
    ASSERT_EQ(feedback.stick_commands.count(FrameId::OUR_ROBOT_1), 1u);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).forward, 0.5, 1e-9);
}

TEST(SimTransmitterTest, SaturatesLikeDriveProcessor) {
    SimTransmitterConfiguration config;
    auto clock = std::make_shared<ManualClock>();
    SimTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());
    transmitter.enable();

    transmitter.send(VelocityCommand{1.0, 0.0, 0.5});
    const auto expected = saturate_velocity(1.0, 0.5, config.velocity_saturation_limit);
    ASSERT_LT(expected.linear, 1.0);
    const CommandFeedback feedback = transmitter.update();
    const StickCommand &stick = feedback.stick_commands.at(FrameId::OUR_ROBOT_1);
    EXPECT_NEAR(stick.forward, expected.linear, 1e-12);
    EXPECT_NEAR(stick.yaw, expected.angular, 1e-12);
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
