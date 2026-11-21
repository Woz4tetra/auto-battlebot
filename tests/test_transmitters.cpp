#include <gtest/gtest.h>

#include <memory>

#include "time/manual_clock.hpp"
#include "transmitter/playback_transmitter.hpp"
#include "transmitter/sim_transmitter.hpp"

namespace auto_battlebot {
namespace {

TEST(PlaybackTransmitterTest, FeedbackReportsStickCommand) {
    PlaybackTransmitterConfiguration config;
    auto clock = std::make_shared<ManualClock>();
    PlaybackTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());

    transmitter.send(StickCommand{0.5, 0.0, -0.25});
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

    transmitter.send(StickCommand{0.4, 0.0, 0.2});
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
    transmitter.send(StickCommand{0.5, 0.0, 0.0});
    const CommandFeedback feedback = transmitter.update();
    EXPECT_FALSE(transmitter.did_init_button_press());
    ASSERT_EQ(feedback.stick_commands.count(FrameId::OUR_ROBOT_1), 1u);
    EXPECT_NEAR(feedback.stick_commands.at(FrameId::OUR_ROBOT_1).forward, 0.5, 1e-9);
}

TEST(SimTransmitterTest, DisableClearsFeedback) {
    SimTransmitterConfiguration config;
    config.init_delay_seconds = 0.0;
    auto clock = std::make_shared<ManualClock>();
    SimTransmitter transmitter(config, clock);
    ASSERT_TRUE(transmitter.initialize());
    transmitter.enable();
    transmitter.send(StickCommand{0.4, 0.0, 0.2});

    transmitter.disable();
    EXPECT_TRUE(transmitter.update().stick_commands.empty());
}

}  // namespace
}  // namespace auto_battlebot
