#include <gtest/gtest.h>

#include "robot_filter/command_ring_buffer.hpp"

namespace auto_battlebot {
namespace {

TimedCommand make_command(double stamp, double linear_x) {
    return TimedCommand{stamp, StickCommand{linear_x, 0.0, 0.0}};
}

TEST(CommandRingBufferTest, EmptyBufferReturnsNull) {
    CommandRingBuffer buffer;
    EXPECT_EQ(buffer.latest_at(1.0), nullptr);
    EXPECT_EQ(buffer.size(), 0u);
}

TEST(CommandRingBufferTest, LatestAtPicksNewestAtOrBefore) {
    CommandRingBuffer buffer;
    buffer.push(make_command(1.0, 0.1));
    buffer.push(make_command(2.0, 0.2));
    buffer.push(make_command(3.0, 0.3));

    EXPECT_EQ(buffer.latest_at(0.5), nullptr);
    ASSERT_NE(buffer.latest_at(2.0), nullptr);
    EXPECT_DOUBLE_EQ(buffer.latest_at(2.0)->command.forward, 0.2);
    EXPECT_DOUBLE_EQ(buffer.latest_at(2.5)->command.forward, 0.2);
    EXPECT_DOUBLE_EQ(buffer.latest_at(10.0)->command.forward, 0.3);
}

TEST(CommandRingBufferTest, WrapAroundDropsOldestKeepsOrder) {
    CommandRingBuffer buffer;
    const size_t total = CommandRingBuffer::kCapacity + 50;
    for (size_t i = 0; i < total; ++i) {
        buffer.push(make_command(static_cast<double>(i), static_cast<double>(i)));
    }
    EXPECT_EQ(buffer.size(), CommandRingBuffer::kCapacity);

    const double oldest = static_cast<double>(total - CommandRingBuffer::kCapacity);
    EXPECT_EQ(buffer.latest_at(oldest - 0.5), nullptr);
    ASSERT_NE(buffer.latest_at(oldest), nullptr);
    EXPECT_DOUBLE_EQ(buffer.latest_at(oldest)->command.forward, oldest);
    EXPECT_DOUBLE_EQ(buffer.latest_at(1e9)->command.forward, static_cast<double>(total - 1));
}

TEST(CommandRingBufferTest, GatherIncludesCommandActiveAtWindowStart) {
    CommandRingBuffer buffer;
    buffer.push(make_command(1.0, 0.1));
    buffer.push(make_command(2.0, 0.2));
    buffer.push(make_command(3.0, 0.3));
    buffer.push(make_command(4.0, 0.4));

    std::vector<TimedCommand> history;
    buffer.gather(2.5, 3.5, history);

    ASSERT_EQ(history.size(), 2u);
    EXPECT_DOUBLE_EQ(history[0].command.forward, 0.2);
    EXPECT_DOUBLE_EQ(history[1].command.forward, 0.3);
}

TEST(CommandRingBufferTest, GatherClearsOutputFirst) {
    CommandRingBuffer buffer;
    buffer.push(make_command(1.0, 0.1));
    buffer.push(make_command(2.0, 0.2));

    std::vector<TimedCommand> history = {make_command(9.0, 9.0), make_command(9.5, 9.5)};
    buffer.gather(0.5, 2.5, history);

    ASSERT_EQ(history.size(), 2u);
    EXPECT_DOUBLE_EQ(history[0].command.forward, 0.1);
    EXPECT_DOUBLE_EQ(history[1].command.forward, 0.2);
}

TEST(CommandRingBufferTest, LatestAtAllNewerIsNull) {
    CommandRingBuffer buffer;
    buffer.push(make_command(5.0, 0.5));
    buffer.push(make_command(6.0, 0.6));
    EXPECT_EQ(buffer.latest_at(4.9), nullptr);
}

TEST(CommandRingBufferTest, ClearEmptiesBuffer) {
    CommandRingBuffer buffer;
    buffer.push(make_command(1.0, 0.1));
    buffer.clear();
    EXPECT_EQ(buffer.size(), 0u);
    EXPECT_EQ(buffer.latest_at(2.0), nullptr);
}

TEST(CommandRingBufferTest, DecreasingStampIsRejected) {
    CommandRingBuffer buffer;
    EXPECT_TRUE(buffer.push(make_command(2.0, 0.2)));
    EXPECT_FALSE(buffer.push(make_command(1.0, 0.1)));
    EXPECT_EQ(buffer.size(), 1u);
    const TimedCommand *latest = buffer.latest_at(3.0);
    ASSERT_NE(latest, nullptr);
    EXPECT_DOUBLE_EQ(latest->stamp, 2.0);
}

TEST(CommandRingBufferTest, EqualStampIsAccepted) {
    CommandRingBuffer buffer;
    EXPECT_TRUE(buffer.push(make_command(2.0, 0.2)));
    EXPECT_TRUE(buffer.push(make_command(2.0, 0.3)));
    EXPECT_EQ(buffer.size(), 2u);
}

}  // namespace
}  // namespace auto_battlebot
