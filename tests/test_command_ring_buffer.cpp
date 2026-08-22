#include <gtest/gtest.h>

#include "robot_filter/command_ring_buffer.hpp"

namespace auto_battlebot {
namespace {

TimedCommand make_command(double stamp, double linear_x) {
    return TimedCommand{stamp, VelocityCommand{linear_x, 0.0, 0.0}};
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
    EXPECT_DOUBLE_EQ(buffer.latest_at(2.0)->command.linear_x, 0.2);
    EXPECT_DOUBLE_EQ(buffer.latest_at(2.5)->command.linear_x, 0.2);
    EXPECT_DOUBLE_EQ(buffer.latest_at(10.0)->command.linear_x, 0.3);
}

TEST(CommandRingBufferTest, WrapAroundDropsOldestKeepsOrder) {
    CommandRingBuffer buffer;
    const size_t total = CommandRingBuffer::kCapacity + 50;
    for (size_t i = 0; i < total; ++i) {
        buffer.push(make_command(static_cast<double>(i), static_cast<double>(i)));
    }
    EXPECT_EQ(buffer.size(), CommandRingBuffer::kCapacity);

    // The oldest surviving entry is total - kCapacity; anything before it is gone.
    const double oldest = static_cast<double>(total - CommandRingBuffer::kCapacity);
    EXPECT_EQ(buffer.latest_at(oldest - 0.5), nullptr);
    ASSERT_NE(buffer.latest_at(oldest), nullptr);
    EXPECT_DOUBLE_EQ(buffer.latest_at(oldest)->command.linear_x, oldest);
    EXPECT_DOUBLE_EQ(buffer.latest_at(1e9)->command.linear_x, static_cast<double>(total - 1));
}

TEST(CommandRingBufferTest, GatherIncludesCommandActiveAtWindowStart) {
    CommandRingBuffer buffer;
    buffer.push(make_command(1.0, 0.1));
    buffer.push(make_command(2.0, 0.2));
    buffer.push(make_command(3.0, 0.3));
    buffer.push(make_command(4.0, 0.4));

    std::vector<TimedCommand> history;
    buffer.gather(2.5, 3.5, history);

    // The command issued at 2.0 is still active when the window opens at 2.5, then the 3.0
    // entry lands inside the window; 4.0 is past the end.
    ASSERT_EQ(history.size(), 2u);
    EXPECT_DOUBLE_EQ(history[0].command.linear_x, 0.2);
    EXPECT_DOUBLE_EQ(history[1].command.linear_x, 0.3);
}

TEST(CommandRingBufferTest, ClearEmptiesBuffer) {
    CommandRingBuffer buffer;
    buffer.push(make_command(1.0, 0.1));
    buffer.clear();
    EXPECT_EQ(buffer.size(), 0u);
    EXPECT_EQ(buffer.latest_at(2.0), nullptr);
}

}  // namespace
}  // namespace auto_battlebot
