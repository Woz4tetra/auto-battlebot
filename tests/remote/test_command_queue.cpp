#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include "remote/command_queue.hpp"

namespace auto_battlebot::remote {
namespace {

TEST(CommandQueueTest, PostedCommandsSurfaceOnTheDrainingThreadInOrder) {
    CommandQueue queue;
    std::thread poster([&] {
        queue.post(SetOpponentCountCommand{2});
        queue.post(SetOpponentCountCommand{3});
    });
    poster.join();

    const std::vector<RemoteCommand> drained = queue.drain();
    ASSERT_EQ(drained.size(), 2u);
    EXPECT_EQ(std::get<SetOpponentCountCommand>(drained[0]).count, 2);
    EXPECT_EQ(std::get<SetOpponentCountCommand>(drained[1]).count, 3);
    EXPECT_TRUE(queue.drain().empty());
}

TEST(CommandQueueTest, SystemActionInvokesInterruptOnPostingThread) {
    CommandQueue queue;
    std::thread::id interrupt_thread;
    std::atomic<int> interrupts{0};
    queue.set_on_system_action([&] {
        interrupt_thread = std::this_thread::get_id();
        ++interrupts;
    });

    queue.post(SetOpponentCountCommand{2});
    EXPECT_EQ(interrupts, 0);

    std::thread::id poster_id;
    std::thread poster([&] {
        poster_id = std::this_thread::get_id();
        queue.post(SystemActionCommand{});
    });
    poster.join();

    EXPECT_EQ(interrupts, 1);
    EXPECT_EQ(interrupt_thread, poster_id);
    EXPECT_NE(interrupt_thread, std::this_thread::get_id());
    EXPECT_EQ(queue.drain().size(), 2u);
}

}  // namespace
}  // namespace auto_battlebot::remote
