#include <gtest/gtest.h>

#include "rl_policy/action_history.hpp"
#include "rl_policy/allocation_counter.hpp"

namespace auto_battlebot::rl_policy {
namespace {

constexpr size_t kCapacity = 4;

void expect_zero(const HistoryEntry &entry, size_t steps_ago) {
    EXPECT_EQ(entry.action_linear, 0.0) << "steps_ago " << steps_ago;
    EXPECT_EQ(entry.action_angular, 0.0) << "steps_ago " << steps_ago;
    EXPECT_EQ(entry.forward_speed_m_s, 0.0) << "steps_ago " << steps_ago;
    EXPECT_EQ(entry.yaw_rate_rad_s, 0.0) << "steps_ago " << steps_ago;
}

TEST(ActionHistoryTest, UnwrittenSlotsAreZero) {
    ActionHistory history(kCapacity);
    history.push(HistoryEntry{0.5, -0.5, 1.0, 2.0});
    EXPECT_EQ(history.entry(0).action_linear, 0.5);
    for (size_t steps_ago = 1; steps_ago < kCapacity; ++steps_ago) {
        expect_zero(history.entry(steps_ago), steps_ago);
    }
}

TEST(ActionHistoryTest, ClearZeroesEverySlot) {
    ActionHistory history(kCapacity);
    for (int i = 1; i <= 6; ++i) history.push(HistoryEntry{0.1 * i, 0.1 * i, 0.1 * i, 0.1 * i});
    history.clear();
    for (size_t steps_ago = 0; steps_ago < kCapacity; ++steps_ago) {
        expect_zero(history.entry(steps_ago), steps_ago);
    }
}

TEST(ActionHistoryTest, NewestFirstAndOldestFallsOffTheEnd) {
    ActionHistory history(kCapacity);
    for (int i = 1; i <= 5; ++i) history.push(HistoryEntry{static_cast<double>(i), 0.0, 0.0, 0.0});
    for (size_t steps_ago = 0; steps_ago < kCapacity; ++steps_ago) {
        EXPECT_EQ(history.entry(steps_ago).action_linear, 5.0 - static_cast<double>(steps_ago));
    }
}

TEST(ActionHistoryTest, PushAndEntryDoNotAllocate) {
    ActionHistory history(kCapacity);
    double sum = 0.0;
    const long allocations = rl_test::allocations_during([&] {
        for (int i = 0; i < 100; ++i) {
            history.push(HistoryEntry{1.0, 1.0, 1.0, 1.0});
            sum += history.entry(1).action_linear;
        }
    });
    EXPECT_EQ(allocations, 0);
    EXPECT_GT(sum, 0.0);
}

}  // namespace
}  // namespace auto_battlebot::rl_policy
