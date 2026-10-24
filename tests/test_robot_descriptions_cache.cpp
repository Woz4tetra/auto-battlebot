// RobotDescriptionsCache: substitutes the last set with both sides, but only for a short gap.

#include <gtest/gtest.h>

#include "enums/frame_id.hpp"
#include "enums/group.hpp"
#include "robot_descriptions_cache.hpp"

namespace auto_battlebot {
namespace {

constexpr double kMaxAgeS = 0.25;

RobotDescription robot(FrameId frame_id, Group group) {
    RobotDescription description;
    description.frame_id = frame_id;
    description.group = group;
    return description;
}

RobotDescriptionsStamped both_sides(double stamp) {
    RobotDescriptionsStamped robots;
    robots.header.stamp = stamp;
    robots.descriptions = {robot(FrameId::OUR_ROBOT_1, Group::OURS),
                           robot(FrameId::THEIR_ROBOT_1, Group::THEIRS)};
    return robots;
}

RobotDescriptionsStamped opponent_only(double stamp) {
    RobotDescriptionsStamped robots;
    robots.header.stamp = stamp;
    robots.descriptions = {robot(FrameId::THEIR_ROBOT_1, Group::THEIRS)};
    return robots;
}

}  // namespace

TEST(RobotDescriptionsCacheTest, SubstitutesTheLastFullSetForAShortGap) {
    RobotDescriptionsCache cache(kMaxAgeS);
    const auto full = both_sides(1.0);
    EXPECT_FALSE(cache.resolve(full).using_previous);

    const auto gap = opponent_only(1.0 + kMaxAgeS);
    const auto resolved = cache.resolve(gap);
    EXPECT_TRUE(resolved.using_previous);
    EXPECT_EQ(resolved.robots.descriptions.size(), 2U);
}

TEST(RobotDescriptionsCacheTest, PassesTheCurrentSetThroughOnceTheGapOutlastsTheCache) {
    RobotDescriptionsCache cache(kMaxAgeS);
    const auto full = both_sides(1.0);
    cache.resolve(full);

    const auto gap = opponent_only(1.0 + kMaxAgeS + 0.01);
    const auto resolved = cache.resolve(gap);
    EXPECT_FALSE(resolved.using_previous);
    ASSERT_EQ(resolved.robots.descriptions.size(), 1U);
    EXPECT_EQ(resolved.robots.descriptions[0].frame_id, FrameId::THEIR_ROBOT_1);

    // A fresh full set refills the cache.
    const auto again = both_sides(2.0);
    cache.resolve(again);
    EXPECT_TRUE(cache.resolve(opponent_only(2.1)).using_previous);
}

}  // namespace auto_battlebot
