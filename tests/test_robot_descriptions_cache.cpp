#include <gtest/gtest.h>

#include "robot_descriptions_cache.hpp"

namespace auto_battlebot {
namespace {

RobotDescription make_robot(FrameId frame_id, Group group, double x, double y) {
    RobotDescription desc;
    desc.frame_id = frame_id;
    desc.label = Label::OPPONENT;
    desc.group = group;
    desc.pose.position.x = x;
    desc.pose.position.y = y;
    desc.is_stale = false;
    return desc;
}

RobotDescriptionsStamped make_stamped(double stamp,
                                      std::vector<RobotDescription> descriptions = {}) {
    RobotDescriptionsStamped stamped;
    stamped.header.frame_id = FrameId::FIELD;
    stamped.header.stamp = stamp;
    stamped.descriptions = std::move(descriptions);
    return stamped;
}

const RobotDescription *find_by_frame_id(const RobotDescriptionsStamped &robots, FrameId fid) {
    for (const auto &d : robots.descriptions) {
        if (d.frame_id == fid) return &d;
    }
    return nullptr;
}

}  // namespace

TEST(RobotDescriptionsCacheTest, EmptyResolveReturnsEmpty) {
    RobotDescriptionsCache cache;
    auto resolved = cache.resolve(make_stamped(1.0));
    EXPECT_TRUE(resolved.robots.descriptions.empty());
    EXPECT_FALSE(resolved.using_previous);
    EXPECT_DOUBLE_EQ(resolved.robots.header.stamp, 1.0);
}

TEST(RobotDescriptionsCacheTest, FreshDetectionsArePassedThroughNotStale) {
    RobotDescriptionsCache cache;
    auto resolved = cache.resolve(
        make_stamped(1.0, {make_robot(FrameId::OUR_ROBOT_1, Group::OURS, 0.1, 0.2),
                           make_robot(FrameId::THEIR_ROBOT_1, Group::THEIRS, 0.5, 0.5)}));

    ASSERT_EQ(resolved.robots.descriptions.size(), 2u);
    EXPECT_FALSE(resolved.using_previous);
    const auto *ours = find_by_frame_id(resolved.robots, FrameId::OUR_ROBOT_1);
    const auto *theirs = find_by_frame_id(resolved.robots, FrameId::THEIR_ROBOT_1);
    ASSERT_NE(ours, nullptr);
    ASSERT_NE(theirs, nullptr);
    EXPECT_FALSE(ours->is_stale);
    EXPECT_FALSE(theirs->is_stale);
}

// The whole point of the cache: when this frame is missing a robot, the previously seen
// description is held with is_stale=true so navigation/target selection still see it.
TEST(RobotDescriptionsCacheTest, MissingFrameIdSubstitutedFromCacheAsStale) {
    RobotDescriptionsCache cache;
    cache.resolve(make_stamped(1.0, {make_robot(FrameId::OUR_ROBOT_1, Group::OURS, 0.1, 0.2),
                                     make_robot(FrameId::THEIR_ROBOT_1, Group::THEIRS, 0.5, 0.5)}));

    auto resolved = cache.resolve(
        make_stamped(1.1, {make_robot(FrameId::OUR_ROBOT_1, Group::OURS, 0.15, 0.25)}));

    ASSERT_EQ(resolved.robots.descriptions.size(), 2u);
    EXPECT_TRUE(resolved.using_previous);

    const auto *ours = find_by_frame_id(resolved.robots, FrameId::OUR_ROBOT_1);
    const auto *theirs = find_by_frame_id(resolved.robots, FrameId::THEIR_ROBOT_1);
    ASSERT_NE(ours, nullptr);
    ASSERT_NE(theirs, nullptr);
    EXPECT_FALSE(ours->is_stale);
    EXPECT_DOUBLE_EQ(ours->pose.position.x, 0.15);
    EXPECT_TRUE(theirs->is_stale);
    EXPECT_DOUBLE_EQ(theirs->pose.position.x, 0.5);
}

// is_stale must clear once a previously-cached FrameId is observed again, not be sticky.
TEST(RobotDescriptionsCacheTest, StaleFlagClearsWhenFrameIdReappears) {
    RobotDescriptionsCache cache;
    cache.resolve(make_stamped(1.0, {make_robot(FrameId::THEIR_ROBOT_1, Group::THEIRS, 0.5, 0.5)}));
    cache.resolve(make_stamped(1.1));

    auto resolved = cache.resolve(
        make_stamped(1.2, {make_robot(FrameId::THEIR_ROBOT_1, Group::THEIRS, 0.7, 0.7)}));

    ASSERT_EQ(resolved.robots.descriptions.size(), 1u);
    EXPECT_FALSE(resolved.using_previous);
    const auto *theirs = find_by_frame_id(resolved.robots, FrameId::THEIR_ROBOT_1);
    ASSERT_NE(theirs, nullptr);
    EXPECT_FALSE(theirs->is_stale);
    EXPECT_DOUBLE_EQ(theirs->pose.position.x, 0.7);
}

TEST(RobotDescriptionsCacheTest, ResolvePersistsCacheAcrossEmptyFrames) {
    RobotDescriptionsCache cache;
    cache.resolve(make_stamped(1.0, {make_robot(FrameId::OUR_ROBOT_1, Group::OURS, 0.1, 0.2)}));
    cache.resolve(make_stamped(1.1));
    auto resolved = cache.resolve(make_stamped(1.2));

    ASSERT_EQ(resolved.robots.descriptions.size(), 1u);
    EXPECT_TRUE(resolved.using_previous);
    EXPECT_TRUE(resolved.robots.descriptions[0].is_stale);
}

TEST(RobotDescriptionsCacheTest, ResetClearsAllState) {
    RobotDescriptionsCache cache;
    cache.resolve(make_stamped(1.0, {make_robot(FrameId::OUR_ROBOT_1, Group::OURS, 0.1, 0.2),
                                     make_robot(FrameId::THEIR_ROBOT_1, Group::THEIRS, 0.5, 0.5)}));

    cache.reset();
    auto resolved = cache.resolve(make_stamped(2.0));
    EXPECT_TRUE(resolved.robots.descriptions.empty());
    EXPECT_FALSE(resolved.using_previous);
}

// Ensures EMPTY-frame_id descriptions aren't silently dropped (and don't poison the cache).
TEST(RobotDescriptionsCacheTest, EmptyFrameIdDescriptionsBypassCache) {
    RobotDescriptionsCache cache;
    auto first =
        cache.resolve(make_stamped(1.0, {make_robot(FrameId::EMPTY, Group::THEIRS, 0.0, 0.0)}));
    ASSERT_EQ(first.robots.descriptions.size(), 1u);
    EXPECT_EQ(first.robots.descriptions[0].frame_id, FrameId::EMPTY);
    EXPECT_FALSE(first.using_previous);

    auto second = cache.resolve(make_stamped(1.1));
    EXPECT_TRUE(second.robots.descriptions.empty());
    EXPECT_FALSE(second.using_previous);
}

}  // namespace auto_battlebot
