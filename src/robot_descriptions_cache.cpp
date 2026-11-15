#include "robot_descriptions_cache.hpp"

#include "enums/frame_id.hpp"
#include "enums/group.hpp"

namespace auto_battlebot {

void RobotDescriptionsCache::reset() {
    previous_ = RobotDescriptionsStamped{};
    has_previous_ = false;
}

RobotDescriptionsCache::Resolved RobotDescriptionsCache::resolve(
    const RobotDescriptionsStamped &current) {
    if (has_navigation_critical_robots(current)) {
        previous_ = current;
        has_previous_ = true;
        return Resolved{current, false};
    }
    if (has_previous_ && has_navigation_critical_robots(previous_) &&
        current.header.stamp - previous_.header.stamp <= max_age_s_) {
        return Resolved{previous_, true};
    }
    return Resolved{current, false};
}

bool RobotDescriptionsCache::has_our_robot(const RobotDescriptionsStamped &robots) {
    for (const auto &robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) return true;
    }
    return false;
}

bool RobotDescriptionsCache::has_their_robot(const RobotDescriptionsStamped &robots) {
    for (const auto &robot : robots.descriptions) {
        if (robot.group == Group::THEIRS) return true;
    }
    return false;
}

bool RobotDescriptionsCache::has_navigation_critical_robots(
    const RobotDescriptionsStamped &robots) {
    return has_our_robot(robots) && has_their_robot(robots);
}

}  // namespace auto_battlebot
