#include "navigation/navigation_robots_cache.hpp"

#include "enums/frame_id.hpp"
#include "enums/group.hpp"

namespace auto_battlebot {

void NavigationRobotsCache::reset() {
    previous_ = RobotDescriptionsStamped{};
    has_previous_ = false;
}

NavigationRobotsCache::Result NavigationRobotsCache::resolve(
    const RobotDescriptionsStamped& current) {
    if (has_navigation_critical_robots(current)) {
        previous_ = current;
        has_previous_ = true;
        return Result{current, false};
    }
    if (has_previous_ && has_navigation_critical_robots(previous_)) {
        return Result{previous_, true};
    }
    return Result{current, false};
}

bool NavigationRobotsCache::has_our_robot(const RobotDescriptionsStamped& robots) {
    for (const auto& robot : robots.descriptions) {
        if (robot.frame_id == FrameId::OUR_ROBOT_1) return true;
    }
    return false;
}

bool NavigationRobotsCache::has_their_robot(const RobotDescriptionsStamped& robots) {
    for (const auto& robot : robots.descriptions) {
        if (robot.group == Group::THEIRS) return true;
    }
    return false;
}

bool NavigationRobotsCache::has_navigation_critical_robots(
    const RobotDescriptionsStamped& robots) {
    return has_our_robot(robots) && has_their_robot(robots);
}

}  // namespace auto_battlebot
