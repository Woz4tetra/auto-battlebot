#pragma once

#include "data_structures/robot.hpp"

namespace auto_battlebot {

/**
 * @brief Caches the most recent "navigation-critical" robot set
 * (one that contains our robot AND an opposing robot) and substitutes
 * it when the current frame is missing either side.
 *
 * Designed to be embedded by any NavigationInterface implementation
 * that needs continuity across momentary detection gaps.
 */
class NavigationRobotsCache {
   public:
    struct Result {
        RobotDescriptionsStamped robots;
        bool using_previous = false;
    };

    void reset();

    /** Update cache with `current` if it is critical, otherwise substitute the cached set. */
    Result resolve(const RobotDescriptionsStamped& current);

   private:
    static bool has_our_robot(const RobotDescriptionsStamped& robots);
    static bool has_their_robot(const RobotDescriptionsStamped& robots);
    static bool has_navigation_critical_robots(const RobotDescriptionsStamped& robots);

    RobotDescriptionsStamped previous_;
    bool has_previous_ = false;
};

}  // namespace auto_battlebot
