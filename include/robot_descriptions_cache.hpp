#pragma once

#include "data_structures/robot.hpp"

namespace auto_battlebot {

/**
 * @brief Caches the most recent "navigation-critical" robot set (one that contains our robot AND
 * an opposing robot) and substitutes it when the current frame is missing either side.
 *
 * Owned by Runner so that target selection and navigation operate on the same effective robot
 * set within a tick. Both consumers read the resolved view; without this, target selection would
 * flap to the previous selection while navigation operated on a stale cache.
 */
class RobotDescriptionsCache {
   public:
    /**
     * @brief Non-owning view of the resolved robot set.
     *
     * `robots` references either the `current` argument passed to resolve() or the cache's
     * internal `previous_` snapshot. The reference is valid until the next call to resolve()
     * or reset(), and (for the `current` branch) the lifetime of the caller's argument.
     */
    struct Resolved {
        const RobotDescriptionsStamped &robots;
        bool using_previous;
    };

    void reset();

    /** Update cache with `current` if it is critical, otherwise substitute the cached set. */
    Resolved resolve(const RobotDescriptionsStamped &current);

   private:
    static bool has_our_robot(const RobotDescriptionsStamped &robots);
    static bool has_their_robot(const RobotDescriptionsStamped &robots);
    static bool has_navigation_critical_robots(const RobotDescriptionsStamped &robots);

    RobotDescriptionsStamped previous_;
    bool has_previous_ = false;
};

}  // namespace auto_battlebot
