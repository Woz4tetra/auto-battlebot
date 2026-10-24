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
 *
 * The substitute expires max_age_s after its own stamp ([control_loop] robot_cache_max_age_s). Past
 * that the current set goes through as-is, so a navigator sees our robot is gone and runs its own
 * no-robot path. Replaying the frozen snapshot forever kept RlPolicyNavigation feeding the policy
 * one unchanging observation, and it held a full-rate spin that kept the robot invisible to the
 * camera.
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

    /** @param max_age_s Longest gap (seconds, by header stamp) the cached set may stand in for. */
    explicit RobotDescriptionsCache(double max_age_s) : max_age_s_(max_age_s) {}

    void reset();

    /** Update cache with `current` if it is critical, otherwise substitute the cached set. */
    Resolved resolve(const RobotDescriptionsStamped &current);

   private:
    static bool has_our_robot(const RobotDescriptionsStamped &robots);
    static bool has_their_robot(const RobotDescriptionsStamped &robots);
    static bool has_navigation_critical_robots(const RobotDescriptionsStamped &robots);

    double max_age_s_;
    RobotDescriptionsStamped previous_;
    bool has_previous_ = false;
};

}  // namespace auto_battlebot
