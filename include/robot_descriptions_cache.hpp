#pragma once

#include "data_structures/robot.hpp"

namespace auto_battlebot {

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

    explicit RobotDescriptionsCache(double max_age_s) : max_age_s_(max_age_s) {}

    void reset();

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
