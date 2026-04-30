#pragma once

#include <map>

#include "data_structures/robot.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot {

/**
 * @brief Per-FrameId hold-last cache for robot descriptions.
 *
 * Each call to resolve() updates an internal per-FrameId snapshot with this frame's
 * descriptions, then returns a merged view that contains one entry per ever-seen FrameId.
 * FrameIds detected this frame appear with their fresh measurement and is_stale=false;
 * FrameIds not detected this frame appear with their last cached measurement and
 * is_stale=true. This lets target selection and navigation see every robot that has ever
 * been detected, bridging momentary detection gaps without requiring downstream code to
 * track its own history.
 *
 * Owned by Runner so target selection and navigation operate on the same merged robot set
 * within a tick.
 */
class RobotDescriptionsCache {
   public:
    /**
     * @brief Non-owning view of the resolved (merged) robot set.
     *
     * `robots` references the cache's internal storage and is valid until the next call to
     * resolve() or reset(). `using_previous` is true iff at least one FrameId in the merged
     * view was substituted from the cache (i.e. not measured this frame).
     */
    struct Resolved {
        const RobotDescriptionsStamped &robots;
        bool using_previous;
    };

    void reset();

    /**
     * Update the per-FrameId cache with `current`'s descriptions and return the merged view.
     * The merged header takes its frame_id and stamp from `current`. Descriptions with
     * FrameId::EMPTY in `current` are passed through to the merged view but not cached.
     */
    Resolved resolve(const RobotDescriptionsStamped &current);

   private:
    std::map<FrameId, RobotDescription> last_per_frame_id_;
    RobotDescriptionsStamped merged_;
};

}  // namespace auto_battlebot
