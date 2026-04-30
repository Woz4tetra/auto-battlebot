#include "robot_descriptions_cache.hpp"

#include <set>

#include "enums/frame_id.hpp"

namespace auto_battlebot {

void RobotDescriptionsCache::reset() {
    last_per_frame_id_.clear();
    merged_ = RobotDescriptionsStamped{};
}

RobotDescriptionsCache::Resolved RobotDescriptionsCache::resolve(
    const RobotDescriptionsStamped &current) {
    std::set<FrameId> seen_this_frame;
    for (const auto &desc : current.descriptions) {
        if (desc.frame_id == FrameId::EMPTY) continue;
        seen_this_frame.insert(desc.frame_id);
        RobotDescription stored = desc;
        stored.is_stale = false;
        last_per_frame_id_[desc.frame_id] = std::move(stored);
    }

    bool using_previous = false;
    for (auto &[frame_id, desc] : last_per_frame_id_) {
        const bool fresh = seen_this_frame.count(frame_id) != 0;
        desc.is_stale = !fresh;
        if (!fresh) using_previous = true;
    }

    merged_.header = current.header;
    merged_.descriptions.clear();
    merged_.descriptions.reserve(last_per_frame_id_.size() + current.descriptions.size());
    for (const auto &[frame_id, desc] : last_per_frame_id_) {
        merged_.descriptions.push_back(desc);
    }
    // Preserve any FrameId::EMPTY descriptions verbatim so we don't silently drop callers'
    // unassigned entries; they bypass the cache because they have no key.
    for (const auto &desc : current.descriptions) {
        if (desc.frame_id == FrameId::EMPTY) merged_.descriptions.push_back(desc);
    }

    return Resolved{merged_, using_previous};
}

}  // namespace auto_battlebot
