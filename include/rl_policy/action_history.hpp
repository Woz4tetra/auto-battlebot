#pragma once

#include <cstddef>
#include <vector>

namespace auto_battlebot::rl_policy {

/** What one policy step leaves in the history: the action it took and our forward speed and yaw
 * rate as measured at that step. */
struct HistoryEntry {
    double action_linear = 0.0;
    double action_angular = 0.0;
    /** m/s, along our heading. */
    double measured_speed = 0.0;
    /** rad/s. */
    double measured_yaw_rate = 0.0;
};

/**
 * Fixed-size ring of the last policy steps, newest first. Allocated once; push() and entry() do
 * not allocate. Slots never written read as zeros, which is also how the training environment
 * starts an episode.
 */
class ActionHistory {
   public:
    explicit ActionHistory(size_t capacity) : entries_(capacity) {}

    size_t capacity() const { return entries_.size(); }

    void push(const HistoryEntry &entry) {
        if (entries_.empty()) return;
        newest_ = (newest_ + entries_.size() - 1) % entries_.size();
        entries_[newest_] = entry;
    }

    /** `age` 0 is the most recent step. */
    const HistoryEntry &entry(size_t age) const {
        return entries_[(newest_ + age) % entries_.size()];
    }

    void clear() {
        for (auto &entry : entries_) entry = HistoryEntry{};
        newest_ = 0;
    }

   private:
    std::vector<HistoryEntry> entries_;
    size_t newest_ = 0;
};

}  // namespace auto_battlebot::rl_policy
