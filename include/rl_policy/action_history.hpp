#pragma once

#include <cstddef>
#include <vector>

namespace auto_battlebot::rl_policy {

struct HistoryEntry {
    double action_linear = 0.0;
    double action_angular = 0.0;
    double forward_speed_m_s = 0.0;
    double yaw_rate_rad_s = 0.0;
};

class ActionHistory {
   public:
    explicit ActionHistory(size_t capacity) : entries_(capacity) {}

    size_t capacity() const { return entries_.size(); }

    void push(const HistoryEntry &entry) {
        if (entries_.empty()) return;
        newest_ = (newest_ + entries_.size() - 1) % entries_.size();
        entries_[newest_] = entry;
    }

    const HistoryEntry &entry(size_t steps_ago) const {
        return entries_[(newest_ + steps_ago) % entries_.size()];
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
