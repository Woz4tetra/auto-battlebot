#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "robot_filter/plant_model_interface.hpp"

namespace auto_battlebot {

/**
 * Fixed-capacity history of issued commands, oldest overwritten first. 256 entries at 30 Hz
 * covers 8 s, far past any transport delay or retrodiction window, and push() never allocates,
 * so it is safe on the control path. Stamps must be pushed in nondecreasing order.
 */
class CommandRingBuffer {
   public:
    static constexpr size_t kCapacity = 256;

    void clear() {
        size_ = 0;
        start_ = 0;
    }

    size_t size() const { return size_; }

    void push(const TimedCommand &command) {
        if (size_ == kCapacity) {
            buffer_[start_] = command;
            start_ = (start_ + 1) % kCapacity;
        } else {
            buffer_[(start_ + size_) % kCapacity] = command;
            ++size_;
        }
    }

    /**
     * The most recent command issued at or before `stamp`, or nullptr when the buffer is empty
     * or every entry is newer than `stamp`.
     */
    const TimedCommand *latest_at(double stamp) const {
        const TimedCommand *result = nullptr;
        for (size_t i = 0; i < size_; ++i) {
            const TimedCommand &entry = at(i);
            if (entry.stamp > stamp) break;
            result = &entry;
        }
        return result;
    }

    /**
     * Copies, oldest first, the command active at `start_stamp` (the newest entry at or before
     * it, when one exists) and every entry in (start_stamp, end_stamp] into `out`. `out` is
     * cleared first; reuse one scratch vector across calls to avoid per-tick allocation. This is
     * the history a delayed plant model consumes as a span.
     */
    void gather(double start_stamp, double end_stamp, std::vector<TimedCommand> &out) const {
        out.clear();
        for (size_t i = 0; i < size_; ++i) {
            const TimedCommand &entry = at(i);
            if (entry.stamp > end_stamp) break;
            if (entry.stamp <= start_stamp) {
                // Keep only the newest entry at or before the window start.
                if (!out.empty() && out.back().stamp <= start_stamp) {
                    out.back() = entry;
                } else {
                    out.push_back(entry);
                }
            } else {
                out.push_back(entry);
            }
        }
    }

   private:
    const TimedCommand &at(size_t index) const { return buffer_[(start_ + index) % kCapacity]; }

    std::array<TimedCommand, kCapacity> buffer_{};
    size_t start_ = 0;
    size_t size_ = 0;
};

}  // namespace auto_battlebot
