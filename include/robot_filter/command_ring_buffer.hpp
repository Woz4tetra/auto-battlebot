#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include "plant/plant_model_interface.hpp"

namespace auto_battlebot {

class CommandRingBuffer {
   public:
    static constexpr size_t kCapacity = 256;

    void clear() {
        size_ = 0;
        start_ = 0;
    }

    size_t size() const { return size_; }

    bool push(const TimedCommand &command) {
        if (size_ > 0 && command.stamp < at(size_ - 1).stamp) return false;
        if (size_ == kCapacity) {
            buffer_[start_] = command;
            start_ = (start_ + 1) % kCapacity;
        } else {
            buffer_[(start_ + size_) % kCapacity] = command;
            ++size_;
        }
        return true;
    }

    const TimedCommand *latest_at(double stamp) const {
        const TimedCommand *result = nullptr;
        for (size_t i = 0; i < size_; ++i) {
            const TimedCommand &entry = at(i);
            if (entry.stamp > stamp) break;
            result = &entry;
        }
        return result;
    }

    void gather(double start_stamp, double end_stamp, std::vector<TimedCommand> &out) const {
        out.clear();
        for (size_t i = 0; i < size_; ++i) {
            const TimedCommand &entry = at(i);
            if (entry.stamp > end_stamp) break;
            if (entry.stamp <= start_stamp) {
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
