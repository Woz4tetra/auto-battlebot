#pragma once

#include <mutex>
#include <utility>
#include <vector>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {

/** Every command source (the viz sink, the LVGL UI) posts here from its own thread; the Runner
 *  drains it at the start of each tick, so every command runs on the loop thread. */
class CommandQueue {
   public:
    void post(RemoteCommand command) {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(std::move(command));
    }

    std::vector<RemoteCommand> drain() {
        std::vector<RemoteCommand> out;
        std::lock_guard<std::mutex> lock(mutex_);
        out.swap(pending_);
        return out;
    }

   private:
    std::mutex mutex_;
    std::vector<RemoteCommand> pending_;
};

}  // namespace auto_battlebot::remote
