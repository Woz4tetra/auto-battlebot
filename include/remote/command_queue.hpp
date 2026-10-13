#pragma once

#include <functional>
#include <mutex>
#include <utility>
#include <variant>
#include <vector>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {

/** Every command source (the viz sink, the LVGL UI) posts here from its own thread; the Runner
 *  drains it at the start of each tick, so every command runs on the loop thread. */
class CommandQueue {
   public:
    void post(RemoteCommand command) {
        std::function<void()> interrupt;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (std::holds_alternative<SystemActionCommand>(command)) interrupt = on_system_action_;
            pending_.push_back(std::move(command));
        }
        if (interrupt) interrupt();
    }

    /** Runs on the posting thread when a reboot or power-off arrives, so it can cut short
     *  whatever the loop thread is blocked in (a camera re-open) and get to the queue. */
    void set_on_system_action(std::function<void()> callback) {
        std::lock_guard<std::mutex> lock(mutex_);
        on_system_action_ = std::move(callback);
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
    std::function<void()> on_system_action_;
};

}  // namespace auto_battlebot::remote
