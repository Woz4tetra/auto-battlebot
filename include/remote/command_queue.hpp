#pragma once

#include <functional>
#include <mutex>
#include <utility>
#include <variant>
#include <vector>

#include "remote/protocol.hpp"

namespace auto_battlebot::remote {

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
