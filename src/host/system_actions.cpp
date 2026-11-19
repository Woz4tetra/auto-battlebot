#include "host/system_actions.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace auto_battlebot {

std::vector<std::string> system_action_commands(SystemAction action) {
    switch (action) {
        case SystemAction::REBOOT_HOST:
            return {"systemctl --no-ask-password reboot", "sudo -n reboot"};
        case SystemAction::POWEROFF_HOST:
            return {"systemctl --no-ask-password poweroff", "sudo -n shutdown now"};
    }
    return {};
}

void handle_system_action(SystemAction action) {
    const char *name = action == SystemAction::REBOOT_HOST ? "reboot" : "poweroff";
    spdlog::warn("Running host {} now.", name);
    int rc = 0;
    bool first = true;
    for (const std::string &command : system_action_commands(action)) {
        if (!first) {
            spdlog::warn("systemctl {} failed (rc={}); trying sudo fallback", name, rc);
        }
        first = false;
        rc = std::system(command.c_str());
        if (rc == 0) {
            return;
        }
    }
    spdlog::error("Failed to execute {} command, rc={}", name, rc);
}

}  // namespace auto_battlebot
