#include "ui/system_actions.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace auto_battlebot {

void handle_system_action(UISystemAction action) {
    int rc = 0;
    switch (action) {
        case UISystemAction::REBOOT_HOST:
            rc = std::system("systemctl reboot");
            if (rc != 0) {
                spdlog::warn("systemctl reboot failed (rc={}); trying sudo fallback", rc);
                rc = std::system("sudo reboot");
            }
            if (rc != 0) {
                spdlog::error("Failed to execute reboot command, rc={}", rc);
            }
            break;
        case UISystemAction::POWEROFF_HOST:
            rc = std::system("systemctl poweroff");
            if (rc != 0) {
                spdlog::warn("systemctl poweroff failed (rc={}); trying sudo fallback", rc);
                rc = std::system("sudo shutdown now");
            }
            if (rc != 0) {
                spdlog::error("Failed to execute poweroff command, rc={}", rc);
            }
            break;
        default:
            break;
    }
}

}  // namespace auto_battlebot
