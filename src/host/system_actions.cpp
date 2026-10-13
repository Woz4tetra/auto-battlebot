#include "host/system_actions.hpp"

#include <spdlog/spdlog.h>

#include <cstdlib>

namespace auto_battlebot {

// Neither command may prompt. The app runs headless under systemd, and a prompt for a password or a
// polkit agent waits forever with the loop already stopped and the recordings closed. -n and
// --no-ask-password make a missing rule fail at once, so the error below is logged.
void handle_system_action(SystemAction action) {
    int rc = 0;
    switch (action) {
        case SystemAction::REBOOT_HOST:
            spdlog::warn("Rebooting the host now.");
            rc = std::system("systemctl --no-ask-password reboot");
            if (rc != 0) {
                spdlog::warn("systemctl reboot failed (rc={}); trying sudo fallback", rc);
                rc = std::system("sudo -n reboot");
            }
            if (rc != 0) {
                spdlog::error("Failed to execute reboot command, rc={}", rc);
            }
            break;
        case SystemAction::POWEROFF_HOST:
            spdlog::warn("Powering off the host now.");
            rc = std::system("systemctl --no-ask-password poweroff");
            if (rc != 0) {
                spdlog::warn("systemctl poweroff failed (rc={}); trying sudo fallback", rc);
                rc = std::system("sudo -n shutdown now");
            }
            if (rc != 0) {
                spdlog::error("Failed to execute poweroff command, rc={}", rc);
            }
            break;
    }
}

}  // namespace auto_battlebot
