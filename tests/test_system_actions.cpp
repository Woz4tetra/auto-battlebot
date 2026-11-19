#include <gtest/gtest.h>

#include "host/system_actions.hpp"

namespace auto_battlebot {

TEST(SystemActionsTest, CommandsAreNonInteractive) {
    for (const SystemAction action : {SystemAction::REBOOT_HOST, SystemAction::POWEROFF_HOST}) {
        const auto commands = system_action_commands(action);
        ASSERT_FALSE(commands.empty());
        for (const std::string &command : commands) {
            const bool non_interactive = command.find("--no-ask-password") != std::string::npos ||
                                         command.rfind("sudo -n ", 0) == 0;
            EXPECT_TRUE(non_interactive) << command;
        }
    }
}

}  // namespace auto_battlebot
