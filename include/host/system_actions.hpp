#pragma once

#include <string>
#include <vector>

#include "enums/system_action.hpp"

namespace auto_battlebot {

std::vector<std::string> system_action_commands(SystemAction action);

void handle_system_action(SystemAction action);

}  // namespace auto_battlebot
