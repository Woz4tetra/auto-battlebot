#pragma once

#include <map>

#include "data_structures/velocity.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot {

struct CommandFeedback {
    std::map<FrameId, StickCommand> stick_commands = std::map<FrameId, StickCommand>();
};

}  // namespace auto_battlebot
