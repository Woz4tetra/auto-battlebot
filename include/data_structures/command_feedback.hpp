#pragma once

#include <map>
#include "data_structures/velocity.hpp"
#include "enums/frame_id.hpp"

namespace auto_battlebot
{
    struct CommandFeedback
    {
        std::map<FrameId, VelocityCommand> commands = std::map<FrameId, VelocityCommand>();
    };

} // namespace auto_battlebot
