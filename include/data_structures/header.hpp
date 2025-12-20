#pragma once

#include <string>
#include "enums/frame_id.hpp"

namespace auto_battlebot
{
    struct Header
    {
        double timestamp = 0.0;
        FrameId frame_id = FrameId::EMPTY;
    };

} // namespace auto_battlebot
