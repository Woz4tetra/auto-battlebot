#pragma once

// Ensure the data layout matches simulation/auto-battlebot-sim/Assets/Scripts/Communication/VelocityCommand.cs

#include <cstdint>
#include <cstddef>

namespace auto_battlebot
{
#pragma pack(push, 1) // ensures there's no bit padding in memory
    struct VelocityCommand
    {
        uint64_t command_id;
        double linear_x;
        double linear_y;
        double angular_z;

        static constexpr size_t SIZE = 32;
    };
#pragma pack(pop)
}
