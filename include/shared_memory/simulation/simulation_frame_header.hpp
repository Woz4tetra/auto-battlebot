#pragma once

// Ensure the data layout matches simulation/auto-battlebot-sim/Assets/Scripts/Communication/FrameHeader.cs

#include <cstdint>
#include <cstddef>

namespace auto_battlebot
{
#pragma pack(push, 1) // ensures there's no bit padding in memory
    struct FrameHeader
    {
        uint64_t frame_id;
        double timestamp;
        int32_t width;
        int32_t height;
        int32_t rgb_offset;
        int32_t depth_offset;
        int32_t pose_offset;
        int32_t active_buffer;
        uint8_t reserved[24]; // Reserved for future use and padding to 64 bytes.

        static constexpr size_t SIZE = 64;
    };
#pragma pack(pop)
}
