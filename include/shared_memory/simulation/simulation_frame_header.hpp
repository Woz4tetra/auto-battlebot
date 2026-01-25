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
        int32_t intrinsics_fx_offset;
        int32_t intrinsics_fy_offset;
        int32_t intrinsics_cx_offset;
        int32_t intrinsics_cy_offset;
        int32_t distortion_k1_offset;
        int32_t distortion_k2_offset;
        int32_t distortion_p1_offset;
        int32_t distortion_p2_offset;
        int32_t distortion_k3_offset;
        uint8_t reserved[16]; // Reserved for future use and padding to 128 bytes.

        static constexpr size_t SIZE = 128;
    };
#pragma pack(pop)
}
