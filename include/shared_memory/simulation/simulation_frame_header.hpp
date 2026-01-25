#pragma once

// Ensure the data layout matches simulation/auto-battlebot-sim/Assets/Scripts/Communication/FrameHeader.cs

#include <cstdint>
#include <cstddef>

namespace auto_battlebot
{
#pragma pack(push, 1) // ensures there's no bit padding in memory
    struct FrameHeader
    {
        uint64_t frame_id;       // 8 bytes (offset 0)
        double timestamp;        // 8 bytes (offset 8)
        int32_t width;           // 4 bytes (offset 16)
        int32_t height;          // 4 bytes (offset 20)
        int32_t rgb_offset;      // 4 bytes (offset 24)
        int32_t depth_offset;    // 4 bytes (offset 28)
        int32_t pose_offset;     // 4 bytes (offset 32)
        int32_t active_buffer;   // 4 bytes (offset 36)
        double fx;               // 8 bytes (offset 40) - focal length X
        double fy;               // 8 bytes (offset 48) - focal length Y
        double cx;               // 8 bytes (offset 56) - principal point X
        double cy;               // 8 bytes (offset 64) - principal point Y
        double k1;               // 8 bytes (offset 72) - radial distortion
        double k2;               // 8 bytes (offset 80) - radial distortion
        double p1;               // 8 bytes (offset 88) - tangential distortion
        double p2;               // 8 bytes (offset 96) - tangential distortion
        double k3;               // 8 bytes (offset 104) - radial distortion
        uint8_t reserved[16];    // 16 bytes (offset 112) - padding to 128 bytes

        static constexpr size_t SIZE = 128;
    };
#pragma pack(pop)

    inline size_t get_simulation_frame_size(int expected_width, int expected_height)
    {
        size_t rgb_size = expected_width * expected_height * 3;
        size_t depth_size = expected_width * expected_height * sizeof(float);
        size_t pose_size = 128; // 4x4 double matrix
        return FrameHeader::SIZE + rgb_size + depth_size + pose_size;
    }
}
