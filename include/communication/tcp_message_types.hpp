/**
 * @file tcp_message_types.hpp
 * @brief TCP message protocol definitions for Unity-C++ communication
 *
 * This header defines the message types and structures for the TCP protocol
 * used to exchange small data between Unity and the C++ application.
 *
 * Protocol Overview:
 * - All messages are fixed-size for efficient parsing
 * - Little-endian byte order throughout
 * - Unity acts as TCP server, C++ connects as client
 * - Camera intrinsics sent once on connection
 * - Frame data (pose + sync) sent each frame from Unity
 * - Velocity commands sent from C++ to Unity
 */

#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace auto_battlebot {

/**
 * @brief TCP message types (first byte of each message)
 */
enum class TcpMessageType : uint8_t {
    /// Velocity command (C++ → Unity)
    VelocityCommand = 0x01,

    /// Camera intrinsics (Unity → C++)
    CameraIntrinsics = 0x02,

    /// Frame processed acknowledgment (C++ → Unity)
    FrameProcessed = 0x03,

    /// Request frame from Unity (C++ → Unity)
    RequestFrame = 0x04,

    /// Frame ready with raw image data (Unity → C++, fallback mode)
    FrameReadyWithData = 0x05,

    /// Frame ready with raw RGB data only, no depth (Unity → C++, fallback mode)
    FrameReadyWithDataNoDepth = 0x06,

    /// Shutdown notification
    Shutdown = 0xFF,
};

/**
 * @brief Message sizes in bytes (including 1-byte type header)
 */
namespace TcpMessageSize {
constexpr size_t TypeHeader = 1;
constexpr size_t VelocityCommand = TypeHeader + 8 + 8 + 8 + 8;     // 33 bytes
constexpr size_t CameraIntrinsics = TypeHeader + 4 + 4 + 32 + 40;  // 81 bytes
constexpr size_t FrameProcessed = TypeHeader + 8;                  // 9 bytes
constexpr size_t RequestFrame = TypeHeader + 1;                    // 2 bytes
constexpr size_t Shutdown = TypeHeader;                            // 1 byte

/// Header size for FrameReadyWithData (variable-length message)
/// Actual message size = Header + rgb_size + depth_size
constexpr size_t FrameReadyWithDataHeader =
    TypeHeader + 8 + 8 + 128 + 4 + 4 + 4 + 4 + 4 + 4;  // 169 bytes
}  // namespace TcpMessageSize

/**
 * @brief Camera intrinsics received from Unity
 *
 * Follows OpenCV/Brown-Conrady camera model convention.
 */
struct TcpCameraIntrinsics {
    int32_t width;
    int32_t height;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;

    /// Check if intrinsics are valid
    bool is_valid() const { return width > 0 && height > 0 && fx > 0 && fy > 0; }

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 80;
};

/**
 * @brief Frame ready message from Unity
 */
struct TcpFrameReadyMessage {
    uint64_t frame_id;
    uint64_t timestamp_ns;
    std::array<double, 16> pose;  // 4x4 matrix, row-major

    /// Get timestamp in seconds
    double timestamp_seconds() const { return static_cast<double>(timestamp_ns) / 1'000'000'000.0; }

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 8 + 8 + 128;
};

/**
 * @brief Frame ready message with raw image data (fallback mode)
 *
 * Used when CUDA interop is unavailable (e.g., Vulkan backend).
 * Contains the frame metadata plus raw image bytes.
 */
struct TcpFrameReadyWithDataMessage {
    uint64_t frame_id;
    uint64_t timestamp_ns;
    std::array<double, 16> pose;  // 4x4 matrix, row-major

    // Image dimensions
    int32_t rgb_width;
    int32_t rgb_height;
    int32_t depth_width;
    int32_t depth_height;

    // Data sizes
    int32_t rgb_data_size;
    int32_t depth_data_size;

    // Image data (dynamically sized)
    std::vector<uint8_t> rgb_data;
    std::vector<uint8_t> depth_data;

    /// Check if this message has depth data
    bool has_depth() const { return depth_data_size > 0 && !depth_data.empty(); }

    /// Get timestamp in seconds
    double timestamp_seconds() const { return static_cast<double>(timestamp_ns) / 1'000'000'000.0; }

    /// Header payload size in bytes (excluding type header and image data)
    static constexpr size_t header_payload_size = 8 + 8 + 128 + 4 + 4 + 4 + 4 + 4 + 4;
};

/**
 * @brief Velocity command to send to Unity
 */
struct TcpVelocityCommand {
    uint64_t command_id;
    double linear_x;
    double linear_y;
    double angular_z;

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 32;
};

/**
 * @brief Frame processed acknowledgment
 */
struct TcpFrameProcessedMessage {
    uint64_t frame_id;

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 8;
};

/**
 * @brief Frame request message
 */
struct TcpRequestFrameMessage {
    bool with_depth;

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 1;
};

}  // namespace auto_battlebot
