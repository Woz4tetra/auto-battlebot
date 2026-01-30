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

#include <cstdint>
#include <array>

namespace auto_battlebot
{

/**
 * @brief TCP message types (first byte of each message)
 */
enum class TcpMessageType : uint8_t
{
    /// Frame ready with pose data (Unity → C++)
    FrameReady = 0x01,

    /// Velocity command (C++ → Unity)
    VelocityCommand = 0x02,

    /// Camera intrinsics (Unity → C++)
    CameraIntrinsics = 0x03,

    /// Frame processed acknowledgment (C++ → Unity)
    FrameProcessed = 0x04,

    /// Ping for connection check
    Ping = 0x05,

    /// Pong response to ping
    Pong = 0x06,

    /// Frame ready without depth (Unity → C++)
    FrameReadyNoDepth = 0x07,

    /// Request frame from Unity (C++ → Unity)
    RequestFrame = 0x08,

    /// Shutdown notification
    Shutdown = 0xFF,
};

/**
 * @brief Message sizes in bytes (including 1-byte type header)
 */
namespace TcpMessageSize
{
    constexpr size_t TypeHeader = 1;
    constexpr size_t FrameReady = TypeHeader + 8 + 8 + 128;        // 145 bytes
    constexpr size_t VelocityCommand = TypeHeader + 8 + 8 + 8 + 8; // 33 bytes
    constexpr size_t CameraIntrinsics = TypeHeader + 4 + 4 + 32 + 40; // 81 bytes
    constexpr size_t FrameProcessed = TypeHeader + 8;              // 9 bytes
    constexpr size_t Ping = TypeHeader;                            // 1 byte
    constexpr size_t Pong = TypeHeader;                            // 1 byte
    constexpr size_t RequestFrame = TypeHeader + 1;                // 2 bytes
    constexpr size_t Shutdown = TypeHeader;                        // 1 byte
} // namespace TcpMessageSize

/**
 * @brief Camera intrinsics received from Unity
 *
 * Follows OpenCV/Brown-Conrady camera model convention.
 */
struct TcpCameraIntrinsics
{
    int32_t width;
    int32_t height;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;

    /// Check if intrinsics are valid
    bool is_valid() const
    {
        return width > 0 && height > 0 && fx > 0 && fy > 0;
    }

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 80;
};

/**
 * @brief Frame ready message from Unity
 */
struct TcpFrameReadyMessage
{
    uint64_t frame_id;
    uint64_t timestamp_ns;
    std::array<double, 16> pose; // 4x4 matrix, row-major

    /// Get timestamp in seconds
    double timestamp_seconds() const
    {
        return static_cast<double>(timestamp_ns) / 1'000'000'000.0;
    }

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 8 + 8 + 128;
};

/**
 * @brief Velocity command to send to Unity
 */
struct TcpVelocityCommand
{
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
struct TcpFrameProcessedMessage
{
    uint64_t frame_id;

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 8;
};

/**
 * @brief Frame request message
 */
struct TcpRequestFrameMessage
{
    bool with_depth;

    /// Payload size in bytes (excluding type header)
    static constexpr size_t payload_size = 1;
};

} // namespace auto_battlebot
