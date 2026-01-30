/**
 * @file sim_tcp_client.hpp
 * @brief TCP client for communication with Unity simulation
 *
 * This class handles TCP socket communication with the Unity simulation
 * for exchanging small data like pose, velocity commands, and camera intrinsics.
 * Large data (images) is handled separately via CUDA Interop.
 */

#pragma once

#include <string>
#include <optional>
#include <chrono>
#include <mutex>
#include <atomic>

#include "communication/tcp_message_types.hpp"
#include "data_structures/velocity.hpp"
#include "diagnostics_logger/diagnostics_logger.hpp"

namespace auto_battlebot
{

/**
 * @brief Configuration for SimTcpClient
 */
struct SimTcpClientConfig
{
    std::string host = "127.0.0.1";
    int port = 18707;
    int connect_timeout_ms = 5000;
    int read_timeout_ms = 100;
    bool auto_reconnect = true;
    int reconnect_interval_ms = 1000;
};

/**
 * @brief TCP client for Unity simulation communication
 *
 * Responsibilities:
 * - Connect to Unity TCP server
 * - Receive camera intrinsics on connection
 * - Receive frame-ready messages with pose data
 * - Send velocity commands
 * - Send frame-processed acknowledgments
 * - Handle reconnection on disconnect
 */
class SimTcpClient
{
public:
    /**
     * @brief Construct a new SimTcpClient
     * @param config Client configuration
     */
    explicit SimTcpClient(const SimTcpClientConfig& config);

    /**
     * @brief Destructor - closes connection
     */
    ~SimTcpClient();

    // Non-copyable
    SimTcpClient(const SimTcpClient&) = delete;
    SimTcpClient& operator=(const SimTcpClient&) = delete;

    /**
     * @brief Connect to the Unity server
     * @return true if connection succeeded
     */
    bool connect();

    /**
     * @brief Disconnect from the server
     */
    void disconnect();

    /**
     * @brief Check if connected
     * @return true if connected
     */
    bool is_connected() const;

    /**
     * @brief Try to reconnect after disconnection
     * @return true if reconnection succeeded
     */
    bool try_reconnect();

    /**
     * @brief Wait for a frame-ready message from Unity
     * @param timeout Timeout duration
     * @return Frame message if received, nullopt on timeout or error
     */
    std::optional<TcpFrameReadyMessage> wait_for_frame(std::chrono::milliseconds timeout);

    /**
     * @brief Check if a frame-ready message is available (non-blocking)
     * @return Frame message if available, nullopt otherwise
     */
    std::optional<TcpFrameReadyMessage> try_get_frame();

    /**
     * @brief Send a velocity command to Unity
     * @param command The velocity command
     * @return true if sent successfully
     */
    bool send_velocity_command(const VelocityCommand& command);

    /**
     * @brief Acknowledge that a frame has been processed
     * @param frame_id The frame ID to acknowledge
     * @return true if sent successfully
     */
    bool send_frame_processed(uint64_t frame_id);

    /**
     * @brief Request a frame from Unity
     * @param with_depth Whether to include depth data
     * @return true if sent successfully
     */
    bool request_frame(bool with_depth);

    /**
     * @brief Send a ping and wait for pong
     * @param timeout_ms Timeout in milliseconds
     * @return true if pong received
     */
    bool ping(int timeout_ms = 100);

    /**
     * @brief Get the camera intrinsics received on connection
     * @return Camera intrinsics, or nullopt if not yet received
     */
    std::optional<TcpCameraIntrinsics> get_intrinsics() const;

    /**
     * @brief Check if intrinsics have been received
     * @return true if intrinsics are available
     */
    bool has_intrinsics() const;

    // Statistics
    uint64_t get_frames_received() const { return frames_received_; }
    uint64_t get_commands_sent() const { return commands_sent_; }
    uint64_t get_reconnect_count() const { return reconnect_count_; }

private:
    SimTcpClientConfig config_;
    int socket_fd_ = -1;
    std::atomic<bool> connected_{false};
    mutable std::mutex socket_mutex_;

    // Received data
    std::optional<TcpCameraIntrinsics> intrinsics_;
    uint64_t command_id_ = 0;

    // Statistics
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> commands_sent_{0};
    std::atomic<uint64_t> reconnect_count_{0};

    // Diagnostics
    std::shared_ptr<DiagnosticsModuleLogger> logger_;

    // Internal helpers
    bool read_exact(void* buffer, size_t size, int timeout_ms);
    bool write_exact(const void* buffer, size_t size);
    bool set_socket_timeout(int timeout_ms);
    bool handle_incoming_message();
    bool read_intrinsics_message();
    std::optional<TcpFrameReadyMessage> read_frame_ready_message();

    // Binary serialization helpers (little-endian)
    static void write_uint64(uint8_t* buffer, uint64_t value);
    static void write_double(uint8_t* buffer, double value);
    static uint64_t read_uint64(const uint8_t* buffer);
    static int32_t read_int32(const uint8_t* buffer);
    static double read_double(const uint8_t* buffer);
};

} // namespace auto_battlebot
