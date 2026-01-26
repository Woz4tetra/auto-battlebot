#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <chrono>

namespace auto_battlebot
{

/// Signal bytes for sync protocol
namespace SyncSignal
{
    constexpr uint8_t FRAME_READY = 0x01;           // Unity → C++: Frame ready
    constexpr uint8_t COMMAND_READY = 0x02;         // C++ → Unity: Command ready
    constexpr uint8_t PING = 0x03;                  // Heartbeat
    constexpr uint8_t PONG = 0x04;                  // Heartbeat response
    constexpr uint8_t REQUEST_FRAME = 0x05;         // C++ → Unity: Request frame (no depth)
    constexpr uint8_t REQUEST_FRAME_WITH_DEPTH = 0x06;  // C++ → Unity: Request frame with depth
    constexpr uint8_t FRAME_NO_DEPTH_READY = 0x07;  // Unity → C++: Frame without depth ready
} // namespace SyncSignal

/// Configuration for SyncSocket
struct SyncSocketConfiguration
{
    std::string socket_path = "/tmp/auto_battlebot_sync.sock";
    int tcp_port = 47821;  // Fallback for Windows
    int timeout_ms = 1000;
    int reconnect_interval_ms = 500;
    bool use_tcp = false;  // Use TCP instead of Unix domain socket
};

/// Synchronization socket for frame timing coordination with Unity.
/// C++ acts as client, Unity acts as server.
class SyncSocket
{
public:
    using FrameReadyCallback = std::function<void(bool has_depth)>;

    explicit SyncSocket(const SyncSocketConfiguration& config = SyncSocketConfiguration{});
    ~SyncSocket();

    // Non-copyable
    SyncSocket(const SyncSocket&) = delete;
    SyncSocket& operator=(const SyncSocket&) = delete;

    /// Connect to Unity's sync socket server
    bool connect();

    /// Disconnect from server
    void disconnect();

    /// Check if connected
    bool is_connected() const;

    /// Request a frame from Unity (with or without depth)
    /// @param with_depth If true, Unity will capture and include depth data
    /// @return True if request was sent successfully
    bool request_frame(bool with_depth);

    /// Wait for frame ready signal from Unity
    /// @param timeout_ms Timeout in milliseconds (-1 for default)
    /// @param has_depth Output: whether the frame includes depth
    /// @return True if frame is ready
    bool wait_for_frame(int timeout_ms = -1, bool* has_depth = nullptr);

    /// Signal that command has been written to shared memory
    /// @return True if signal was sent successfully
    bool signal_command_ready();

    /// Send ping and wait for pong (connection test)
    /// @param timeout_ms Timeout in milliseconds
    /// @return Round-trip time in microseconds, or -1 on failure
    int64_t ping(int timeout_ms = 100);

    /// Get connection statistics
    struct Stats
    {
        uint64_t frames_received = 0;
        uint64_t commands_sent = 0;
        uint64_t timeouts = 0;
        uint64_t reconnects = 0;
        double avg_latency_us = 0;
        int64_t max_latency_us = 0;
    };
    Stats get_stats() const;

    /// Reset statistics
    void reset_stats();

    /// Set callback for when frame ready signal is received (async mode)
    void set_frame_ready_callback(FrameReadyCallback callback);

private:
    bool connect_unix();
    bool connect_tcp();
    bool send_signal(uint8_t signal);
    bool wait_for_signal(uint8_t expected_signal, int timeout_ms);
    int wait_for_any_signal(int timeout_ms);
    void update_latency_stats(int64_t latency_us);

    SyncSocketConfiguration config_;
    int socket_fd_ = -1;
    std::atomic<bool> connected_{false};

    // Statistics
    mutable Stats stats_;
    std::chrono::steady_clock::time_point last_request_time_;
    int64_t total_latency_us_ = 0;
    uint64_t latency_count_ = 0;

    // Callback
    FrameReadyCallback frame_ready_callback_;
};

} // namespace auto_battlebot
