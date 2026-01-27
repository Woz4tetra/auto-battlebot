#pragma once

#include "communication/sync_socket.hpp"
#include <memory>
#include <mutex>

namespace auto_battlebot
{

/// Singleton manager for simulation synchronization.
/// Shared between SimRgbdCamera (frame requests) and SimTransmitter (command signals).
/// 
/// Usage pattern:
///   1. SimRgbdCamera::get() calls request_frame(with_depth) then wait_for_frame()
///   2. After processing, SimTransmitter::send() calls signal_command_ready()
///   3. Unity receives command-ready, processes, then sends next frame-ready
class SimulationSyncManager
{
public:
    /// Get the singleton instance
    static SimulationSyncManager& instance();

    /// Initialize with configuration (call once at startup)
    /// @return True if connected successfully
    bool initialize(const SyncSocketConfiguration& config = SyncSocketConfiguration{});

    /// Shutdown and cleanup
    void shutdown();

    /// Check if initialized and connected (auto-reconnects if needed)
    bool is_connected();

    /// Request a frame from Unity
    /// @param with_depth Whether to include depth data
    /// @return True if request was sent
    bool request_frame(bool with_depth);

    /// Wait for frame ready signal
    /// @param timeout_ms Timeout (-1 for default)
    /// @param has_depth Output: whether frame includes depth
    /// @return True if frame is ready
    bool wait_for_frame(int timeout_ms = -1, bool* has_depth = nullptr);

    /// Signal that command is ready (called by SimTransmitter)
    /// @return True if signal was sent
    bool signal_command_ready();

    /// Get the underlying sync socket (for advanced usage)
    SyncSocket* get_socket();

    /// Get statistics
    SyncSocket::Stats get_stats() const;

private:
    SimulationSyncManager() = default;
    ~SimulationSyncManager() = default;

    // Non-copyable
    SimulationSyncManager(const SimulationSyncManager&) = delete;
    SimulationSyncManager& operator=(const SimulationSyncManager&) = delete;

    std::unique_ptr<SyncSocket> sync_socket_;
    mutable std::mutex mutex_;
    bool initialized_ = false;
};

} // namespace auto_battlebot
