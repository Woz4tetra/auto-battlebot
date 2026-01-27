#include "communication/simulation_sync_manager.hpp"
#include <iostream>

namespace auto_battlebot
{

SimulationSyncManager& SimulationSyncManager::instance()
{
    static SimulationSyncManager instance;
    return instance;
}

bool SimulationSyncManager::initialize(const SyncSocketConfiguration& config)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_)
    {
        // Reuse existing socket and attempt reconnect if needed (supports Unity restarts)
        if (sync_socket_)
        {
            sync_socket_->connect();
            return sync_socket_->is_connected();
        }
        return false;
    }

    std::cout << "[SimulationSyncManager] Initializing sync socket..." << std::endl;
    
    sync_socket_ = std::make_unique<SyncSocket>(config);

    initialized_ = true;

    // Best-effort initial connect (Unity might not be up yet). We'll auto-reconnect later.
    if (!sync_socket_->connect())
    {
        std::cerr << "[SimulationSyncManager] Failed to connect to Unity sync socket (will retry)" << std::endl;
        return false;
    }

    std::cout << "[SimulationSyncManager] Connected to Unity" << std::endl;
    return true;
}

void SimulationSyncManager::shutdown()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (sync_socket_)
    {
        sync_socket_->disconnect();
        sync_socket_.reset();
    }
    initialized_ = false;
    std::cout << "[SimulationSyncManager] Shutdown complete" << std::endl;
}

bool SimulationSyncManager::is_connected()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || !sync_socket_)
    {
        return false;
    }

    // Auto-reconnect: SyncSocket methods also attempt reconnect, but callers often gate on is_connected().
    // Calling connect() here allows recovery after Unity restarts.
    if (!sync_socket_->is_connected())
    {
        sync_socket_->connect();
    }

    return sync_socket_->is_connected();
}

bool SimulationSyncManager::request_frame(bool with_depth)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!sync_socket_)
    {
        return false;
    }
    
    return sync_socket_->request_frame(with_depth);
}

bool SimulationSyncManager::wait_for_frame(int timeout_ms, bool* has_depth)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!sync_socket_)
    {
        return false;
    }
    
    return sync_socket_->wait_for_frame(timeout_ms, has_depth);
}

bool SimulationSyncManager::signal_command_ready()
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!sync_socket_)
    {
        return false;
    }
    
    return sync_socket_->signal_command_ready();
}

SyncSocket* SimulationSyncManager::get_socket()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return sync_socket_.get();
}

SyncSocket::Stats SimulationSyncManager::get_stats() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!sync_socket_)
    {
        return SyncSocket::Stats{};
    }
    
    return sync_socket_->get_stats();
}

} // namespace auto_battlebot
