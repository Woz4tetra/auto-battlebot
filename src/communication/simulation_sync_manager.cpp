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
        std::cout << "[SimulationSyncManager] Already initialized" << std::endl;
        return sync_socket_ && sync_socket_->is_connected();
    }

    std::cout << "[SimulationSyncManager] Initializing sync socket..." << std::endl;
    
    sync_socket_ = std::make_unique<SyncSocket>(config);
    
    if (!sync_socket_->connect())
    {
        std::cerr << "[SimulationSyncManager] Failed to connect to Unity sync socket" << std::endl;
        sync_socket_.reset();
        return false;
    }

    initialized_ = true;
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

bool SimulationSyncManager::is_connected() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_ && sync_socket_ && sync_socket_->is_connected();
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
