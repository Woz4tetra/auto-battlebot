#include "communication/sync_socket.hpp"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

namespace auto_battlebot
{

SyncSocket::SyncSocket(const SyncSocketConfiguration& config)
    : config_(config)
{
}

SyncSocket::~SyncSocket()
{
    disconnect();
}

bool SyncSocket::connect()
{
    if (connected_)
    {
        return true;
    }

    bool success = config_.use_tcp ? connect_tcp() : connect_unix();
    
    if (success)
    {
        connected_ = true;
        std::cout << "[SyncSocket] Connected to Unity sync socket" << std::endl;
    }
    
    return success;
}

bool SyncSocket::connect_unix()
{
    socket_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd_ < 0)
    {
        std::cerr << "[SyncSocket] Failed to create Unix socket: " << strerror(errno) << std::endl;
        return false;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, config_.socket_path.c_str(), sizeof(addr.sun_path) - 1);

    if (::connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "[SyncSocket] Failed to connect to " << config_.socket_path 
                  << ": " << strerror(errno) << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set socket timeout for reads
    struct timeval tv;
    tv.tv_sec = config_.timeout_ms / 1000;
    tv.tv_usec = (config_.timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
}

bool SyncSocket::connect_tcp()
{
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0)
    {
        std::cerr << "[SyncSocket] Failed to create TCP socket: " << strerror(errno) << std::endl;
        return false;
    }

    // Disable Nagle's algorithm for low latency
    int flag = 1;
    setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(config_.tcp_port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (::connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "[SyncSocket] Failed to connect to localhost:" << config_.tcp_port 
                  << ": " << strerror(errno) << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set socket timeout for reads
    struct timeval tv;
    tv.tv_sec = config_.timeout_ms / 1000;
    tv.tv_usec = (config_.timeout_ms % 1000) * 1000;
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    return true;
}

void SyncSocket::disconnect()
{
    if (socket_fd_ >= 0)
    {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    connected_ = false;
}

bool SyncSocket::is_connected() const
{
    return connected_ && socket_fd_ >= 0;
}

bool SyncSocket::request_frame(bool with_depth)
{
    if (!is_connected())
    {
        return false;
    }

    last_request_time_ = std::chrono::steady_clock::now();
    
    uint8_t signal = with_depth ? SyncSignal::REQUEST_FRAME_WITH_DEPTH : SyncSignal::REQUEST_FRAME;
    return send_signal(signal);
}

bool SyncSocket::wait_for_frame(int timeout_ms, bool* has_depth)
{
    if (!is_connected())
    {
        return false;
    }

    int actual_timeout = (timeout_ms < 0) ? config_.timeout_ms : timeout_ms;
    
    int signal = wait_for_any_signal(actual_timeout);
    
    if (signal < 0)
    {
        stats_.timeouts++;
        return false;
    }

    if (signal == SyncSignal::FRAME_READY)
    {
        if (has_depth) *has_depth = true;
        stats_.frames_received++;
        
        // Update latency stats
        auto now = std::chrono::steady_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_request_time_).count();
        update_latency_stats(latency);
        
        if (frame_ready_callback_)
        {
            frame_ready_callback_(true);
        }
        return true;
    }
    else if (signal == SyncSignal::FRAME_NO_DEPTH_READY)
    {
        if (has_depth) *has_depth = false;
        stats_.frames_received++;
        
        auto now = std::chrono::steady_clock::now();
        auto latency = std::chrono::duration_cast<std::chrono::microseconds>(
            now - last_request_time_).count();
        update_latency_stats(latency);
        
        if (frame_ready_callback_)
        {
            frame_ready_callback_(false);
        }
        return true;
    }
    else if (signal == SyncSignal::PING)
    {
        // Respond to ping and continue waiting
        send_signal(SyncSignal::PONG);
        return wait_for_frame(timeout_ms, has_depth);
    }

    std::cerr << "[SyncSocket] Unexpected signal: 0x" << std::hex << signal << std::dec << std::endl;
    return false;
}

bool SyncSocket::signal_command_ready()
{
    if (!is_connected())
    {
        return false;
    }

    bool success = send_signal(SyncSignal::COMMAND_READY);
    if (success)
    {
        stats_.commands_sent++;
    }
    return success;
}

int64_t SyncSocket::ping(int timeout_ms)
{
    if (!is_connected())
    {
        return -1;
    }

    auto start = std::chrono::steady_clock::now();
    
    if (!send_signal(SyncSignal::PING))
    {
        return -1;
    }

    if (!wait_for_signal(SyncSignal::PONG, timeout_ms))
    {
        return -1;
    }

    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
}

SyncSocket::Stats SyncSocket::get_stats() const
{
    Stats s = stats_;
    if (latency_count_ > 0)
    {
        s.avg_latency_us = static_cast<double>(total_latency_us_) / latency_count_;
    }
    return s;
}

void SyncSocket::reset_stats()
{
    stats_ = Stats{};
    total_latency_us_ = 0;
    latency_count_ = 0;
}

void SyncSocket::set_frame_ready_callback(FrameReadyCallback callback)
{
    frame_ready_callback_ = std::move(callback);
}

bool SyncSocket::send_signal(uint8_t signal)
{
    if (socket_fd_ < 0)
    {
        return false;
    }

    ssize_t sent = write(socket_fd_, &signal, 1);
    if (sent != 1)
    {
        std::cerr << "[SyncSocket] Failed to send signal: " << strerror(errno) << std::endl;
        disconnect();
        return false;
    }
    return true;
}

bool SyncSocket::wait_for_signal(uint8_t expected_signal, int timeout_ms)
{
    int signal = wait_for_any_signal(timeout_ms);
    return signal == expected_signal;
}

int SyncSocket::wait_for_any_signal(int timeout_ms)
{
    if (socket_fd_ < 0)
    {
        return -1;
    }

    // Use poll for timeout
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0)
    {
        if (ret == 0)
        {
            // Timeout
            return -1;
        }
        std::cerr << "[SyncSocket] Poll error: " << strerror(errno) << std::endl;
        disconnect();
        return -2;
    }

    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
    {
        std::cerr << "[SyncSocket] Socket error during poll" << std::endl;
        disconnect();
        return -2;
    }

    uint8_t signal;
    ssize_t received = read(socket_fd_, &signal, 1);
    if (received != 1)
    {
        if (received == 0)
        {
            std::cerr << "[SyncSocket] Connection closed by Unity" << std::endl;
        }
        else
        {
            std::cerr << "[SyncSocket] Read error: " << strerror(errno) << std::endl;
        }
        disconnect();
        return -2;
    }

    return signal;
}

void SyncSocket::update_latency_stats(int64_t latency_us)
{
    total_latency_us_ += latency_us;
    latency_count_++;
    if (latency_us > stats_.max_latency_us)
    {
        stats_.max_latency_us = latency_us;
    }
}

} // namespace auto_battlebot
