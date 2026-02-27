/**
 * @file sim_tcp_client.cpp
 * @brief TCP client implementation for Unity simulation communication
 */

#include "communication/sim_tcp_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace auto_battlebot {

SimTcpClient &SimTcpClient::instance() {
    static SimTcpClient instance;
    return instance;
}

SimTcpClient::SimTcpClient() {
    logger_ = DiagnosticsLogger::get_logger("sim_tcp_client");
    // Pre-allocate receive buffer for image data (8MB)
    recv_buffer_.resize(8 * 1024 * 1024);
}

SimTcpClient::~SimTcpClient() { disconnect(); }

void SimTcpClient::configure(const SimTcpClientConfig &config) {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (connected_) {
        std::cerr << "Configuration changed while connected - will apply on next connect"
                  << std::endl;
    }

    config_ = config;
    configured_ = true;

    std::cout << "Socket configured. Host: " << config_.host << ". Port: " << config_.port
              << std::endl;
}

void SimTcpClient::configure_optimizations() {
    if (socket_fd_ < 0) return;

    // Large socket buffers for image data (4MB default)
    int buf_size = config_.socket_buffer_size;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0) {
        std::cerr << "Failed to configure socket: " << strerror(errno) << " (" << errno << ")"
                  << std::endl;
    }
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size)) < 0) {
        std::cerr << "Failed to configure socket: " << strerror(errno) << " (" << errno << ")"
                  << std::endl;
    }

    // Disable Nagle's algorithm for low latency
    int flag = 1;
    if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0) {
        std::cerr << "Failed to configure socket: " << strerror(errno) << " (" << errno << ")"
                  << std::endl;
    }

    // Disable delayed ACKs (Linux-specific)
    if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(flag)) < 0) {
        std::cerr << "Failed to configure socket: " << strerror(errno) << " (" << errno << ")"
                  << std::endl;
    }

    std::cout << "Socket optimizations configured" << std::endl;
}

bool SimTcpClient::connect() {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (connected_) {
        return true;
    }

    if (!configured_) {
        std::cout << "Connecting with default configuration" << std::endl;
    }

    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket: " << strerror(errno) << " (" << errno << ")"
                  << std::endl;
        return false;
    }

    // Apply Linux socket optimizations
    configure_optimizations();

    // Set up address
    struct sockaddr_in server_addr {};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(config_.port);

    if (inet_pton(AF_INET, config_.host.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << config_.host << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set non-blocking for connect with timeout
    int flags = fcntl(socket_fd_, F_GETFL, 0);
    fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

    // Attempt connection
    int result = ::connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));

    if (result < 0 && errno != EINPROGRESS) {
        std::cerr << "Failed to connect. Error: " << strerror(errno) << " (" << errno << ")"
                  << ". Host: " << config_.host << ". Port: " << config_.port << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Wait for connection with timeout
    struct pollfd pfd {};
    pfd.fd = socket_fd_;
    pfd.events = POLLOUT;

    result = poll(&pfd, 1, config_.connect_timeout_ms);

    if (result <= 0) {
        std::cerr << "Connection timeout: " << config_.connect_timeout_ms << " (ms)" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Check for connection errors
    int error = 0;
    socklen_t len = sizeof(error);
    if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
        std::cerr << "Connection error: " << strerror(errno) << " (" << errno << ")" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    // Set back to blocking mode
    fcntl(socket_fd_, F_SETFL, flags);

    connected_ = true;
    logger_->info("connected", {{"host", config_.host}, {"port", config_.port}});

    // Wait for intrinsics message from Unity
    if (!set_socket_timeout(config_.connect_timeout_ms)) {
        std::cerr << "Failed to set socket timeout" << std::endl;
    }

    // Read the intrinsics message that Unity sends on connection
    uint8_t msg_type;
    if (read_exact(&msg_type, 1, config_.connect_timeout_ms)) {
        if (static_cast<TcpMessageType>(msg_type) == TcpMessageType::CameraIntrinsics) {
            if (!read_intrinsics_message()) {
                std::cerr << "Failed to read intrinsics message" << std::endl;
            }
        } else {
            std::cerr << "Unexpected first message: " << static_cast<int>(msg_type) << std::endl;
        }
    } else {
        std::cerr << "No intrinsics message received from server" << std::endl;
    }

    std::cout << "Socket connection complete" << std::endl;

    return true;
}

void SimTcpClient::disconnect() {
    if (!is_connected()) {
        return;
    }
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }

    connected_ = false;
    std::cout << "Disconnected from server" << std::endl;
}

bool SimTcpClient::is_connected() const { return connected_; }

bool SimTcpClient::wait_for_connection() {
    disconnect();
    while (!is_connected()) {
        if (!miniros::ok()) {
            miniros::shutdown();
            return false;
        }
        if (!connect()) {
            std::cerr << "Failed to initialize client." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
    }
    return true;
    std::cout << "Done waiting for connection" << std::endl;
}

bool SimTcpClient::try_reconnect() {
    disconnect();

    if (config_.auto_reconnect) {
        reconnect_count_++;
        std::cout << "attempting_reconnect: " << static_cast<int>(reconnect_count_.load())
                  << std::endl;
        return connect();
    }

    return false;
}

std::optional<TcpFrameReadyWithDataMessage> SimTcpClient::wait_for_frame_with_data(
    std::chrono::milliseconds timeout) {
    if (!connected_) {
        return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);

    // Set timeout
    if (!set_socket_timeout(static_cast<int>(timeout.count()))) {
        return std::nullopt;
    }

    // Read message type
    uint8_t msg_type;
    while (read_exact(&msg_type, 1, static_cast<int>(timeout.count()))) {
        auto type = static_cast<TcpMessageType>(msg_type);

        switch (type) {
            case TcpMessageType::FrameReadyWithData:
            case TcpMessageType::FrameReadyWithDataNoDepth: {
                fallback_mode_detected_ = true;
                auto frame = read_frame_ready_with_data_message();
                if (frame) {
                    frames_received_++;
                    return frame;
                }
                return std::nullopt;
            }

            case TcpMessageType::CameraIntrinsics:
                read_intrinsics_message();
                break;

            default:
                std::cerr << "Unexpected message type: " << static_cast<int>(msg_type) << std::endl;
                return std::nullopt;
        }
    }

    return std::nullopt;
}

bool SimTcpClient::send_velocity_command(const VelocityCommand &command) {
    if (!connected_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);

    uint8_t buffer[TcpMessageSize::VelocityCommand];
    size_t offset = 0;

    // Message type
    buffer[offset++] = static_cast<uint8_t>(TcpMessageType::VelocityCommand);

    // Command ID
    write_uint64(buffer + offset, ++command_id_);
    offset += 8;

    // Velocity values
    write_double(buffer + offset, command.linear_x);
    offset += 8;
    write_double(buffer + offset, command.linear_y);
    offset += 8;
    write_double(buffer + offset, command.angular_z);

    if (write_exact(buffer, sizeof(buffer))) {
        commands_sent_++;
        return true;
    }

    std::cerr << "Send command failed: " << strerror(errno) << " (" << errno << ")" << std::endl;
    return false;
}

bool SimTcpClient::send_frame_processed(uint64_t frame_id) {
    if (!connected_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);

    uint8_t buffer[TcpMessageSize::FrameProcessed];

    buffer[0] = static_cast<uint8_t>(TcpMessageType::FrameProcessed);
    write_uint64(buffer + 1, frame_id);

    return write_exact(buffer, sizeof(buffer));
}

bool SimTcpClient::request_frame(bool with_depth) {
    if (!connected_) {
        return false;
    }

    std::lock_guard<std::mutex> lock(socket_mutex_);

    uint8_t buffer[TcpMessageSize::RequestFrame];
    buffer[0] = static_cast<uint8_t>(TcpMessageType::RequestFrame);
    buffer[1] = with_depth ? 1 : 0;

    return write_exact(buffer, sizeof(buffer));
}

std::optional<TcpCameraIntrinsics> SimTcpClient::get_intrinsics() const { return intrinsics_; }

bool SimTcpClient::has_intrinsics() const { return intrinsics_.has_value(); }

// Private methods

bool SimTcpClient::read_exact(void *buffer, size_t size, int timeout_ms) {
    uint8_t *ptr = static_cast<uint8_t *>(buffer);
    size_t remaining = size;

    // Use poll for timeout
    struct pollfd pfd {};
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;

    while (remaining > 0) {
        int poll_result = poll(&pfd, 1, timeout_ms);

        if (poll_result < 0) {
            if (errno == EINTR) {
                continue;
            }
            connected_ = false;
            return false;
        }

        if (poll_result == 0) {
            // Timeout
            return false;
        }

        ssize_t bytes_read = recv(socket_fd_, ptr, remaining, 0);

        if (bytes_read < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            connected_ = false;
            return false;
        }

        if (bytes_read == 0) {
            // Connection closed
            connected_ = false;
            return false;
        }

        ptr += bytes_read;
        remaining -= bytes_read;
    }

    return true;
}

bool SimTcpClient::write_exact(const void *buffer, size_t size) {
    const uint8_t *ptr = static_cast<const uint8_t *>(buffer);
    size_t remaining = size;

    while (remaining > 0) {
        ssize_t bytes_written = send(socket_fd_, ptr, remaining, MSG_NOSIGNAL);

        if (bytes_written < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            connected_ = false;
            return false;
        }

        ptr += bytes_written;
        remaining -= bytes_written;
    }

    return true;
}

bool SimTcpClient::set_socket_timeout(int timeout_ms) {
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        return false;
    }

    return true;
}

bool SimTcpClient::handle_incoming_message() {
    uint8_t msg_type;
    if (!read_exact(&msg_type, 1, config_.read_timeout_ms)) {
        return false;
    }

    auto type = static_cast<TcpMessageType>(msg_type);

    switch (type) {
        case TcpMessageType::CameraIntrinsics:
            return read_intrinsics_message();

        default:
            std::cerr << "Unexpected message type: " << static_cast<int>(msg_type) << std::endl;
            return false;
    }
}

bool SimTcpClient::read_intrinsics_message() {
    uint8_t buffer[TcpCameraIntrinsics::payload_size];

    if (!read_exact(buffer, sizeof(buffer), config_.read_timeout_ms)) {
        return false;
    }

    TcpCameraIntrinsics intrinsics;
    size_t offset = 0;

    intrinsics.width = read_int32(buffer + offset);
    offset += 4;
    intrinsics.height = read_int32(buffer + offset);
    offset += 4;
    intrinsics.fx = read_double(buffer + offset);
    offset += 8;
    intrinsics.fy = read_double(buffer + offset);
    offset += 8;
    intrinsics.cx = read_double(buffer + offset);
    offset += 8;
    intrinsics.cy = read_double(buffer + offset);
    offset += 8;
    intrinsics.k1 = read_double(buffer + offset);
    offset += 8;
    intrinsics.k2 = read_double(buffer + offset);
    offset += 8;
    intrinsics.p1 = read_double(buffer + offset);
    offset += 8;
    intrinsics.p2 = read_double(buffer + offset);
    offset += 8;
    intrinsics.k3 = read_double(buffer + offset);

    if (intrinsics.is_valid()) {
        intrinsics_ = intrinsics;
        return true;
    }

    return false;
}

std::optional<TcpFrameReadyMessage> SimTcpClient::read_frame_ready_message() {
    uint8_t buffer[TcpFrameReadyMessage::payload_size];

    if (!read_exact(buffer, sizeof(buffer), config_.read_timeout_ms)) {
        return std::nullopt;
    }

    TcpFrameReadyMessage frame;
    size_t offset = 0;

    frame.frame_id = read_uint64(buffer + offset);
    offset += 8;
    frame.timestamp_ns = read_uint64(buffer + offset);
    offset += 8;

    for (int i = 0; i < 16; i++) {
        frame.pose[i] = read_double(buffer + offset);
        offset += 8;
    }

    return frame;
}

std::optional<TcpFrameReadyWithDataMessage> SimTcpClient::read_frame_ready_with_data_message() {
    // Read header first
    uint8_t header_buffer[TcpFrameReadyWithDataMessage::header_payload_size];

    if (!read_exact(header_buffer, sizeof(header_buffer), config_.read_timeout_ms)) {
        std::cerr << "Failed to read frame header" << std::endl;
        return std::nullopt;
    }

    TcpFrameReadyWithDataMessage frame;
    size_t offset = 0;

    // Parse header
    frame.frame_id = read_uint64(header_buffer + offset);
    offset += 8;
    frame.timestamp_ns = read_uint64(header_buffer + offset);
    offset += 8;

    for (int i = 0; i < 16; i++) {
        frame.pose[i] = read_double(header_buffer + offset);
        offset += 8;
    }

    frame.rgb_width = read_int32(header_buffer + offset);
    offset += 4;
    frame.rgb_height = read_int32(header_buffer + offset);
    offset += 4;
    frame.depth_width = read_int32(header_buffer + offset);
    offset += 4;
    frame.depth_height = read_int32(header_buffer + offset);
    offset += 4;
    frame.rgb_data_size = read_int32(header_buffer + offset);
    offset += 4;
    frame.depth_data_size = read_int32(header_buffer + offset);

    // Validate sizes
    if (frame.rgb_data_size < 0 || frame.depth_data_size < 0) {
        std::cerr << "Frame data has invalid sizes. RGB size: " << frame.rgb_data_size
                  << ". Depth size: " << frame.depth_data_size << std::endl;
        return std::nullopt;
    }

    // Read RGB data
    if (frame.rgb_data_size > 0) {
        frame.rgb_data.resize(frame.rgb_data_size);
        if (!read_exact(frame.rgb_data.data(), frame.rgb_data_size, config_.read_timeout_ms * 10)) {
            std::cerr << "Failed to read RGB size. RGB size: " << frame.rgb_data_size << std::endl;
            return std::nullopt;
        }
    }

    // Read depth data
    if (frame.depth_data_size > 0) {
        frame.depth_data.resize(frame.depth_data_size);
        if (!read_exact(frame.depth_data.data(), frame.depth_data_size,
                        config_.read_timeout_ms * 10)) {
            std::cerr << "Failed to read RGB size. Depth size: " << frame.depth_data_size
                      << std::endl;
            return std::nullopt;
        }
    }

    logger_->debug("frame_with_data_received",
                   {{"frame_id", static_cast<int>(frame.frame_id)},
                    {"rgb_size", static_cast<int>(frame.rgb_data_size)},
                    {"depth_size", static_cast<int>(frame.depth_data_size)}});

    return frame;
}

// Binary serialization helpers (little-endian)

void SimTcpClient::write_uint64(uint8_t *buffer, uint64_t value) {
    buffer[0] = static_cast<uint8_t>(value & 0xFF);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    buffer[4] = static_cast<uint8_t>((value >> 32) & 0xFF);
    buffer[5] = static_cast<uint8_t>((value >> 40) & 0xFF);
    buffer[6] = static_cast<uint8_t>((value >> 48) & 0xFF);
    buffer[7] = static_cast<uint8_t>((value >> 56) & 0xFF);
}

void SimTcpClient::write_double(uint8_t *buffer, double value) {
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    write_uint64(buffer, bits);
}

uint64_t SimTcpClient::read_uint64(const uint8_t *buffer) {
    return static_cast<uint64_t>(buffer[0]) | (static_cast<uint64_t>(buffer[1]) << 8) |
           (static_cast<uint64_t>(buffer[2]) << 16) | (static_cast<uint64_t>(buffer[3]) << 24) |
           (static_cast<uint64_t>(buffer[4]) << 32) | (static_cast<uint64_t>(buffer[5]) << 40) |
           (static_cast<uint64_t>(buffer[6]) << 48) | (static_cast<uint64_t>(buffer[7]) << 56);
}

int32_t SimTcpClient::read_int32(const uint8_t *buffer) {
    return static_cast<int32_t>(buffer[0]) | (static_cast<int32_t>(buffer[1]) << 8) |
           (static_cast<int32_t>(buffer[2]) << 16) | (static_cast<int32_t>(buffer[3]) << 24);
}

double SimTcpClient::read_double(const uint8_t *buffer) {
    uint64_t bits = read_uint64(buffer);
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

}  // namespace auto_battlebot
