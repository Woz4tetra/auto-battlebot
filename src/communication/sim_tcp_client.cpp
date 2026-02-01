/**
 * @file sim_tcp_client.cpp
 * @brief TCP client implementation for Unity simulation communication
 */

#include "communication/sim_tcp_client.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cerrno>
#include <cstring>

namespace auto_battlebot
{

    SimTcpClient &SimTcpClient::instance()
    {
        static SimTcpClient instance;
        return instance;
    }

    SimTcpClient::SimTcpClient()
    {
        logger_ = DiagnosticsLogger::get_logger("sim_tcp_client");
        // Pre-allocate receive buffer for image data (8MB)
        recv_buffer_.resize(8 * 1024 * 1024);
    }

    SimTcpClient::~SimTcpClient()
    {
        disconnect();
    }

    void SimTcpClient::configure(const SimTcpClientConfig &config)
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (connected_)
        {
            logger_->warning("configure_while_connected",
                             "Configuration changed while connected - will apply on next connect");
        }

        config_ = config;
        configured_ = true;

        logger_->info("configured",
                      {{"host", config_.host},
                       {"port", config_.port},
                       {"buffer_size", config_.socket_buffer_size}});
    }

    void SimTcpClient::configure_optimizations()
    {
        if (socket_fd_ < 0)
            return;

        // Large socket buffers for image data (4MB default)
        int buf_size = config_.socket_buffer_size;
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0)
        {
            logger_->warning("so_rcvbuf_failed", {{"error", strerror(errno)}});
        }
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_SNDBUF, &buf_size, sizeof(buf_size)) < 0)
        {
            logger_->warning("so_sndbuf_failed", {{"error", strerror(errno)}});
        }

        // Disable Nagle's algorithm for low latency
        int flag = 1;
        if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)) < 0)
        {
            logger_->warning("tcp_nodelay_failed", {{"error", strerror(errno)}});
        }

        // Disable delayed ACKs (Linux-specific)
        if (setsockopt(socket_fd_, IPPROTO_TCP, TCP_QUICKACK, &flag, sizeof(flag)) < 0)
        {
            logger_->warning("tcp_quickack_failed", {{"error", strerror(errno)}});
        }

        logger_->info("optimizations_configured",
                      {{"buffer_size", buf_size}});
    }

    bool SimTcpClient::connect()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (connected_)
        {
            return true;
        }

        if (!configured_)
        {
            logger_->warning("connect_without_configure",
                             "Connecting with default configuration");
        }

        // Create socket
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0)
        {
            logger_->error("socket_create_failed", {{"error", strerror(errno)}});
            return false;
        }

        // Apply Linux socket optimizations
        configure_optimizations();

        // Set up address
        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(config_.port);

        if (inet_pton(AF_INET, config_.host.c_str(), &server_addr.sin_addr) <= 0)
        {
            logger_->error("invalid_address", {{"host", config_.host}});
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Set non-blocking for connect with timeout
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        // Attempt connection
        int result = ::connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr));

        if (result < 0 && errno != EINPROGRESS)
        {
            logger_->error("connect_failed", {{"error", strerror(errno)}, {"host", config_.host}, {"port", config_.port}});
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Wait for connection with timeout
        struct pollfd pfd{};
        pfd.fd = socket_fd_;
        pfd.events = POLLOUT;

        result = poll(&pfd, 1, config_.connect_timeout_ms);

        if (result <= 0)
        {
            logger_->error("connect_timeout", {{"timeout_ms", config_.connect_timeout_ms}});
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Check for connection errors
        int error = 0;
        socklen_t len = sizeof(error);
        if (getsockopt(socket_fd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0)
        {
            logger_->error("connect_error", {{"error", strerror(error)}});
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Set back to blocking mode
        fcntl(socket_fd_, F_SETFL, flags);

        connected_ = true;
        logger_->info("connected", {{"host", config_.host}, {"port", config_.port}});

        // Wait for intrinsics message from Unity
        if (!set_socket_timeout(config_.connect_timeout_ms))
        {
            logger_->warning("set_timeout_failed", "Failed to set socket timeout");
        }

        // Read the intrinsics message that Unity sends on connection
        uint8_t msg_type;
        if (read_exact(&msg_type, 1, config_.connect_timeout_ms))
        {
            if (static_cast<TcpMessageType>(msg_type) == TcpMessageType::CameraIntrinsics)
            {
                if (read_intrinsics_message())
                {
                    logger_->info("intrinsics_received",
                                  {{"width", intrinsics_->width},
                                   {"height", intrinsics_->height},
                                   {"fx", intrinsics_->fx},
                                   {"fy", intrinsics_->fy}});
                }
                else
                {
                    logger_->warning("intrinsics_read_failed", "Failed to read intrinsics message");
                }
            }
            else
            {
                logger_->warning("unexpected_first_message", {{"type", static_cast<int>(msg_type)}});
            }
        }
        else
        {
            logger_->warning("no_intrinsics_received", "No intrinsics message received from server");
        }

        return true;
    }

    void SimTcpClient::disconnect()
    {
        std::lock_guard<std::mutex> lock(socket_mutex_);

        if (socket_fd_ >= 0)
        {
            close(socket_fd_);
            socket_fd_ = -1;
        }

        connected_ = false;
        logger_->info("disconnected", "Disconnected from server");
    }

    bool SimTcpClient::is_connected() const
    {
        return connected_;
    }

    bool SimTcpClient::try_reconnect()
    {
        disconnect();

        if (config_.auto_reconnect)
        {
            reconnect_count_++;
            logger_->info("attempting_reconnect", {{"attempt", static_cast<int>(reconnect_count_.load())}});
            return connect();
        }

        return false;
    }

    std::optional<TcpFrameReadyMessage> SimTcpClient::wait_for_frame(std::chrono::milliseconds timeout)
    {
        if (!connected_)
        {
            return std::nullopt;
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);

        // Set timeout
        if (!set_socket_timeout(static_cast<int>(timeout.count())))
        {
            return std::nullopt;
        }

        // Read message type
        uint8_t msg_type;
        while (read_exact(&msg_type, 1, static_cast<int>(timeout.count())))
        {
            auto type = static_cast<TcpMessageType>(msg_type);

            switch (type)
            {
            case TcpMessageType::FrameReady:
            case TcpMessageType::FrameReadyNoDepth:
            {
                auto frame = read_frame_ready_message();
                if (frame)
                {
                    frames_received_++;
                    return frame;
                }
                return std::nullopt;
            }

            case TcpMessageType::FrameReadyWithData:
            case TcpMessageType::FrameReadyWithDataNoDepth:
            {
                // Fallback mode detected - read the message with data
                // and convert to basic frame message (caller should use wait_for_frame_with_data)
                fallback_mode_detected_ = true;
                auto frame_with_data = read_frame_ready_with_data_message();
                if (frame_with_data)
                {
                    frames_received_++;
                    // Return just the frame metadata
                    TcpFrameReadyMessage frame;
                    frame.frame_id = frame_with_data->frame_id;
                    frame.timestamp_ns = frame_with_data->timestamp_ns;
                    frame.pose = frame_with_data->pose;
                    return frame;
                }
                return std::nullopt;
            }

            case TcpMessageType::CameraIntrinsics:
                // Intrinsics can be re-sent, handle it
                read_intrinsics_message();
                break;

            case TcpMessageType::Ping:
            {
                // Respond with pong
                uint8_t pong = static_cast<uint8_t>(TcpMessageType::Pong);
                write_exact(&pong, 1);
                break;
            }

            default:
                logger_->warning("unexpected_message_type", {{"type", static_cast<int>(msg_type)}});
                return std::nullopt;
            }
        }

        return std::nullopt;
    }

    std::optional<TcpFrameReadyWithDataMessage> SimTcpClient::wait_for_frame_with_data(std::chrono::milliseconds timeout)
    {
        if (!connected_)
        {
            return std::nullopt;
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);

        // Set timeout
        if (!set_socket_timeout(static_cast<int>(timeout.count())))
        {
            std::cerr << "Failed to get frame. Failed to set socket timeout" << std::endl;
            return std::nullopt;
        }

        // Read message type
        uint8_t msg_type;
        std::cout << "Waiting for frame message..." << std::endl;
        while (read_exact(&msg_type, 1, static_cast<int>(timeout.count())))
        {
            auto type = static_cast<TcpMessageType>(msg_type);
            std::cout << "Received message type: 0x" << std::hex << static_cast<int>(msg_type) << std::dec << std::endl;

            switch (type)
            {
            case TcpMessageType::FrameReadyWithData:
            case TcpMessageType::FrameReadyWithDataNoDepth:
            {
                fallback_mode_detected_ = true;
                auto frame = read_frame_ready_with_data_message();
                if (frame)
                {
                    frames_received_++;
                    return frame;
                }
                return std::nullopt;
            }

            case TcpMessageType::FrameReady:
            case TcpMessageType::FrameReadyNoDepth:
            {
                // Non-fallback mode - convert to with-data format (no image data)
                auto basic_frame = read_frame_ready_message();
                if (basic_frame)
                {
                    frames_received_++;
                    TcpFrameReadyWithDataMessage frame;
                    frame.frame_id = basic_frame->frame_id;
                    frame.timestamp_ns = basic_frame->timestamp_ns;
                    frame.pose = basic_frame->pose;
                    frame.rgb_width = 0;
                    frame.rgb_height = 0;
                    frame.depth_width = 0;
                    frame.depth_height = 0;
                    frame.rgb_data_size = 0;
                    frame.depth_data_size = 0;
                    return frame;
                }
                return std::nullopt;
            }

            case TcpMessageType::CameraIntrinsics:
                read_intrinsics_message();
                break;

            case TcpMessageType::Ping:
            {
                uint8_t pong = static_cast<uint8_t>(TcpMessageType::Pong);
                write_exact(&pong, 1);
                break;
            }

            default:
                logger_->warning("unexpected_message_type", {{"type", static_cast<int>(msg_type)}});
                return std::nullopt;
            }
        }

        return std::nullopt;
    }

    std::optional<TcpFrameReadyMessage> SimTcpClient::try_get_frame()
    {
        if (!connected_)
        {
            return std::nullopt;
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);

        // Check if data is available without blocking
        struct pollfd pfd{};
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        int result = poll(&pfd, 1, 0);
        if (result <= 0)
        {
            return std::nullopt;
        }

        // Data available, try to read
        return wait_for_frame(std::chrono::milliseconds(10));
    }

    bool SimTcpClient::send_velocity_command(const VelocityCommand &command)
    {
        if (!connected_)
        {
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

        if (write_exact(buffer, sizeof(buffer)))
        {
            commands_sent_++;
            return true;
        }

        logger_->warning("send_command_failed", {{"error", strerror(errno)}});
        return false;
    }

    bool SimTcpClient::send_frame_processed(uint64_t frame_id)
    {
        if (!connected_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);

        uint8_t buffer[TcpMessageSize::FrameProcessed];

        buffer[0] = static_cast<uint8_t>(TcpMessageType::FrameProcessed);
        write_uint64(buffer + 1, frame_id);

        return write_exact(buffer, sizeof(buffer));
    }

    bool SimTcpClient::request_frame(bool with_depth)
    {
        if (!connected_)
        {
            std::cerr << "request_frame: not connected" << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);

        uint8_t buffer[TcpMessageSize::RequestFrame];
        buffer[0] = static_cast<uint8_t>(TcpMessageType::RequestFrame);
        buffer[1] = with_depth ? 1 : 0;

        bool result = write_exact(buffer, sizeof(buffer));
        std::cout << "request_frame sent: " << (result ? "success" : "failed") << ", with_depth=" << with_depth << std::endl;
        return result;
    }

    bool SimTcpClient::ping(int timeout_ms)
    {
        if (!connected_)
        {
            return false;
        }

        std::lock_guard<std::mutex> lock(socket_mutex_);

        // Send ping
        uint8_t ping_byte = static_cast<uint8_t>(TcpMessageType::Ping);
        if (!write_exact(&ping_byte, 1))
        {
            return false;
        }

        // Wait for pong
        set_socket_timeout(timeout_ms);

        uint8_t response;
        if (read_exact(&response, 1, timeout_ms))
        {
            return response == static_cast<uint8_t>(TcpMessageType::Pong);
        }

        return false;
    }

    std::optional<TcpCameraIntrinsics> SimTcpClient::get_intrinsics() const
    {
        return intrinsics_;
    }

    bool SimTcpClient::has_intrinsics() const
    {
        return intrinsics_.has_value();
    }

    // Private methods

    bool SimTcpClient::read_exact(void *buffer, size_t size, int timeout_ms)
    {
        uint8_t *ptr = static_cast<uint8_t *>(buffer);
        size_t remaining = size;
        size_t total_read = 0;

        // Use poll for timeout
        struct pollfd pfd{};
        pfd.fd = socket_fd_;
        pfd.events = POLLIN;

        while (remaining > 0)
        {
            int poll_result = poll(&pfd, 1, timeout_ms);

            if (poll_result < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                connected_ = false;
                std::cerr << "Read poll error: errno=" << errno << ", read " << total_read << "/" << size << " bytes" << std::endl;
                return false;
            }

            if (poll_result == 0)
            {
                // Timeout
                std::cerr << "Read poll timed out after reading " << total_read << "/" << size << " bytes" << std::endl;
                return false;
            }

            ssize_t bytes_read = recv(socket_fd_, ptr, remaining, 0);

            if (bytes_read < 0)
            {
                if (errno == EINTR || errno == EAGAIN)
                {
                    continue;
                }
                connected_ = false;
                std::cerr << "Read failed (" << errno << ") after reading " << total_read << "/" << size << " bytes" << std::endl;
                return false;
            }

            if (bytes_read == 0)
            {
                // Connection closed
                connected_ = false;
                std::cerr << "Read failed. Connection closed after reading " << total_read << "/" << size << " bytes" << std::endl;
                return false;
            }

            ptr += bytes_read;
            remaining -= bytes_read;
            total_read += bytes_read;
        }

        return true;
    }

    bool SimTcpClient::write_exact(const void *buffer, size_t size)
    {
        const uint8_t *ptr = static_cast<const uint8_t *>(buffer);
        size_t remaining = size;

        while (remaining > 0)
        {
            ssize_t bytes_written = send(socket_fd_, ptr, remaining, MSG_NOSIGNAL);

            if (bytes_written < 0)
            {
                if (errno == EINTR || errno == EAGAIN)
                {
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

    bool SimTcpClient::set_socket_timeout(int timeout_ms)
    {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;

        if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        {
            return false;
        }

        return true;
    }

    bool SimTcpClient::handle_incoming_message()
    {
        uint8_t msg_type;
        if (!read_exact(&msg_type, 1, config_.read_timeout_ms))
        {
            return false;
        }

        auto type = static_cast<TcpMessageType>(msg_type);

        switch (type)
        {
        case TcpMessageType::CameraIntrinsics:
            return read_intrinsics_message();

        case TcpMessageType::Ping:
        {
            uint8_t pong = static_cast<uint8_t>(TcpMessageType::Pong);
            return write_exact(&pong, 1);
        }

        default:
            logger_->warning("unhandled_message_type", {{"type", static_cast<int>(msg_type)}});
            return false;
        }
    }

    bool SimTcpClient::read_intrinsics_message()
    {
        uint8_t buffer[TcpCameraIntrinsics::payload_size];

        if (!read_exact(buffer, sizeof(buffer), config_.read_timeout_ms))
        {
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

        if (intrinsics.is_valid())
        {
            intrinsics_ = intrinsics;
            return true;
        }

        return false;
    }

    std::optional<TcpFrameReadyMessage> SimTcpClient::read_frame_ready_message()
    {
        uint8_t buffer[TcpFrameReadyMessage::payload_size];

        if (!read_exact(buffer, sizeof(buffer), config_.read_timeout_ms))
        {
            return std::nullopt;
        }

        TcpFrameReadyMessage frame;
        size_t offset = 0;

        frame.frame_id = read_uint64(buffer + offset);
        offset += 8;
        frame.timestamp_ns = read_uint64(buffer + offset);
        offset += 8;

        for (int i = 0; i < 16; i++)
        {
            frame.pose[i] = read_double(buffer + offset);
            offset += 8;
        }

        return frame;
    }

    std::optional<TcpFrameReadyWithDataMessage> SimTcpClient::read_frame_ready_with_data_message()
    {
        // Read header first
        uint8_t header_buffer[TcpFrameReadyWithDataMessage::header_payload_size];
        
        std::cout << "Reading frame header (" << sizeof(header_buffer) << " bytes)..." << std::endl;

        if (!read_exact(header_buffer, sizeof(header_buffer), config_.read_timeout_ms))
        {
            std::cerr << "Failed to read frame header" << std::endl;
            return std::nullopt;
        }
        
        std::cout << "Frame header read successfully" << std::endl;

        TcpFrameReadyWithDataMessage frame;
        size_t offset = 0;

        // Parse header
        frame.frame_id = read_uint64(header_buffer + offset);
        offset += 8;
        frame.timestamp_ns = read_uint64(header_buffer + offset);
        offset += 8;

        for (int i = 0; i < 16; i++)
        {
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
        
        std::cout << "Frame header parsed: frame_id=" << frame.frame_id 
                  << ", rgb=" << frame.rgb_width << "x" << frame.rgb_height
                  << ", depth=" << frame.depth_width << "x" << frame.depth_height
                  << ", rgb_size=" << frame.rgb_data_size 
                  << ", depth_size=" << frame.depth_data_size << std::endl;

        // Validate sizes
        if (frame.rgb_data_size < 0 || frame.depth_data_size < 0)
        {
            std::cerr << "Frame has invalid sizes. RGB size: " << frame.rgb_data_size << ", Depth size: " << frame.depth_data_size << std::endl;
            return std::nullopt;
        }

        // Read RGB data
        if (frame.rgb_data_size > 0)
        {
            std::cout << "Reading RGB data (" << frame.rgb_data_size << " bytes)..." << std::endl;
            frame.rgb_data.resize(frame.rgb_data_size);
            if (!read_exact(frame.rgb_data.data(), frame.rgb_data_size, config_.read_timeout_ms * 10))
            {
                std::cerr << "RGB read failed." << std::endl;
                return std::nullopt;
            }
            std::cout << "RGB data read successfully" << std::endl;
        }

        // Read depth data
        if (frame.depth_data_size > 0)
        {
            frame.depth_data.resize(frame.depth_data_size);
            if (!read_exact(frame.depth_data.data(), frame.depth_data_size, config_.read_timeout_ms * 10))
            {
                std::cerr << "Depth read failed." << std::endl;
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

    void SimTcpClient::write_uint64(uint8_t *buffer, uint64_t value)
    {
        buffer[0] = static_cast<uint8_t>(value & 0xFF);
        buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
        buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
        buffer[4] = static_cast<uint8_t>((value >> 32) & 0xFF);
        buffer[5] = static_cast<uint8_t>((value >> 40) & 0xFF);
        buffer[6] = static_cast<uint8_t>((value >> 48) & 0xFF);
        buffer[7] = static_cast<uint8_t>((value >> 56) & 0xFF);
    }

    void SimTcpClient::write_double(uint8_t *buffer, double value)
    {
        uint64_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        write_uint64(buffer, bits);
    }

    uint64_t SimTcpClient::read_uint64(const uint8_t *buffer)
    {
        return static_cast<uint64_t>(buffer[0]) |
               (static_cast<uint64_t>(buffer[1]) << 8) |
               (static_cast<uint64_t>(buffer[2]) << 16) |
               (static_cast<uint64_t>(buffer[3]) << 24) |
               (static_cast<uint64_t>(buffer[4]) << 32) |
               (static_cast<uint64_t>(buffer[5]) << 40) |
               (static_cast<uint64_t>(buffer[6]) << 48) |
               (static_cast<uint64_t>(buffer[7]) << 56);
    }

    int32_t SimTcpClient::read_int32(const uint8_t *buffer)
    {
        return static_cast<int32_t>(buffer[0]) |
               (static_cast<int32_t>(buffer[1]) << 8) |
               (static_cast<int32_t>(buffer[2]) << 16) |
               (static_cast<int32_t>(buffer[3]) << 24);
    }

    double SimTcpClient::read_double(const uint8_t *buffer)
    {
        uint64_t bits = read_uint64(buffer);
        double value;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

} // namespace auto_battlebot
