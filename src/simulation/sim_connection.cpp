#include "simulation/sim_connection.hpp"

#include <arpa/inet.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <thread>

namespace auto_battlebot {

// Wire format — keep in sync with simulation/sim_server.py
#pragma pack(push, 1)
struct SimRequest {
    double linear_x;
    double linear_y;
    double angular_z;
};

struct SimResponseHeader {
    uint32_t width;
    uint32_t height;
    double tf_matrix[16];
    double fx, fy, cx, cy;
};
#pragma pack(pop)

std::shared_ptr<SimConnection> SimConnection::instance_;

void SimConnection::configure(const std::string &host, int port) {
    instance_ = std::shared_ptr<SimConnection>(new SimConnection(host, port));
}

std::shared_ptr<SimConnection> SimConnection::instance() {
    if (!instance_) {
        instance_ = std::shared_ptr<SimConnection>(new SimConnection("127.0.0.1", 14882));
    }
    return instance_;
}

SimConnection::SimConnection(const std::string &host, int port) : host_(host), port_(port) {}

SimConnection::~SimConnection() { disconnect(); }

bool SimConnection::connect() {
    if (sock_fd_ >= 0) return true;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (inet_pton(AF_INET, host_.c_str(), &addr.sin_addr) <= 0) {
        spdlog::error("SimConnection: invalid host '{}'", host_);
        return false;
    }

    constexpr int MAX_RETRIES = 60;
    constexpr int RETRY_INTERVAL_MS = 1000;

    for (int attempt = 1; attempt <= MAX_RETRIES; ++attempt) {
        sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock_fd_ < 0) {
            spdlog::error("SimConnection: socket() failed: {}", strerror(errno));
            return false;
        }

        if (::connect(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
            spdlog::info("SimConnection: connected to {}:{}", host_, port_);
            return true;
        }

        ::close(sock_fd_);
        sock_fd_ = -1;

        spdlog::info("SimConnection: waiting for server at {}:{} (attempt {}/{})",
                      host_, port_, attempt, MAX_RETRIES);
        std::this_thread::sleep_for(std::chrono::milliseconds(RETRY_INTERVAL_MS));
    }

    spdlog::error("SimConnection: server at {}:{} not reachable after {} seconds",
                   host_, port_, MAX_RETRIES);
    return false;
}

void SimConnection::disconnect() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

bool SimConnection::is_connected() const { return sock_fd_ >= 0; }

void SimConnection::set_command(VelocityCommand cmd) { pending_command_ = cmd; }

bool SimConnection::step_and_receive(CameraData &data) {
    if (!is_connected()) return false;

    SimRequest req{};
    req.linear_x = pending_command_.linear_x;
    req.linear_y = pending_command_.linear_y;
    req.angular_z = pending_command_.angular_z;
    if (!send_all(&req, sizeof(req))) {
        spdlog::error("SimConnection: failed to send command");
        disconnect();
        return false;
    }

    SimResponseHeader hdr{};
    if (!recv_all(&hdr, sizeof(hdr))) {
        spdlog::error("SimConnection: failed to receive header");
        disconnect();
        return false;
    }

    int w = static_cast<int>(hdr.width);
    int h = static_cast<int>(hdr.height);

    cv::Mat rgb(h, w, CV_8UC3);
    if (!recv_all(rgb.data, static_cast<size_t>(w * h * 3))) {
        spdlog::error("SimConnection: failed to receive rgb");
        disconnect();
        return false;
    }

    cv::Mat depth(h, w, CV_32FC1);
    if (!recv_all(depth.data, static_cast<size_t>(w * h) * sizeof(float))) {
        spdlog::error("SimConnection: failed to receive depth");
        disconnect();
        return false;
    }

    double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
                     .count();
    Header stamp{now, FrameId::CAMERA};

    data.rgb.header = stamp;
    data.rgb.image = rgb;

    data.depth.header = stamp;
    data.depth.image = depth;

    data.camera_info.header = stamp;
    data.camera_info.width = w;
    data.camera_info.height = h;
    data.camera_info.intrinsics =
        (cv::Mat_<double>(3, 3) << hdr.fx, 0, hdr.cx, 0, hdr.fy, hdr.cy, 0, 0, 1);
    data.camera_info.distortion = cv::Mat::zeros(1, 5, CV_64F);

    Eigen::Matrix4d tf;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) tf(r, c) = hdr.tf_matrix[r * 4 + c];

    data.tf_visodom_from_camera.header = {now, FrameId::VISUAL_ODOMETRY};
    data.tf_visodom_from_camera.child_frame_id = FrameId::CAMERA;
    data.tf_visodom_from_camera.transform.tf = tf;

    return true;
}

bool SimConnection::send_all(const void *buf, size_t len) {
    const auto *p = static_cast<const uint8_t *>(buf);
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = ::send(sock_fd_, p + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) return false;
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool SimConnection::recv_all(void *buf, size_t len) {
    auto *p = static_cast<uint8_t *>(buf);
    size_t received = 0;
    while (received < len) {
        ssize_t n = ::recv(sock_fd_, p + received, len - received, 0);
        if (n <= 0) return false;
        received += static_cast<size_t>(n);
    }
    return true;
}

}  // namespace auto_battlebot
