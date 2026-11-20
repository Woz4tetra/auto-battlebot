#include "viz/relay_app_link.hpp"

#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "viz/frame.hpp"

namespace auto_battlebot::viz {

RelayAppLink::RelayAppLink(std::string socket_path, RelayAppLinkConfig config,
                           RelayAppLinkHooks hooks)
    : socket_path_(std::move(socket_path)), config_(config), hooks_(std::move(hooks)) {}

RelayAppLink::~RelayAppLink() {
    close_app();
    close_listener();
}

void RelayAppLink::close_listener() {
    if (listen_fd_ < 0) return;
    ::close(listen_fd_);
    listen_fd_ = -1;
    ::unlink(socket_path_.c_str());
}

bool RelayAppLink::open_listener() {
    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (listen_fd_ < 0) {
        spdlog::error("socket(): {}", std::strerror(errno));
        return false;
    }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(addr.sun_path)) {
        spdlog::error("Socket path too long: {}", socket_path_);
        return false;
    }
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
    ::unlink(socket_path_.c_str());
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        spdlog::error("bind({}): {}", socket_path_, std::strerror(errno));
        return false;
    }
    ::chmod(socket_path_.c_str(), 0666);
    if (::listen(listen_fd_, 2) != 0) {
        spdlog::error("listen(): {}", std::strerror(errno));
        return false;
    }
    return true;
}

void RelayAppLink::poll_once(int timeout_ms) {
    pollfd fds[2];
    fds[0] = {listen_fd_, POLLIN, 0};
    const bool sending = app_out_sent_ < app_out_.size();
    fds[1] = {app_fd_, static_cast<short>(POLLIN | (sending ? POLLOUT : 0)), 0};
    const nfds_t count = app_fd_ >= 0 ? 2 : 1;
    int ready = ::poll(fds, count, timeout_ms);
    if (ready <= 0) return;
    if (count == 2 && (fds[1].revents & (POLLIN | POLLHUP | POLLERR))) read_app();
    if (count == 2 && app_fd_ >= 0 && (fds[1].revents & POLLOUT)) flush_to_app();
    if (fds[0].revents & POLLIN) accept_app();
}

void RelayAppLink::accept_app() {
    int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
    if (fd < 0) return;
    if (app_fd_ >= 0) {
        if (!warned_second_connection_) {
            spdlog::warn("Second app connection refused; one app at a time");
            warned_second_connection_ = true;
        }
        ::close(fd);
        return;
    }
    warned_second_connection_ = false;
    app_fd_ = fd;
    read_buffer_.clear();
    spdlog::info("auto_battlebot connected");
    if (hooks_.on_connected) hooks_.on_connected();
}

void RelayAppLink::close_app() {
    if (app_fd_ < 0) return;
    ::close(app_fd_);
    app_fd_ = -1;
    read_buffer_.clear();
    app_out_.clear();
    app_out_sent_ = 0;
    warned_app_out_full_ = false;
    spdlog::info("auto_battlebot disconnected");
    if (hooks_.on_disconnected) hooks_.on_disconnected();
}

void RelayAppLink::read_app() {
    std::byte chunk[1 << 16];
    while (true) {
        ssize_t n = ::recv(app_fd_, chunk, sizeof(chunk), 0);
        if (n == 0) {
            close_app();
            return;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            spdlog::warn("recv(): {}", std::strerror(errno));
            close_app();
            return;
        }
        read_buffer_.insert(read_buffer_.end(), chunk, chunk + n);
        if (static_cast<size_t>(n) < sizeof(chunk)) break;
    }
    size_t off = 0;
    while (read_buffer_.size() - off >= kFrameHeaderBytes) {
        uint32_t len = 0;
        size_t body_off = off;
        detail::get_u32(read_buffer_.data(), read_buffer_.size(), body_off, len);
        if (len > kMaxFrameBytes) {
            spdlog::error("Corrupt frame ({} bytes); dropping app connection", len);
            close_app();
            return;
        }
        if (read_buffer_.size() - body_off < len) break;
        if (hooks_.on_frame) hooks_.on_frame(read_buffer_.data() + body_off, len);
        off = body_off + len;
    }
    read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + static_cast<long>(off));
}

void RelayAppLink::send(const std::vector<std::byte>& frame) {
    if (app_fd_ < 0) return;
    if (app_out_.size() - app_out_sent_ + frame.size() > config_.max_out_bytes) {
        if (!warned_app_out_full_) {
            spdlog::warn("auto_battlebot is not reading; dropping frames to it");
            warned_app_out_full_ = true;
        }
        return;
    }
    if (app_out_sent_ == app_out_.size()) {
        app_out_.clear();
        app_out_sent_ = 0;
    }
    app_out_.insert(app_out_.end(), frame.begin(), frame.end());
    flush_to_app();
}

void RelayAppLink::flush_to_app() {
    while (app_out_sent_ < app_out_.size()) {
        ssize_t n = ::send(app_fd_, app_out_.data() + app_out_sent_,
                           app_out_.size() - app_out_sent_, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return;
        }
        app_out_sent_ += static_cast<size_t>(n);
    }
    app_out_.clear();
    app_out_sent_ = 0;
    warned_app_out_full_ = false;
}

void ShutdownWatchdog::arm(std::chrono::milliseconds deadline, ExitFn exit_fn) {
    std::thread([deadline, exit_fn = std::move(exit_fn)] {
        std::this_thread::sleep_for(deadline);
        exit_fn();
    }).detach();
}

}  // namespace auto_battlebot::viz
