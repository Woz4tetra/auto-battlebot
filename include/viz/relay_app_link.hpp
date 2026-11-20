#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace auto_battlebot::viz {

struct RelayAppLinkConfig {
    size_t max_out_bytes = 1u << 20;
};

struct RelayAppLinkHooks {
    std::function<void(const std::byte *body, size_t len)> on_frame;
    std::function<void()> on_connected;
    std::function<void()> on_disconnected;
};

class RelayAppLink {
   public:
    RelayAppLink(std::string socket_path, RelayAppLinkConfig config, RelayAppLinkHooks hooks);
    ~RelayAppLink();

    RelayAppLink(const RelayAppLink &) = delete;
    RelayAppLink &operator=(const RelayAppLink &) = delete;

    bool open_listener();
    void close_listener();
    void poll_once(int timeout_ms);
    bool connected() const { return app_fd_ >= 0; }
    void send(const std::vector<std::byte> &frame);
    void close_app();

   private:
    void read_app();
    void accept_app();
    void flush_to_app();

    std::string socket_path_;
    RelayAppLinkConfig config_;
    RelayAppLinkHooks hooks_;
    int listen_fd_ = -1;
    int app_fd_ = -1;
    bool warned_second_connection_ = false;
    std::vector<std::byte> read_buffer_;
    std::vector<std::byte> app_out_;
    size_t app_out_sent_ = 0;
    bool warned_app_out_full_ = false;
};

class ShutdownWatchdog {
   public:
    using ExitFn = std::function<void()>;
    static void arm(std::chrono::milliseconds deadline, ExitFn exit_fn);
};

}  // namespace auto_battlebot::viz
