// viz_relay: the process that owns the Foxglove WebSocket server.
//
// auto_battlebot connects over a unix socket and streams already-encoded messages; this process
// re-publishes them on ws://<host>:8765 with foxglove::WebSocketServer. Because the server lives
// here and not in the app, Foxglove stays connected across app restarts: channels are keyed by
// topic and reused when the app reconnects with the same schema, so panels never blink.
//
// It also latches: the last message on a topic advertised with latch=1 is re-sent whenever a new
// client subscribes, which is what lets Foxglove attach mid-run and still see the field border
// and inlier cloud that were published once at startup.
//
// Framing is in include/viz/frame.hpp; the layout is documented in
// docs/foxglove_recording_format.md.

#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <deque>
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/server.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "viz/frame.hpp"

namespace {

std::atomic<bool> g_stop{false};
void handle_signal(int) { g_stop.store(true); }

struct RelayChannel {
    std::unique_ptr<foxglove::RawChannel> channel;
    std::string message_encoding;
    std::string schema_name;
    std::string schema_encoding;
    std::string schema_data;
    bool latch = false;
    std::vector<std::byte> retained;
    uint64_t retained_log_time = 0;
    uint32_t subscribers = 0;
    // The app's id for this topic on the current connection, 0 when the app has not (re)advertised
    // it since connecting. Subscriber counts are only sent back for advertised channels.
    uint32_t app_channel_id = 0;
};

struct SubscriptionEvent {
    uint64_t channel_id;
    int delta;
};

class Relay {
   public:
    Relay(std::string socket_path, std::string host, uint16_t port)
        : socket_path_(std::move(socket_path)), host_(std::move(host)), port_(port) {}

    int run() {
        context_ = foxglove::Context::create();

        foxglove::WebSocketServerOptions options;
        options.context = context_;
        options.name = "auto_battlebot";
        options.host = host_;
        options.port = port_;
        options.callbacks.onSubscribe = [this](uint64_t channel_id,
                                               const foxglove::ClientMetadata&) {
            std::lock_guard<std::mutex> lock(events_mutex_);
            events_.push_back({channel_id, +1});
        };
        options.callbacks.onUnsubscribe = [this](uint64_t channel_id,
                                                 const foxglove::ClientMetadata&) {
            std::lock_guard<std::mutex> lock(events_mutex_);
            events_.push_back({channel_id, -1});
        };
        auto server = foxglove::WebSocketServer::create(std::move(options));
        if (!server.has_value()) {
            spdlog::error("Failed to start WebSocket server on {}:{}: {}", host_, port_,
                          foxglove::strerror(server.error()));
            return 1;
        }
        server_.emplace(std::move(server.value()));
        spdlog::info("Foxglove WebSocket server listening on ws://{}:{}", host_, server_->port());

        if (!open_listener()) return 1;
        spdlog::info("Waiting for auto_battlebot on {}", socket_path_);

        while (!g_stop.load()) {
            poll_once();
            process_subscription_events();
        }

        spdlog::info("Shutting down");
        close_app();
        if (listen_fd_ >= 0) ::close(listen_fd_);
        ::unlink(socket_path_.c_str());
        for (auto& [topic, ch] : channels_) {
            if (ch.channel) ch.channel->close();
        }
        server_->stop();
        return 0;
    }

   private:
    bool open_listener() {
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
        // A stale socket file from a previous relay would make bind() fail with EADDRINUSE.
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

    void poll_once() {
        pollfd fds[2];
        fds[0] = {listen_fd_, POLLIN, 0};
        fds[1] = {app_fd_, POLLIN, 0};
        const nfds_t count = app_fd_ >= 0 ? 2 : 1;
        int ready = ::poll(fds, count, 20);
        if (ready <= 0) return;
        if (fds[0].revents & POLLIN) accept_app();
        if (count == 2 && (fds[1].revents & (POLLIN | POLLHUP | POLLERR))) read_app();
    }

    void accept_app() {
        int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
        if (fd < 0) return;
        if (app_fd_ >= 0) {
            // Keep the app that is already attached. Dropping it in favour of the newcomer
            // made two apps (a dev replay next to a service instance, say) ping-pong forever,
            // since each reconnects the moment it loses the socket. A dead app is noticed by
            // EOF on read, so the slot frees itself.
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
    }

    void close_app() {
        if (app_fd_ < 0) return;
        ::close(app_fd_);
        app_fd_ = -1;
        read_buffer_.clear();
        app_channels_.clear();
        for (auto& [topic, ch] : channels_) ch.app_channel_id = 0;
        spdlog::info("auto_battlebot disconnected; channels stay advertised");
    }

    void read_app() {
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
            // Keep draining while data is available, but do not starve the poll loop on a
            // firehose: one 64 KB chunk per iteration is plenty at 60 Hz.
            if (static_cast<size_t>(n) < sizeof(chunk)) break;
        }
        size_t off = 0;
        while (read_buffer_.size() - off >= auto_battlebot::viz::kFrameHeaderBytes) {
            uint32_t len = 0;
            size_t body_off = off;
            auto_battlebot::viz::detail::get_u32(read_buffer_.data(), read_buffer_.size(), body_off,
                                                 len);
            if (len > auto_battlebot::viz::kMaxFrameBytes) {
                spdlog::error("Corrupt frame ({} bytes); dropping app connection", len);
                close_app();
                return;
            }
            if (read_buffer_.size() - body_off < len) break;
            handle_frame(read_buffer_.data() + body_off, len);
            off = body_off + len;
        }
        read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + static_cast<long>(off));
    }

    void handle_frame(const std::byte* body, size_t len) {
        using namespace auto_battlebot::viz;
        auto kind = frame_kind(body, len);
        if (!kind) return;
        if (*kind == FrameKind::ADVERTISE) {
            AdvertiseFrame adv;
            if (!decode_advertise(body, len, adv)) {
                spdlog::warn("Malformed ADVERTISE frame");
                return;
            }
            handle_advertise(adv);
        } else if (*kind == FrameKind::MESSAGE) {
            MessageFrame msg;
            if (!decode_message(body, len, msg)) return;
            auto it = app_channels_.find(msg.channel_id);
            if (it == app_channels_.end()) return;
            RelayChannel& ch = it->second->second;
            ch.channel->log(msg.payload, msg.payload_len, msg.log_time_ns);
            if (ch.latch) {
                ch.retained.assign(msg.payload, msg.payload + msg.payload_len);
                ch.retained_log_time = msg.log_time_ns;
            }
        }
    }

    void handle_advertise(const auto_battlebot::viz::AdvertiseFrame& adv) {
        auto it = channels_.find(adv.topic);
        const bool same_schema = it != channels_.end() &&
                                 it->second.message_encoding == adv.message_encoding &&
                                 it->second.schema_name == adv.schema_name &&
                                 it->second.schema_encoding == adv.schema_encoding &&
                                 it->second.schema_data == adv.schema_data;
        if (it != channels_.end() && !same_schema) {
            spdlog::info("Schema changed on {}; re-advertising", adv.topic);
            it->second.channel->close();
            channel_by_id_.erase(it->second.channel->id());
            channels_.erase(it);
            it = channels_.end();
        }
        if (it == channels_.end()) {
            RelayChannel ch;
            ch.message_encoding = adv.message_encoding;
            ch.schema_name = adv.schema_name;
            ch.schema_encoding = adv.schema_encoding;
            ch.schema_data = adv.schema_data;
            std::optional<foxglove::Schema> schema;
            if (!adv.schema_name.empty()) {
                schema = foxglove::Schema{adv.schema_name, adv.schema_encoding,
                                          reinterpret_cast<const std::byte*>(ch.schema_data.data()),
                                          ch.schema_data.size()};
            }
            auto created =
                foxglove::RawChannel::create(adv.topic, adv.message_encoding, schema, context_);
            if (!created.has_value()) {
                spdlog::error("Failed to create channel {}: {}", adv.topic,
                              foxglove::strerror(created.error()));
                return;
            }
            ch.channel = std::make_unique<foxglove::RawChannel>(std::move(created.value()));
            channel_by_id_[ch.channel->id()] = adv.topic;
            it = channels_.emplace(adv.topic, std::move(ch)).first;
            spdlog::info("Advertised {} ({}, {})", adv.topic, adv.message_encoding,
                         adv.schema_name);
        }
        it->second.latch = adv.latch;
        it->second.app_channel_id = adv.channel_id;
        app_channels_[adv.channel_id] = it;
        send_subscriber_count(it->second);
    }

    void process_subscription_events() {
        std::deque<SubscriptionEvent> events;
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            events.swap(events_);
        }
        for (const auto& event : events) {
            auto id_it = channel_by_id_.find(event.channel_id);
            if (id_it == channel_by_id_.end()) continue;
            auto ch_it = channels_.find(id_it->second);
            if (ch_it == channels_.end()) continue;
            RelayChannel& ch = ch_it->second;
            if (event.delta > 0) {
                ++ch.subscribers;
                // Re-send the retained message so a client that attaches mid-run sees geometry
                // that was published once at startup. log() reaches every subscriber, which is
                // harmless for these topics.
                if (ch.latch && !ch.retained.empty()) {
                    ch.channel->log(ch.retained.data(), ch.retained.size(), ch.retained_log_time);
                }
            } else if (ch.subscribers > 0) {
                --ch.subscribers;
            }
            send_subscriber_count(ch);
        }
    }

    void send_subscriber_count(const RelayChannel& ch) {
        if (app_fd_ < 0 || ch.app_channel_id == 0) return;
        auto frame =
            auto_battlebot::viz::encode_subscriber_count(ch.app_channel_id, ch.subscribers);
        size_t sent = 0;
        while (sent < frame.size()) {
            ssize_t n = ::send(app_fd_, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
            if (n < 0) {
                if (errno == EINTR) continue;
                // The app reads counts opportunistically; a full pipe is not worth a disconnect.
                return;
            }
            sent += static_cast<size_t>(n);
        }
    }

    std::string socket_path_;
    std::string host_;
    uint16_t port_;

    foxglove::Context context_;
    std::optional<foxglove::WebSocketServer> server_;

    int listen_fd_ = -1;
    int app_fd_ = -1;
    bool warned_second_connection_ = false;
    std::vector<std::byte> read_buffer_;

    std::map<std::string, RelayChannel> channels_;
    std::unordered_map<uint64_t, std::string> channel_by_id_;
    std::unordered_map<uint32_t, std::map<std::string, RelayChannel>::iterator> app_channels_;

    std::mutex events_mutex_;
    std::deque<SubscriptionEvent> events_;
};

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"viz_relay - Foxglove WebSocket relay for auto_battlebot"};
    std::string socket_path = auto_battlebot::viz::default_socket_path();
    std::string host = "0.0.0.0";
    uint16_t port = 8765;
    app.add_option("--socket", socket_path, "Unix socket path the app connects to");
    app.add_option("--host", host, "WebSocket bind address");
    app.add_option("--port", port, "WebSocket port");
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [viz_relay] [%^%l%$] %v");
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    Relay relay(socket_path, host, port);
    return relay.run();
}
