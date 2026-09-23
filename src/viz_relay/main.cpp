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
// It also serves the web dashboard (web/dist) over HTTP with Crow. Both servers live here because
// this process outlives app restarts: when the app crashes the page still loads and shows the app
// as down.
//
// Framing is in include/viz/frame.hpp; the layout is documented in
// docs/foxglove_recording_format.md.

#include <arpa/inet.h>
#include <crow.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <foxglove/channel.hpp>
#include <foxglove/context.hpp>
#include <foxglove/server.hpp>
#include <fstream>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "viz/frame.hpp"
#include "viz/static_files.hpp"

namespace {

std::atomic<bool> g_stop{false};
constexpr std::chrono::seconds kShutdownDeadline{2};
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

struct ClientMessage {
    std::string topic;
    std::vector<std::byte> payload;
};

/** Routes Crow's log lines to spdlog. */
class CrowSpdlogHandler : public crow::ILogHandler {
   public:
    void log(std::string message, crow::LogLevel level) override {
        switch (level) {
            case crow::LogLevel::Debug:
                spdlog::debug("[http] {}", message);
                break;
            case crow::LogLevel::Info:
                spdlog::info("[http] {}", message);
                break;
            case crow::LogLevel::Warning:
                spdlog::warn("[http] {}", message);
                break;
            case crow::LogLevel::Error:
            case crow::LogLevel::Critical:
                spdlog::error("[http] {}", message);
                break;
        }
    }
};

/** "cable" for the IPv4 link-local range the dashboard port uses, "local" for loopback, and
 *  "wifi" for anything else. */
std::string link_for_address(const std::string& address) {
    std::string v4 = address;
    if (v4.starts_with("::ffff:")) v4 = v4.substr(7);
    in_addr addr{};
    if (::inet_pton(AF_INET, v4.c_str(), &addr) == 1) {
        const uint32_t ip = ntohl(addr.s_addr);
        if ((ip >> 16) == 0xA9FE) return "cable";
        if ((ip >> 24) == 127) return "local";
        return "wifi";
    }
    return address == "::1" ? "local" : "wifi";
}

/** The first of <exe_dir>/web (installed) and <exe_dir>/../web/dist (the build/ tree). */
std::filesystem::path default_web_root() {
    std::error_code ec;
    const auto exe_dir = std::filesystem::read_symlink("/proc/self/exe", ec).parent_path();
    for (const auto& candidate : {exe_dir / "web", exe_dir / ".." / "web" / "dist"}) {
        if (std::filesystem::is_directory(candidate, ec)) return candidate.lexically_normal();
    }
    return (exe_dir / ".." / "web" / "dist").lexically_normal();
}

/**
 * The dashboard's HTTP side: static files from `web_root` and /healthz. Crow runs its own
 * threads; everything here is read-only apart from the atomic app flag.
 */
class DashboardHttp {
   public:
    DashboardHttp(std::filesystem::path web_root, const std::atomic<bool>& app_connected)
        : web_root_(std::move(web_root)), app_connected_(app_connected) {}

    bool start(const std::string& host, uint16_t port) {
        crow::logger::setHandler(&log_handler_);
        app_.loglevel(crow::LogLevel::Warning);
        // The relay handles SIGINT and SIGTERM itself.
        app_.signal_clear();

        CROW_ROUTE(app_, "/healthz")
        ([this](const crow::request& req) {
            crow::response res(200);
            res.set_header("Content-Type", "application/json");
            res.set_header("Cache-Control", "no-cache");
            res.body = std::string("{\"app_connected\":") +
                       (app_connected_.load() ? "true" : "false") + ",\"link\":\"" +
                       link_for_address(req.remote_ip_address) + "\"}";
            return res;
        });
        CROW_ROUTE(app_, "/")([this](const crow::request& req) { return serve(req.url); });
        CROW_ROUTE(app_, "/<path>")
        ([this](const crow::request& req, const std::string&) { return serve(req.url); });

        if (!std::filesystem::is_directory(web_root_)) {
            spdlog::warn("Web root {} is missing; run scripts/build_web.sh", web_root_.string());
        }
        app_.bindaddr(host).port(port).concurrency(2);
        running_ = app_.run_async();
        if (app_.wait_for_server_start() != std::cv_status::no_timeout) {
            spdlog::error("Dashboard HTTP server did not start on {}:{}", host, port);
            return false;
        }
        spdlog::info("Dashboard on http://{}:{} from {}", host, port, web_root_.string());
        return true;
    }

    void stop() {
        app_.stop();
        if (running_.valid()) running_.wait();
    }

   private:
    crow::response serve(const std::string& url) const {
        if (!std::filesystem::is_directory(web_root_)) {
            crow::response res(503);
            res.set_header("Content-Type", "text/plain; charset=utf-8");
            res.body = "Dashboard not built. Run scripts/build_web.sh on the box.\n";
            return res;
        }
        const auto path = auto_battlebot::viz::resolve_static_path(web_root_, url);
        if (!path) return crow::response(404);
        std::ifstream in(*path, std::ios::binary);
        if (!in) return crow::response(404);
        crow::response res(200);
        res.body.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
        res.set_header("Content-Type", std::string(auto_battlebot::viz::content_type_for(*path)));
        // Vite hashes every asset name, so assets never change under the same URL. index.html
        // and the manifest must be revalidated so a rebuilt page reaches the tablet.
        res.set_header("Cache-Control", url.starts_with("/assets/")
                                            ? "public, max-age=31536000, immutable"
                                            : "no-cache");
        return res;
    }

    std::filesystem::path web_root_;
    const std::atomic<bool>& app_connected_;
    CrowSpdlogHandler log_handler_;
    crow::SimpleApp app_;
    std::future<void> running_;
};

// Client channel ids are only unique within one client connection.
uint64_t client_channel_key(uint32_t client_id, uint32_t channel_id) {
    return (static_cast<uint64_t>(client_id) << 32) | channel_id;
}

class Relay {
   public:
    Relay(std::string socket_path, std::string host, uint16_t port, uint16_t http_port,
          std::filesystem::path web_root)
        : socket_path_(std::move(socket_path)),
          host_(std::move(host)),
          port_(port),
          http_port_(http_port),
          web_root_(std::move(web_root)) {}

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
        // Clients publish commands back to the app (e.g. /reinit_field). The server callbacks run
        // on SDK threads, so they only queue; the main loop forwards over the app socket.
        options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish;
        options.supported_encodings = {"json"};
        options.callbacks.onClientAdvertise = [this](uint32_t client_id,
                                                     const foxglove::ClientChannel& channel) {
            std::lock_guard<std::mutex> lock(events_mutex_);
            client_topics_[client_channel_key(client_id, channel.id)] = std::string(channel.topic);
        };
        options.callbacks.onClientUnadvertise = [this](uint32_t client_id, uint32_t channel_id) {
            std::lock_guard<std::mutex> lock(events_mutex_);
            client_topics_.erase(client_channel_key(client_id, channel_id));
        };
        options.callbacks.onMessageData = [this](uint32_t client_id, uint32_t channel_id,
                                                 const std::byte* data, size_t len) {
            std::lock_guard<std::mutex> lock(events_mutex_);
            auto it = client_topics_.find(client_channel_key(client_id, channel_id));
            if (it == client_topics_.end()) return;
            client_messages_.push_back({it->second, std::vector<std::byte>(data, data + len)});
        };
        auto server = foxglove::WebSocketServer::create(std::move(options));
        if (!server.has_value()) {
            spdlog::error("Failed to start WebSocket server on {}:{}: {}", host_, port_,
                          foxglove::strerror(server.error()));
            return 1;
        }
        server_.emplace(std::move(server.value()));
        spdlog::info("Foxglove WebSocket server listening on ws://{}:{}", host_, server_->port());

        if (http_port_ != 0) {
            http_ = std::make_unique<DashboardHttp>(web_root_, app_connected_);
            // A dashboard that fails to bind is logged, not fatal: Foxglove still works.
            if (!http_->start(host_, http_port_)) http_.reset();
        }

        if (!open_listener()) return 1;
        spdlog::info("Waiting for auto_battlebot on {}", socket_path_);

        while (!g_stop.load()) {
            poll_once();
            process_subscription_events();
            forward_client_messages();
        }

        spdlog::info("Shutting down");
        // The SDK's stop() is graceful: it closes each client and waits for the close to finish.
        // A client whose network vanished (the iPad's Ethernet link dropping, say) never answers,
        // so stop() waited until TCP gave up on it, and systemd killed the relay after 90 s.
        // Nothing here needs flushing to disk, so past the deadline the relay just exits.
        std::thread([] {
            std::this_thread::sleep_for(kShutdownDeadline);
            spdlog::warn("Shutdown still waiting on a client after {} s; exiting anyway",
                         kShutdownDeadline.count());
            spdlog::default_logger()->flush();
            std::_Exit(0);
        }).detach();
        if (http_) http_->stop();
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
        app_connected_.store(true);
        read_buffer_.clear();
        spdlog::info("auto_battlebot connected");
    }

    void close_app() {
        if (app_fd_ < 0) return;
        ::close(app_fd_);
        app_fd_ = -1;
        app_connected_.store(false);
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

    void forward_client_messages() {
        std::deque<ClientMessage> messages;
        {
            std::lock_guard<std::mutex> lock(events_mutex_);
            messages.swap(client_messages_);
        }
        for (const auto& msg : messages) {
            if (app_fd_ < 0) {
                spdlog::warn("Dropping client message on {}: no app connected", msg.topic);
                continue;
            }
            spdlog::info("Forwarding client message on {} ({} bytes)", msg.topic,
                         msg.payload.size());
            send_to_app(auto_battlebot::viz::encode_client_message(msg.topic, msg.payload.data(),
                                                                   msg.payload.size()));
        }
    }

    void send_subscriber_count(const RelayChannel& ch) {
        if (app_fd_ < 0 || ch.app_channel_id == 0) return;
        send_to_app(
            auto_battlebot::viz::encode_subscriber_count(ch.app_channel_id, ch.subscribers));
    }

    void send_to_app(const std::vector<std::byte>& frame) {
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
    uint16_t http_port_;
    std::filesystem::path web_root_;
    std::atomic<bool> app_connected_{false};
    std::unique_ptr<DashboardHttp> http_;

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
    std::unordered_map<uint64_t, std::string> client_topics_;
    std::deque<ClientMessage> client_messages_;
};

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"viz_relay - Foxglove WebSocket relay for auto_battlebot"};
    std::string socket_path = auto_battlebot::viz::default_socket_path();
    std::string host = "0.0.0.0";
    uint16_t port = 8765;
    uint16_t http_port = 8080;
    std::string web_root;
    app.add_option("--socket", socket_path, "Unix socket path the app connects to");
    app.add_option("--host", host, "WebSocket and HTTP bind address");
    app.add_option("--port", port, "WebSocket port");
    app.add_option("--http-port", http_port, "Dashboard HTTP port; 0 disables it");
    app.add_option(
        "--web-root", web_root,
        "Built dashboard directory (default: <exe_dir>/web, then <exe_dir>/../web/dist)");
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [viz_relay] [%^%l%$] %v");
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    Relay relay(socket_path, host, port, http_port,
                web_root.empty() ? default_web_root() : std::filesystem::path(web_root));
    return relay.run();
}
