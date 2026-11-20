
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
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "host/dashboard_network.hpp"
#include "viz/frame.hpp"
#include "viz/relay_app_link.hpp"
#include "viz/static_files.hpp"

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
    std::optional<uint32_t> app_channel_id;
};

struct SubscriptionEvent {
    uint64_t channel_id;
    int delta;
};

struct ClientMessage {
    std::string topic;
    std::vector<std::byte> payload;
};

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

enum class ClientLink { CABLE, LOCAL, WIFI };

const char* link_name(ClientLink link) {
    switch (link) {
        case ClientLink::CABLE:
            return "cable";
        case ClientLink::LOCAL:
            return "local";
        case ClientLink::WIFI:
            return "wifi";
    }
    return "wifi";
}

ClientLink link_for_address(const std::string& address) {
    std::string v4 = address;
    if (v4.starts_with("::ffff:")) v4 = v4.substr(7);
    in_addr addr{};
    if (::inet_pton(AF_INET, v4.c_str(), &addr) == 1) {
        const uint32_t ip = ntohl(addr.s_addr);
        if (auto_battlebot::is_dashboard_cable_address(ip)) return ClientLink::CABLE;
        if ((ip >> 24) == 127) return ClientLink::LOCAL;
        return ClientLink::WIFI;
    }
    return address == "::1" ? ClientLink::LOCAL : ClientLink::WIFI;
}

std::filesystem::path default_web_root() {
    std::error_code ec;
    const auto exe_dir = std::filesystem::read_symlink("/proc/self/exe", ec).parent_path();
    for (const auto& candidate : {exe_dir / "web", exe_dir / ".." / "web" / "dist"}) {
        if (std::filesystem::is_directory(candidate, ec)) return candidate.lexically_normal();
    }
    return (exe_dir / ".." / "web" / "dist").lexically_normal();
}

class DashboardHttp {
   public:
    DashboardHttp(std::filesystem::path web_root, const std::atomic<bool>& app_connected)
        : web_root_(std::move(web_root)), app_connected_(app_connected) {}

    bool start(const std::string& host, uint16_t port) {
        crow::logger::setHandler(&log_handler_);
        app_.loglevel(crow::LogLevel::Warning);
        app_.signal_clear();

        CROW_ROUTE(app_, "/healthz")
        ([this](const crow::request& req) {
            crow::response res(200);
            res.set_header("Content-Type", "application/json");
            res.set_header("Cache-Control", "no-cache");
            res.body = std::string("{\"app_connected\":") +
                       (app_connected_.load() ? "true" : "false") + ",\"link\":\"" +
                       link_name(link_for_address(req.remote_ip_address)) + "\"}";
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
        res.set_header("Cache-Control", std::string(auto_battlebot::viz::cache_control_for(url)));
        return res;
    }

    std::filesystem::path web_root_;
    const std::atomic<bool>& app_connected_;
    CrowSpdlogHandler log_handler_;
    crow::SimpleApp app_;
    std::future<void> running_;
};

uint64_t client_channel_key(uint32_t client_id, uint32_t channel_id) {
    return (static_cast<uint64_t>(client_id) << 32) | channel_id;
}

class SdkEventQueue {
   public:
    void subscribe(uint64_t channel_id, int delta) {
        std::lock_guard<std::mutex> lock(mutex_);
        events_.push_back({channel_id, delta});
    }

    void client_advertise(uint32_t client_id, uint32_t channel_id, std::string topic) {
        std::lock_guard<std::mutex> lock(mutex_);
        client_topics_[client_channel_key(client_id, channel_id)] = std::move(topic);
    }

    void client_unadvertise(uint32_t client_id, uint32_t channel_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        client_topics_.erase(client_channel_key(client_id, channel_id));
    }

    void client_message(uint32_t client_id, uint32_t channel_id, const std::byte* data,
                        size_t len) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = client_topics_.find(client_channel_key(client_id, channel_id));
        if (it == client_topics_.end()) return;
        client_messages_.push_back({it->second, std::vector<std::byte>(data, data + len)});
    }

    std::deque<SubscriptionEvent> take_subscription_events() {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(events_, {});
    }

    std::deque<ClientMessage> take_client_messages() {
        std::lock_guard<std::mutex> lock(mutex_);
        return std::exchange(client_messages_, {});
    }

   private:
    std::mutex mutex_;
    std::deque<SubscriptionEvent> events_;
    std::unordered_map<uint64_t, std::string> client_topics_;
    std::deque<ClientMessage> client_messages_;
};

class Relay {
   public:
    Relay(std::string socket_path, std::string host, uint16_t port, uint16_t http_port,
          std::filesystem::path web_root, std::chrono::seconds shutdown_deadline)
        : socket_path_(std::move(socket_path)),
          host_(std::move(host)),
          port_(port),
          http_port_(http_port),
          web_root_(std::move(web_root)),
          shutdown_deadline_(shutdown_deadline),
          link_(socket_path_, {},
                {
                    .on_frame = [this](const std::byte* body,
                                       size_t len) { handle_frame(body, len); },
                    .on_connected = [this] { app_connected_.store(true); },
                    .on_disconnected = [this] { on_app_disconnected(); },
                }) {}

    int run() {
        context_ = foxglove::Context::create();

        foxglove::WebSocketServerOptions options;
        options.context = context_;
        options.name = "auto_battlebot";
        options.host = host_;
        options.port = port_;
        SdkEventQueue* queue = &sdk_events_;
        options.callbacks.onSubscribe = [queue](uint64_t channel_id,
                                                const foxglove::ClientMetadata&) {
            queue->subscribe(channel_id, +1);
        };
        options.callbacks.onUnsubscribe = [queue](uint64_t channel_id,
                                                  const foxglove::ClientMetadata&) {
            queue->subscribe(channel_id, -1);
        };
        options.capabilities = foxglove::WebSocketServerCapabilities::ClientPublish;
        options.supported_encodings = {"json"};
        options.callbacks.onClientAdvertise = [queue](uint32_t client_id,
                                                      const foxglove::ClientChannel& channel) {
            queue->client_advertise(client_id, channel.id, std::string(channel.topic));
        };
        options.callbacks.onClientUnadvertise = [queue](uint32_t client_id, uint32_t channel_id) {
            queue->client_unadvertise(client_id, channel_id);
        };
        options.callbacks.onMessageData = [queue](uint32_t client_id, uint32_t channel_id,
                                                  const std::byte* data, size_t len) {
            queue->client_message(client_id, channel_id, data, len);
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
            if (!http_->start(host_, http_port_)) http_.reset();
        }

        if (!link_.open_listener()) return 1;
        spdlog::info("Waiting for auto_battlebot on {}", socket_path_);

        while (!g_stop.load()) {
            link_.poll_once(20);
            process_subscription_events();
            forward_client_messages();
        }

        spdlog::info("Shutting down");
        auto_battlebot::viz::ShutdownWatchdog::arm(
            std::chrono::duration_cast<std::chrono::milliseconds>(shutdown_deadline_), [this] {
                spdlog::warn("Shutdown still waiting on a client after {} s; exiting anyway",
                             shutdown_deadline_.count());
                spdlog::default_logger()->flush();
                std::_Exit(0);
            });
        if (http_) http_->stop();
        link_.close_app();
        link_.close_listener();
        for (auto& [topic, ch] : channels_) {
            if (ch.channel) ch.channel->close();
        }
        server_->stop();
        return 0;
    }

   private:
    void on_app_disconnected() {
        app_connected_.store(false);
        app_channels_.clear();
        for (auto& [topic, ch] : channels_) ch.app_channel_id.reset();
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
            ch.channel->log(msg.payload.data(), msg.payload.size(), msg.log_time_ns);
            if (ch.latch) {
                ch.retained.assign(msg.payload.begin(), msg.payload.end());
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
        const std::deque<SubscriptionEvent> events = sdk_events_.take_subscription_events();
        for (const auto& event : events) {
            auto id_it = channel_by_id_.find(event.channel_id);
            if (id_it == channel_by_id_.end()) continue;
            auto ch_it = channels_.find(id_it->second);
            if (ch_it == channels_.end()) continue;
            RelayChannel& ch = ch_it->second;
            if (event.delta > 0) {
                ++ch.subscribers;
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
        const std::deque<ClientMessage> messages = sdk_events_.take_client_messages();
        for (const auto& msg : messages) {
            if (!link_.connected()) {
                spdlog::warn("Dropping client message on {}: no app connected", msg.topic);
                continue;
            }
            spdlog::info("Forwarding client message on {} ({} bytes)", msg.topic,
                         msg.payload.size());
            link_.send(auto_battlebot::viz::encode_client_message(msg.topic, msg.payload.data(),
                                                                  msg.payload.size()));
        }
    }

    void send_subscriber_count(const RelayChannel& ch) {
        if (!link_.connected() || !ch.app_channel_id) return;
        link_.send(
            auto_battlebot::viz::encode_subscriber_count(*ch.app_channel_id, ch.subscribers));
    }

    std::string socket_path_;
    std::string host_;
    uint16_t port_;
    uint16_t http_port_;
    std::filesystem::path web_root_;
    std::chrono::seconds shutdown_deadline_;
    std::atomic<bool> app_connected_{false};
    std::unique_ptr<DashboardHttp> http_;

    foxglove::Context context_;
    std::optional<foxglove::WebSocketServer> server_;

    std::map<std::string, RelayChannel> channels_;
    std::unordered_map<uint64_t, std::string> channel_by_id_;
    std::unordered_map<uint32_t, std::map<std::string, RelayChannel>::iterator> app_channels_;

    SdkEventQueue sdk_events_;
    auto_battlebot::viz::RelayAppLink link_;
};

}  // namespace

int main(int argc, char** argv) {
    CLI::App app{"viz_relay - Foxglove WebSocket relay for auto_battlebot"};
    std::string socket_path = auto_battlebot::viz::default_socket_path();
    std::string host = "0.0.0.0";
    uint16_t port = 8765;
    uint16_t http_port = 8080;
    std::string web_root;
    int shutdown_deadline_s = 2;
    app.add_option("--socket", socket_path, "Unix socket path the app connects to");
    app.add_option("--host", host, "WebSocket and HTTP bind address");
    app.add_option("--port", port, "WebSocket port");
    app.add_option("--http-port", http_port, "Dashboard HTTP port; 0 disables it");
    app.add_option(
        "--web-root", web_root,
        "Built dashboard directory (default: <exe_dir>/web, then <exe_dir>/../web/dist)");
    app.add_option("--shutdown-deadline-s", shutdown_deadline_s,
                   "Seconds to wait for clients to close before exiting anyway");
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [viz_relay] [%^%l%$] %v");
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    Relay relay(socket_path, host, port, http_port,
                web_root.empty() ? default_web_root() : std::filesystem::path(web_root),
                std::chrono::seconds(shutdown_deadline_s));
    return relay.run();
}
