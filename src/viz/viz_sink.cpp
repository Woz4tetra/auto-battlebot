#include "viz/viz_sink.hpp"

#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace auto_battlebot {

namespace {
// Bound on queued bytes while the relay is slow or absent. Above this the newest message is
// dropped rather than growing without limit; live viz is best effort.
constexpr size_t kMaxQueuedBytes = 32u * 1024u * 1024u;
constexpr auto kReconnectInterval = std::chrono::seconds(1);
}  // namespace

VizSink::VizSink(std::string socket_path) : socket_path_(std::move(socket_path)) {
    thread_ = std::thread([this] { run(); });
}

VizSink::~VizSink() {
    stop_.store(true);
    cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    disconnect();
}

uint32_t VizSink::advertise(const std::string& topic, const std::string& message_encoding,
                            const VizSchema& schema, bool latch) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto it = channel_by_topic_.find(topic); it != channel_by_topic_.end()) {
        return it->second;
    }
    viz::AdvertiseFrame frame;
    frame.channel_id = static_cast<uint32_t>(advertises_.size() + 1);
    frame.latch = latch;
    frame.topic = topic;
    frame.message_encoding = message_encoding;
    frame.schema_name = schema.name;
    frame.schema_encoding = schema.encoding;
    frame.schema_data = schema.data;
    channel_by_topic_[topic] = frame.channel_id;
    advertises_.push_back(frame);
    // Queue it for the live connection too; a reconnect replays advertises_ instead.
    auto encoded = viz::encode_advertise(frame);
    queued_bytes_ += encoded.size();
    queue_.push_back(std::move(encoded));
    cv_.notify_one();
    return frame.channel_id;
}

void VizSink::publish(uint32_t channel_id, const std::byte* data, size_t len,
                      uint64_t log_time_ns) {
    if (!connected_.load()) {
        dropped_.fetch_add(1);
        return;
    }
    auto frame = viz::encode_message(channel_id, log_time_ns, data, len);
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_bytes_ + frame.size() > kMaxQueuedBytes) {
        dropped_.fetch_add(1);
        return;
    }
    queued_bytes_ += frame.size();
    queue_.push_back(std::move(frame));
    cv_.notify_one();
}

uint32_t VizSink::num_subscribers(uint32_t channel_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscribers_.find(channel_id);
    return it == subscribers_.end() ? 0 : it->second;
}

bool VizSink::try_connect() {
    int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) return false;
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    if (socket_path_.size() >= sizeof(addr.sun_path)) {
        ::close(fd);
        spdlog::error("[VizSink] Socket path too long: {}", socket_path_);
        return false;
    }
    std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
    // A unix socket connect() cannot hang: it returns ENOENT or ECONNREFUSED at once when the
    // relay is not listening, so a missing relay can never stall startup.
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return false;
    }
    fd_ = fd;
    return true;
}

void VizSink::disconnect() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    connected_.store(false);
    std::lock_guard<std::mutex> lock(mutex_);
    subscribers_.clear();
    read_buffer_.clear();
}

bool VizSink::send_all(const std::vector<std::byte>& frame) {
    size_t sent = 0;
    while (sent < frame.size()) {
        ssize_t n = ::send(fd_, frame.data() + sent, frame.size() - sent, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

void VizSink::read_incoming() {
    std::byte chunk[4096];
    while (true) {
        ssize_t n = ::recv(fd_, chunk, sizeof(chunk), MSG_DONTWAIT);
        if (n <= 0) break;
        read_buffer_.insert(read_buffer_.end(), chunk, chunk + n);
    }
    size_t off = 0;
    while (read_buffer_.size() - off >= viz::kFrameHeaderBytes) {
        uint32_t len = 0;
        size_t tmp = off;
        viz::detail::get_u32(read_buffer_.data(), read_buffer_.size(), tmp, len);
        if (len > viz::kMaxFrameBytes) {
            spdlog::warn("[VizSink] Corrupt frame from relay; reconnecting");
            disconnect();
            return;
        }
        if (read_buffer_.size() - tmp < len) break;
        const std::byte* body = read_buffer_.data() + tmp;
        if (viz::frame_kind(body, len) == viz::FrameKind::SUBSCRIBER_COUNT) {
            viz::SubscriberCountFrame frame;
            if (viz::decode_subscriber_count(body, len, frame)) {
                std::lock_guard<std::mutex> lock(mutex_);
                subscribers_[frame.channel_id] = frame.count;
            }
        }
        off = tmp + len;
    }
    read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + static_cast<long>(off));
}

void VizSink::run() {
    bool warned = false;
    while (!stop_.load()) {
        if (fd_ < 0) {
            if (!try_connect()) {
                if (!warned) {
                    spdlog::warn(
                        "[VizSink] viz_relay not reachable at {}; live viz off until it is "
                        "(recording and control unaffected)",
                        socket_path_);
                    warned = true;
                }
                std::unique_lock<std::mutex> lock(mutex_);
                // Nothing reaches the relay while disconnected, so the queue only ever holds
                // advertises here; keep those, they replay from advertises_ anyway.
                queue_.clear();
                queued_bytes_ = 0;
                cv_.wait_for(lock, kReconnectInterval, [this] { return stop_.load(); });
                continue;
            }
            if (!lost_warned_) {
                spdlog::info("[VizSink] Connected to viz_relay at {}", socket_path_);
            }
            warned = false;
            std::vector<std::vector<std::byte>> replay;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                queue_.clear();
                queued_bytes_ = 0;
                for (const auto& adv : advertises_) replay.push_back(viz::encode_advertise(adv));
            }
            bool ok = true;
            for (const auto& frame : replay) {
                if (!send_all(frame)) {
                    ok = false;
                    break;
                }
            }
            if (!ok) {
                // The relay accepted and then closed us: it already has an app attached.
                if (!lost_warned_) {
                    spdlog::warn(
                        "[VizSink] viz_relay refused this connection (another app attached?); "
                        "retrying quietly");
                    lost_warned_ = true;
                }
                disconnect();
                continue;
            }
            connected_.store(true);
            if (lost_warned_) {
                spdlog::info("[VizSink] Reconnected to viz_relay at {}", socket_path_);
                lost_warned_ = false;
            }
        }

        std::vector<std::byte> frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(50),
                         [this] { return stop_.load() || !queue_.empty(); });
            if (!queue_.empty()) {
                frame = std::move(queue_.front());
                queue_.pop_front();
                queued_bytes_ -= frame.size();
            }
        }
        if (!frame.empty() && !send_all(frame)) {
            // Once per outage: a relay that refuses us (another app holds it) would otherwise
            // produce a warning every reconnect interval.
            if (!lost_warned_) {
                spdlog::warn("[VizSink] Lost connection to viz_relay ({}); reconnecting",
                             std::strerror(errno));
                lost_warned_ = true;
            }
            disconnect();
            continue;
        }
        read_incoming();
    }
}

}  // namespace auto_battlebot
