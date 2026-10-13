#include "viz/viz_sink.hpp"

#include <fcntl.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace auto_battlebot {

namespace {
constexpr size_t kMaxBulkBytes = 32u * 1024u * 1024u;
constexpr size_t kMaxPriorityBytes = 4u * 1024u * 1024u;
constexpr auto kReconnectInterval = std::chrono::seconds(1);
/** A relay that takes nothing for this long is stuck. Its own poll loop runs every 20 ms. */
constexpr auto kSendStallTimeout = std::chrono::seconds(2);
/** A connection the relay closes sooner than this was refused, not lost. */
constexpr auto kSettleTime = std::chrono::seconds(1);
/** Upper bound on a poll() so stop and the stall timer are checked even when nothing happens. */
constexpr int kPollMs = 50;

bool is_bulk_schema(const std::string& name) {
    return name == "foxglove.CompressedImage" || name == "foxglove.CompressedVideo" ||
           name == "foxglove.RawImage" || name == "foxglove.PointCloud";
}
}  // namespace

VizSink::VizSink(std::string socket_path) : socket_path_(std::move(socket_path)) {
    wake_fd_ = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    thread_ = std::thread([this] { run(); });
}

VizSink::~VizSink() {
    stop_.store(true);
    wake();
    if (thread_.joinable()) thread_.join();
    disconnect();
    if (wake_fd_ >= 0) ::close(wake_fd_);
}

void VizSink::wake() {
    if (wake_fd_ < 0) return;
    const uint64_t one = 1;
    // A full counter still leaves the fd readable, which is all the wake needs.
    [[maybe_unused]] const ssize_t n = ::write(wake_fd_, &one, sizeof(one));
}

uint32_t VizSink::advertise(const std::string& topic, const std::string& message_encoding,
                            const VizSchema& schema, bool latch) {
    uint32_t channel_id = 0;
    {
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
        if (is_bulk_schema(schema.name)) bulk_channels_.insert(frame.channel_id);
        advertises_.push_back(frame);
        // Priority, so it reaches the relay before any message on the channel, bulk or not.
        auto encoded = viz::encode_advertise(frame);
        priority_bytes_ += encoded.size();
        priority_queue_.push_back(std::move(encoded));
        channel_id = frame.channel_id;
    }
    wake();
    return channel_id;
}

void VizSink::publish(uint32_t channel_id, const std::byte* data, size_t len,
                      uint64_t log_time_ns) {
    if (!connected_.load()) {
        dropped_.fetch_add(1);
        return;
    }
    auto frame = viz::encode_message(channel_id, log_time_ns, data, len);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool bulk = bulk_channels_.count(channel_id) != 0;
        size_t& bytes = bulk ? bulk_bytes_ : priority_bytes_;
        if (bytes + frame.size() > (bulk ? kMaxBulkBytes : kMaxPriorityBytes)) {
            dropped_.fetch_add(1);
            return;
        }
        bytes += frame.size();
        (bulk ? bulk_queue_ : priority_queue_).push_back(std::move(frame));
    }
    wake();
}

uint32_t VizSink::num_subscribers(uint32_t channel_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subscribers_.find(channel_id);
    return it == subscribers_.end() ? 0 : it->second;
}

void VizSink::set_client_message_handler(ClientMessageHandler handler) {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    client_message_handler_ = std::move(handler);
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
    // A blocking connect on a unix socket returns at once: the relay's backlog, or a refusal.
    if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        return false;
    }
    ::fcntl(fd, F_SETFL, ::fcntl(fd, F_GETFL) | O_NONBLOCK);
    fd_ = fd;
    return true;
}

void VizSink::clear_queues_locked() {
    priority_queue_.clear();
    bulk_queue_.clear();
    priority_bytes_ = 0;
    bulk_bytes_ = 0;
}

void VizSink::disconnect() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    connected_.store(false);
    out_.clear();
    out_sent_ = 0;
    read_buffer_.clear();
    std::lock_guard<std::mutex> lock(mutex_);
    subscribers_.clear();
    clear_queues_locked();
}

void VizSink::lose_connection(const char* reason) {
    if (!lost_warned_) {
        if (settled_) {
            spdlog::warn("[VizSink] Lost connection to viz_relay ({}); reconnecting", reason);
        } else {
            spdlog::warn(
                "[VizSink] viz_relay closed this connection at once (another app attached?); "
                "retrying every {} s",
                kReconnectInterval.count());
        }
        lost_warned_ = true;
    }
    disconnect();
    // Every retry waits, so a relay that accepts and then refuses is not hammered in a loop.
    retry_at_ = std::chrono::steady_clock::now() + kReconnectInterval;
}

void VizSink::wait_for_wake(int timeout_ms) {
    pollfd fd{wake_fd_, POLLIN, 0};
    if (::poll(&fd, 1, timeout_ms) > 0) {
        uint64_t count = 0;
        [[maybe_unused]] const ssize_t n = ::read(wake_fd_, &count, sizeof(count));
    }
}

bool VizSink::take_next_frame() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& queue = !priority_queue_.empty() ? priority_queue_ : bulk_queue_;
    if (queue.empty()) return false;
    out_ = std::move(queue.front());
    queue.pop_front();
    (&queue == &priority_queue_ ? priority_bytes_ : bulk_bytes_) -= out_.size();
    out_sent_ = 0;
    last_send_progress_ = std::chrono::steady_clock::now();
    return true;
}

bool VizSink::send_pending() {
    while (true) {
        if (out_sent_ == out_.size() && !take_next_frame()) return true;
        const ssize_t n =
            ::send(fd_, out_.data() + out_sent_, out_.size() - out_sent_, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return errno == EAGAIN || errno == EWOULDBLOCK;
        }
        out_sent_ += static_cast<size_t>(n);
        last_send_progress_ = std::chrono::steady_clock::now();
    }
}

bool VizSink::read_incoming() {
    std::byte chunk[4096];
    while (true) {
        const ssize_t n = ::recv(fd_, chunk, sizeof(chunk), 0);
        if (n == 0) return false;
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            return false;
        }
        read_buffer_.insert(read_buffer_.end(), chunk, chunk + n);
    }
    size_t off = 0;
    while (read_buffer_.size() - off >= viz::kFrameHeaderBytes) {
        uint32_t len = 0;
        size_t tmp = off;
        viz::detail::get_u32(read_buffer_.data(), read_buffer_.size(), tmp, len);
        if (len > viz::kMaxFrameBytes) {
            spdlog::warn("[VizSink] Corrupt frame from relay; reconnecting");
            return false;
        }
        if (read_buffer_.size() - tmp < len) break;
        const std::byte* body = read_buffer_.data() + tmp;
        const auto kind = viz::frame_kind(body, len);
        if (kind == viz::FrameKind::SUBSCRIBER_COUNT) {
            viz::SubscriberCountFrame frame;
            if (viz::decode_subscriber_count(body, len, frame)) {
                std::lock_guard<std::mutex> lock(mutex_);
                subscribers_[frame.channel_id] = frame.count;
            }
        } else if (kind == viz::FrameKind::CLIENT_MESSAGE) {
            viz::ClientMessageFrame frame;
            if (viz::decode_client_message(body, len, frame)) {
                std::lock_guard<std::mutex> lock(handler_mutex_);
                if (client_message_handler_) {
                    client_message_handler_(frame.topic, frame.payload, frame.payload_len);
                }
            }
        }
        off = tmp + len;
    }
    read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + static_cast<long>(off));
    return true;
}

void VizSink::run() {
    while (!stop_.load()) {
        if (fd_ < 0) {
            const auto now = std::chrono::steady_clock::now();
            if (now < retry_at_) {
                wait_for_wake(static_cast<int>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(retry_at_ - now)
                        .count()));
                continue;
            }
            if (!try_connect()) {
                if (!unreachable_warned_) {
                    spdlog::warn(
                        "[VizSink] viz_relay not reachable at {}; live viz off until it is "
                        "(recording and control unaffected)",
                        socket_path_);
                    unreachable_warned_ = true;
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    clear_queues_locked();
                }
                retry_at_ = now + kReconnectInterval;
                continue;
            }
            unreachable_warned_ = false;
            settled_ = false;
            connected_at_ = now;
            {
                // Whatever was queued for the last connection is stale. The new one starts with
                // every advertise, so the relay knows each channel before its first message.
                std::lock_guard<std::mutex> lock(mutex_);
                clear_queues_locked();
                for (const auto& adv : advertises_) {
                    auto encoded = viz::encode_advertise(adv);
                    priority_bytes_ += encoded.size();
                    priority_queue_.push_back(std::move(encoded));
                }
            }
            connected_.store(true);
        }

        if (!send_pending()) {
            lose_connection(std::strerror(errno));
            continue;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!settled_ && now - connected_at_ >= kSettleTime) {
            settled_ = true;
            if (lost_warned_) {
                spdlog::info("[VizSink] Reconnected to viz_relay at {}", socket_path_);
                lost_warned_ = false;
            } else {
                spdlog::info("[VizSink] Connected to viz_relay at {}", socket_path_);
            }
        }
        const bool sending = out_sent_ < out_.size();
        if (sending && now - last_send_progress_ > kSendStallTimeout) {
            lose_connection("relay stopped reading");
            continue;
        }

        pollfd fds[2];
        fds[0] = {fd_, static_cast<short>(POLLIN | (sending ? POLLOUT : 0)), 0};
        fds[1] = {wake_fd_, POLLIN, 0};
        if (::poll(fds, 2, kPollMs) < 0 && errno != EINTR) {
            lose_connection(std::strerror(errno));
            continue;
        }
        if (fds[1].revents & POLLIN) {
            uint64_t count = 0;
            [[maybe_unused]] const ssize_t n = ::read(wake_fd_, &count, sizeof(count));
        }
        if ((fds[0].revents & (POLLIN | POLLHUP | POLLERR)) && !read_incoming()) {
            lose_connection("relay closed the connection");
            continue;
        }
    }
}

}  // namespace auto_battlebot
