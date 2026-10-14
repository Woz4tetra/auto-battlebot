#include "esp32_diagnostics/esp32_wifi_telemetry.hpp"

#include <arpa/inet.h>
#include <espnow_protocol.h>
#include <netinet/in.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>

#include "esp32_diagnostics/esp32_telemetry_batch.hpp"

namespace auto_battlebot {
namespace {
constexpr auto kPollSlice = std::chrono::milliseconds(100);
constexpr auto kRequestTimeout = std::chrono::milliseconds(1000);
/** A full reply is 32 + 256 * 118 bytes; anything much past that is not ours. */
constexpr size_t kMaxReplyBytes = 64 * 1024;
/** Ten seconds of records. The transmitter drains every app tick, so this only fills when the
 *  app stalls. */
constexpr size_t kMaxQueuedEvents = 1000;

uint64_t system_clock_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

class SocketGuard {
   public:
    explicit SocketGuard(int fd) : fd_(fd) {}
    ~SocketGuard() {
        if (fd_ >= 0) ::close(fd_);
    }
    SocketGuard(const SocketGuard &) = delete;
    SocketGuard &operator=(const SocketGuard &) = delete;
    int get() const { return fd_; }

   private:
    int fd_;
};

/** Waits for fd to become ready for events, in slices so a stop request is seen. */
bool wait_ready(int fd, short events, std::chrono::steady_clock::time_point deadline,
                const std::atomic<bool> &stop) {
    while (!stop.load() && std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{fd, events, 0};
        const int ready = ::poll(&pfd, 1, static_cast<int>(kPollSlice.count()));
        if (ready > 0) return true;
        if (ready < 0 && errno != EINTR) return false;
    }
    return false;
}
}  // namespace

Esp32WifiTelemetry::Esp32WifiTelemetry(Esp32WifiTelemetryOptions options, ClockNs clock_ns)
    : options_(std::move(options)),
      clock_ns_(clock_ns ? std::move(clock_ns) : ClockNs(system_clock_ns)) {}

Esp32WifiTelemetry::~Esp32WifiTelemetry() { stop(); }

void Esp32WifiTelemetry::start() {
    if (thread_.joinable()) return;
    stop_.store(false);
    thread_ = std::thread([this] { run(); });
}

void Esp32WifiTelemetry::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_.store(true);
    }
    stop_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

std::vector<Esp32DiagnosticsEvent> Esp32WifiTelemetry::drain() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Esp32DiagnosticsEvent> events(std::make_move_iterator(queue_.begin()),
                                              std::make_move_iterator(queue_.end()));
    queue_.clear();
    return events;
}

Esp32WifiTelemetryStats Esp32WifiTelemetry::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void Esp32WifiTelemetry::wait_for(std::chrono::milliseconds duration) {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_cv_.wait_for(lock, duration, [this] { return stop_.load(); });
}

void Esp32WifiTelemetry::run() {
    while (!stop_.load()) {
        bool more = false;
        const bool ok = poll_once(more);
        if (stop_.load()) break;
        if (!ok) {
            wait_for(options_.retry_period);
        } else if (!more) {
            wait_for(options_.poll_period);
        }
    }
}

bool Esp32WifiTelemetry::poll_once(bool &more) {
    using espnow_protocol::TELEMETRY_BATCH_MAX;
    const bool synced = boot_id_.has_value();
    const std::string path = synced ? "/telemetry?from=" + std::to_string(next_seq_) +
                                          "&max=" + std::to_string(TELEMETRY_BATCH_MAX)
                                    : std::string("/telemetry");
    const std::optional<std::string> body = http_get(path);
    const uint64_t now_ns = clock_ns_();

    std::lock_guard<std::mutex> lock(mutex_);
    stats_.polls++;
    if (!body) {
        if (stats_.connected) {
            spdlog::warn("Esp32WifiTelemetry: lost {}:{}", options_.host, options_.port);
        }
        stats_.connected = false;
        stats_.poll_failures++;
        return false;
    }
    const std::optional<Esp32TelemetryBatch> batch = decode_esp32_telemetry_batch(*body);
    if (!batch) {
        stats_.connected = false;
        stats_.parse_errors++;
        return false;
    }
    if (!stats_.connected) {
        spdlog::info("Esp32WifiTelemetry: polling {}:{}, ring holds {} records", options_.host,
                     options_.port, batch->header.capacity);
    }
    stats_.connected = true;
    stats_.ring_capacity = batch->header.capacity;
    const auto &h = batch->header;

    // New client, or the robot rebooted and its seqs restarted: start from the head, not from
    // history nobody was recording tags for. The records in this reply, if any, are dropped.
    if (!boot_id_ || *boot_id_ != h.boot_id || h.head < next_seq_) {
        if (boot_id_) stats_.reboots++;
        boot_id_ = h.boot_id;
        next_seq_ = h.head;
        stats_.lag_records = 0;
        more = false;
        return true;
    }

    if (h.first_seq > next_seq_) stats_.lost_records += h.first_seq - next_seq_;
    const bool caught_up = h.first_seq + h.count == h.head;
    for (uint32_t i = 0; i < h.count; i++) {
        const bool backfill = !(caught_up && i + 1 == h.count);
        if (queue_.size() >= kMaxQueuedEvents) {
            queue_.pop_front();
            stats_.dropped_events++;
        }
        queue_.push_back(to_esp32_diagnostics_event(batch->records[i], now_ns, backfill));
        stats_.events++;
        if (backfill) stats_.backfilled++;
    }
    next_seq_ = h.first_seq + h.count;
    stats_.lag_records = h.head - next_seq_;
    more = !caught_up && h.count > 0;
    return true;
}

std::optional<std::string> Esp32WifiTelemetry::http_get(const std::string &path) {
    const auto deadline = std::chrono::steady_clock::now() + kRequestTimeout;
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(options_.port));
    if (inet_pton(AF_INET, options_.host.c_str(), &addr.sin_addr) <= 0) {
        spdlog::error("Esp32WifiTelemetry: invalid host '{}'", options_.host);
        return std::nullopt;
    }
    SocketGuard fd(::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0));
    if (fd.get() < 0) return std::nullopt;
    if (::connect(fd.get(), reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        if (errno != EINPROGRESS || !wait_ready(fd.get(), POLLOUT, deadline, stop_)) {
            return std::nullopt;
        }
        int error = 0;
        socklen_t length = sizeof(error);
        if (::getsockopt(fd.get(), SOL_SOCKET, SO_ERROR, &error, &length) != 0 || error != 0) {
            return std::nullopt;
        }
    }

    const std::string request =
        "GET " + path + " HTTP/1.1\r\nHost: " + options_.host + "\r\nConnection: close\r\n\r\n";
    size_t sent = 0;
    while (sent < request.size()) {
        const ssize_t n = ::send(fd.get(), request.data() + sent, request.size() - sent,
                                 MSG_NOSIGNAL | MSG_DONTWAIT);
        if (n > 0) {
            sent += static_cast<size_t>(n);
        } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            if (!wait_ready(fd.get(), POLLOUT, deadline, stop_)) return std::nullopt;
        } else {
            return std::nullopt;
        }
    }

    // Read until the body is complete by Content-Length, or the server closes.
    std::string reply;
    std::optional<size_t> body_start;
    std::optional<size_t> content_length;
    char buffer[4096];
    while (true) {
        if (body_start && content_length && reply.size() >= *body_start + *content_length) break;
        if (!wait_ready(fd.get(), POLLIN, deadline, stop_)) return std::nullopt;
        const ssize_t n = ::recv(fd.get(), buffer, sizeof(buffer), 0);
        if (n == 0) break;
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            return std::nullopt;
        }
        reply.append(buffer, static_cast<size_t>(n));
        if (reply.size() > kMaxReplyBytes) return std::nullopt;
        if (!body_start) {
            const size_t end = reply.find("\r\n\r\n");
            if (end == std::string::npos) continue;
            body_start = end + 4;
            // "HTTP/1.1 200 OK"
            if (reply.rfind("HTTP/", 0) != 0 || reply.size() < 12 || reply[9] != '2') {
                return std::nullopt;
            }
            std::string head = reply.substr(0, end);
            for (char &c : head) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            const size_t key = head.find("\r\ncontent-length:");
            if (key != std::string::npos) {
                content_length = std::strtoul(head.c_str() + key + 17, nullptr, 10);
            }
        }
    }
    if (!body_start) return std::nullopt;
    std::string body = reply.substr(*body_start);
    if (content_length) {
        if (body.size() < *content_length) return std::nullopt;
        body.resize(*content_length);
    }
    return body;
}

}  // namespace auto_battlebot
