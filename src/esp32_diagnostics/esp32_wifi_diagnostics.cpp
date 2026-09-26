#include "esp32_diagnostics/esp32_wifi_diagnostics.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>

namespace auto_battlebot {
namespace {
constexpr auto kPollSlice = std::chrono::milliseconds(100);
constexpr auto kMaxReconnectPeriod = std::chrono::milliseconds(10000);
constexpr auto kRequestTimeout = std::chrono::milliseconds(1000);
/** Shutdown waits on this, so it stays short. A robot that is gone just misses the stop. */
constexpr auto kRecordStopTimeout = std::chrono::milliseconds(500);
/** Bigger than any header block the firmware sends; past it the reply is not ours. */
constexpr size_t kMaxHeaderBytes = 8192;
/** A line longer than this is garbage, not a diagnostics event (the firmware buffer is 320). */
constexpr size_t kMaxLineBytes = 4096;

uint64_t system_clock_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count());
}

/** Status code from an HTTP status line, or -1. */
int parse_status_code(const std::string &head) {
    // "HTTP/1.1 200 OK"
    if (head.rfind("HTTP/", 0) != 0) return -1;
    const size_t space = head.find(' ');
    if (space == std::string::npos || space + 4 > head.size()) return -1;
    int code = 0;
    for (size_t i = space + 1; i < space + 4; ++i) {
        if (head[i] < '0' || head[i] > '9') return -1;
        code = code * 10 + (head[i] - '0');
    }
    return code;
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
}  // namespace

Esp32WifiDiagnostics::Esp32WifiDiagnostics(Esp32WifiDiagnosticsOptions options, ClockNs clock_ns)
    : options_(std::move(options)),
      clock_ns_(clock_ns ? std::move(clock_ns) : ClockNs(system_clock_ns)) {}

Esp32WifiDiagnostics::~Esp32WifiDiagnostics() { stop(); }

void Esp32WifiDiagnostics::start() {
    if (thread_.joinable()) return;
    stop_.store(false);
    finishing_ = false;
    thread_ = std::thread(&Esp32WifiDiagnostics::run, this);
}

void Esp32WifiDiagnostics::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_.store(true);
    }
    stop_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

std::vector<Esp32DiagnosticsEvent> Esp32WifiDiagnostics::drain() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Esp32DiagnosticsEvent> events(std::make_move_iterator(queue_.begin()),
                                              std::make_move_iterator(queue_.end()));
    queue_.clear();
    return events;
}

Esp32WifiDiagnosticsStats Esp32WifiDiagnostics::stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void Esp32WifiDiagnostics::wait_for(std::chrono::milliseconds duration) {
    std::unique_lock<std::mutex> lock(mutex_);
    stop_cv_.wait_for(lock, duration, [this] { return stop_.load(); });
}

void Esp32WifiDiagnostics::run() {
    auto backoff = options_.reconnect_period;
    bool ever_streamed = false;
    while (!stop_.load()) {
        const bool streamed = run_stream();
        ever_streamed = ever_streamed || streamed;
        if (stop_.load()) break;
        if (streamed) {
            backoff = options_.reconnect_period;
        } else {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.connect_failures++;
        }
        wait_for(backoff);
        backoff = std::min(backoff * 2, std::max(kMaxReconnectPeriod, options_.reconnect_period));
    }
    if (options_.record_mode && ever_streamed) {
        // Put the firmware back on its 10 Hz rate so the next client does not flood the link.
        // The stop request itself is what is being honored here, so it must not abort on it.
        finishing_ = true;
        const bool stopped = simple_get("/record/stop", kRecordStopTimeout);
        if (!stopped) spdlog::warn("Esp32WifiDiagnostics: /record/stop did not reach the robot");
    }
}

int Esp32WifiDiagnostics::connect_socket(std::chrono::milliseconds timeout) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(options_.port));
    if (inet_pton(AF_INET, options_.host.c_str(), &addr.sin_addr) <= 0) {
        spdlog::error("Esp32WifiDiagnostics: invalid host '{}'", options_.host);
        return -1;
    }
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;
    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) return fd;
    if (errno != EINPROGRESS) {
        ::close(fd);
        return -1;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!stopping() && std::chrono::steady_clock::now() < deadline) {
        pollfd pfd{fd, POLLOUT, 0};
        const int ready = ::poll(&pfd, 1, static_cast<int>(kPollSlice.count()));
        if (ready < 0 && errno != EINTR) break;
        if (ready <= 0) continue;
        int error = 0;
        socklen_t length = sizeof(error);
        if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &length) != 0 || error != 0) break;
        int flag = 1;
        ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        return fd;
    }
    ::close(fd);
    return -1;
}

bool Esp32WifiDiagnostics::send_request(int fd, const std::string &path, bool event_stream) {
    std::string request = "GET " + path + " HTTP/1.1\r\nHost: " + options_.host + "\r\n";
    if (event_stream) {
        request += "Accept: text/event-stream\r\nCache-Control: no-cache\r\n";
    } else {
        request += "Connection: close\r\n";
    }
    request += "\r\n";
    size_t sent = 0;
    const auto deadline = std::chrono::steady_clock::now() + kRequestTimeout;
    while (sent < request.size()) {
        if (stopping() || std::chrono::steady_clock::now() >= deadline) return false;
        const ssize_t n =
            ::send(fd, request.data() + sent, request.size() - sent, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            pollfd pfd{fd, POLLOUT, 0};
            ::poll(&pfd, 1, static_cast<int>(kPollSlice.count()));
            continue;
        }
        return false;
    }
    return true;
}

bool Esp32WifiDiagnostics::simple_get(const std::string &path, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    SocketGuard fd(connect_socket(timeout));
    if (fd.get() < 0 || !send_request(fd.get(), path, false)) return false;
    std::string reply;
    char buffer[512];
    while (!stopping() && std::chrono::steady_clock::now() < deadline) {
        // Only the status line matters; the firmware answers "ok".
        if (reply.find("\r\n") != std::string::npos) break;
        pollfd pfd{fd.get(), POLLIN, 0};
        const int ready = ::poll(&pfd, 1, static_cast<int>(kPollSlice.count()));
        if (ready < 0 && errno != EINTR) return false;
        if (ready <= 0) continue;
        const ssize_t n = ::recv(fd.get(), buffer, sizeof(buffer), 0);
        if (n <= 0) break;
        reply.append(buffer, static_cast<size_t>(n));
        if (reply.size() > kMaxHeaderBytes) break;
    }
    const int code = parse_status_code(reply);
    return code >= 200 && code < 300;
}

bool Esp32WifiDiagnostics::run_stream() {
    if (options_.record_mode && !simple_get("/record/start", kRequestTimeout)) return false;

    SocketGuard fd(connect_socket(kRequestTimeout));
    if (fd.get() < 0 || !send_request(fd.get(), "/events", true)) return false;

    std::string buffer;
    bool headers_done = false;
    std::string event_name;
    std::string data;
    auto last_bytes = std::chrono::steady_clock::now();
    char chunk[2048];

    while (!stop_.load()) {
        pollfd pfd{fd.get(), POLLIN, 0};
        const int ready = ::poll(&pfd, 1, static_cast<int>(kPollSlice.count()));
        const auto now = std::chrono::steady_clock::now();
        if (ready < 0 && errno != EINTR) break;
        if (ready <= 0) {
            const auto idle_limit = headers_done ? options_.stream_idle_timeout : kRequestTimeout;
            if (now - last_bytes > idle_limit) {
                spdlog::warn(
                    "Esp32WifiDiagnostics: no data from {} for {} ms, reconnecting", options_.host,
                    std::chrono::duration_cast<std::chrono::milliseconds>(idle_limit).count());
                break;
            }
            continue;
        }
        const ssize_t n = ::recv(fd.get(), chunk, sizeof(chunk), 0);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) continue;
        if (n <= 0) break;
        last_bytes = now;
        buffer.append(chunk, static_cast<size_t>(n));

        if (!headers_done) {
            const size_t end = buffer.find("\r\n\r\n");
            if (end == std::string::npos) {
                if (buffer.size() > kMaxHeaderBytes) break;
                continue;
            }
            const int code = parse_status_code(buffer);
            if (code != 200) {
                spdlog::warn("Esp32WifiDiagnostics: /events answered {}", code);
                break;
            }
            buffer.erase(0, end + 4);
            headers_done = true;
            std::lock_guard<std::mutex> lock(mutex_);
            if (streamed_before_) stats_.reconnects++;
            streamed_before_ = true;
            stats_.connected = true;
            spdlog::info("Esp32WifiDiagnostics: streaming from {}:{}{}", options_.host,
                         options_.port, options_.record_mode ? " in record mode" : "");
        }

        size_t start = 0;
        while (true) {
            const size_t newline = buffer.find('\n', start);
            if (newline == std::string::npos) break;
            handle_sse_line(std::string_view(buffer).substr(start, newline - start), event_name,
                            data);
            start = newline + 1;
        }
        buffer.erase(0, start);
        if (buffer.size() > kMaxLineBytes) {
            std::lock_guard<std::mutex> lock(mutex_);
            stats_.parse_errors++;
            buffer.clear();
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    stats_.connected = false;
    return headers_done;
}

void Esp32WifiDiagnostics::handle_sse_line(std::string_view line, std::string &event_name,
                                           std::string &data) {
    if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
    if (line.empty()) {
        // A blank line ends the event.
        dispatch_event(event_name, data);
        event_name.clear();
        data.clear();
        return;
    }
    if (line.front() == ':') return;  // comment
    const size_t colon = line.find(':');
    const std::string_view field = line.substr(0, colon);
    std::string_view value =
        colon == std::string_view::npos ? std::string_view{} : line.substr(colon + 1);
    if (!value.empty() && value.front() == ' ') value.remove_prefix(1);
    if (field == "data") {
        if (!data.empty()) data += '\n';
        data.append(value);
    } else if (field == "event") {
        event_name.assign(value);
    }
    // id: and retry: carry nothing the parser needs.
}

void Esp32WifiDiagnostics::dispatch_event(const std::string &event_name, const std::string &data) {
    if (data.empty()) return;
    // Named events are not diagnostics rows. The firmware's onConnect hello is an unnamed event
    // whose data is the word "connected".
    if (!event_name.empty() && event_name != "message") return;
    if (data == "connected") return;

    const uint64_t receive_ns = clock_ns_();
    std::string_view rest(data);
    while (!rest.empty()) {
        const size_t newline = rest.find('\n');
        const std::string_view line = rest.substr(0, newline);
        rest = newline == std::string_view::npos ? std::string_view{} : rest.substr(newline + 1);
        auto event = parse_esp32_diagnostics_csv(line, receive_ns);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!event) {
            stats_.parse_errors++;
            continue;
        }
        stats_.events++;
        if (queue_.size() >= options_.max_queued_events) {
            queue_.pop_front();
            stats_.dropped_events++;
        }
        queue_.push_back(*event);
    }
}

}  // namespace auto_battlebot
