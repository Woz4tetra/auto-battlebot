#include <arpa/inet.h>
#include <espnow_protocol.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "esp32_diagnostics/esp32_telemetry_batch.hpp"
#include "esp32_diagnostics/esp32_wifi_telemetry.hpp"

namespace auto_battlebot {
namespace {

using espnow_protocol::TelemetryBatchHeader;
using espnow_protocol::TelemetryPayload;

TelemetryPayload record(uint32_t seq) {
    TelemetryPayload r = {};
    r.telemetry_seq = seq;
    r.timestamp_ms = 1000 + 10 * seq;
    r.a_percent = static_cast<float>(seq);
    r.armed = 1;
    r.flip_switch = 0;
    r.vbat = 15.5f;
    r.ibat = NAN;
    r.yaw_rate = 12.0f;
    r.yaw_rate_cmd = NAN;
    r.motor_rise_left = 31.5f;
    r.motor_speed_right = NAN;
    r.guard_scale = 0.5f;
    r.guard_state = 3;
    r.stall_flags = 2;
    r.erpm_crc_errors = 517;
    return r;
}

std::string batch_body(uint32_t boot_id, uint32_t capacity, uint32_t head, uint32_t first,
                       uint32_t count) {
    TelemetryBatchHeader h = {};
    h.magic = espnow_protocol::TELEMETRY_BATCH_MAGIC;
    h.version = espnow_protocol::TELEMETRY_BATCH_VERSION;
    h.record_size = sizeof(TelemetryPayload);
    h.boot_id = boot_id;
    h.capacity = capacity;
    h.head = head;
    h.oldest = head > capacity ? head - capacity : 0;
    h.first_seq = first;
    h.count = count;
    std::string body(reinterpret_cast<const char *>(&h), sizeof(h));
    for (uint32_t i = 0; i < count; i++) {
        const TelemetryPayload r = record(first + i);
        body.append(reinterpret_cast<const char *>(&r), sizeof(r));
    }
    return body;
}

TEST(Esp32TelemetryBatchTest, DecodesHeaderAndRecords) {
    const auto batch = decode_esp32_telemetry_batch(batch_body(7, 100, 12, 10, 2));
    ASSERT_TRUE(batch.has_value());
    EXPECT_EQ(batch->header.boot_id, 7u);
    EXPECT_EQ(batch->header.head, 12u);
    ASSERT_EQ(batch->records.size(), 2u);
    EXPECT_EQ(batch->records[1].telemetry_seq, 11u);
    EXPECT_EQ(batch->records[1].timestamp_ms, 1110u);
}

TEST(Esp32TelemetryBatchTest, RejectsWrongMagicVersionSizeOrLength) {
    std::string body = batch_body(7, 100, 12, 10, 2);
    EXPECT_FALSE(decode_esp32_telemetry_batch(body.substr(0, body.size() - 1)).has_value());
    EXPECT_FALSE(decode_esp32_telemetry_batch(body + "x").has_value());
    EXPECT_FALSE(decode_esp32_telemetry_batch(body.substr(0, 10)).has_value());
    std::string bad = body;
    bad[0] ^= 1;  // magic
    EXPECT_FALSE(decode_esp32_telemetry_batch(bad).has_value());
    bad = body;
    bad[4] ^= 1;  // version
    EXPECT_FALSE(decode_esp32_telemetry_batch(bad).has_value());
    bad = body;
    bad[6] ^= 1;  // record_size
    EXPECT_FALSE(decode_esp32_telemetry_batch(bad).has_value());
}

TEST(Esp32TelemetryBatchTest, RecordBecomesDiagnosticsEvent) {
    const Esp32DiagnosticsEvent e = to_esp32_diagnostics_event(record(42), 99, true);
    EXPECT_EQ(e.host_receive_ns, 99u);
    EXPECT_EQ(e.timestamp_ms, 1420u);
    EXPECT_TRUE(e.armed);
    EXPECT_DOUBLE_EQ(e.a_percent, 42.0);
    ASSERT_TRUE(e.vbat.has_value());
    EXPECT_FALSE(e.ibat.has_value());
    EXPECT_FALSE(e.yaw_rate_cmd.has_value());
    EXPECT_EQ(e.seq, 42u);
    EXPECT_EQ(e.backfill, true);
    ASSERT_TRUE(e.guard.has_value());
    EXPECT_DOUBLE_EQ(e.guard->motor_rise_left, 31.5);
    EXPECT_TRUE(std::isnan(e.guard->motor_speed_right));
    EXPECT_DOUBLE_EQ(e.guard->guard_scale, 0.5);
    EXPECT_EQ(e.guard->guard_state, 3);
    EXPECT_EQ(e.guard->stall_flags, 2);
    EXPECT_EQ(e.guard->erpm_crc_errors, 517u);
    const std::string json = to_esp32_diagnostics_json(e);
    EXPECT_NE(json.find("\"seq\":42"), std::string::npos) << json;
    EXPECT_NE(json.find("\"backfill\":true"), std::string::npos) << json;
    EXPECT_NE(json.find("\"ibat\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"motor_speed_right\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"erpm_crc_errors\":517"), std::string::npos) << json;
}

/**
 * A stand-in for the firmware's GET /telemetry: a ring of `capacity` records whose head the test
 * moves, answering the way handle_telemetry in diagnostics_server.cpp does. While `offline` is
 * set it accepts and closes without replying, like a robot out of WiFi range.
 */
class FakeRingServer {
   public:
    FakeRingServer() {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int reuse = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
        socklen_t length = sizeof(addr);
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &length);
        port_ = ntohs(addr.sin_port);
        ::listen(listen_fd_, 16);
        thread_ = std::thread([this] { serve(); });
    }
    ~FakeRingServer() {
        stop_.store(true);
        thread_.join();
        ::close(listen_fd_);
    }

    int port() const { return port_; }
    void set(uint32_t boot_id, uint32_t head) {
        std::lock_guard<std::mutex> lock(mutex_);
        boot_id_ = boot_id;
        head_ = head;
    }
    void set_offline(bool offline) { offline_.store(offline); }
    std::vector<std::string> paths() {
        std::lock_guard<std::mutex> lock(mutex_);
        return paths_;
    }

    uint32_t capacity = 50;
    uint32_t batch_max = 20;

   private:
    void serve() {
        while (!stop_.load()) {
            pollfd pfd{listen_fd_, POLLIN, 0};
            if (::poll(&pfd, 1, 50) <= 0) continue;
            const int fd = ::accept(listen_fd_, nullptr, nullptr);
            if (fd < 0) continue;
            std::string request;
            char buffer[1024];
            while (request.find("\r\n\r\n") == std::string::npos) {
                const ssize_t n = ::recv(fd, buffer, sizeof(buffer), 0);
                if (n <= 0) break;
                request.append(buffer, static_cast<size_t>(n));
            }
            if (!offline_.load()) respond(fd, request);
            ::close(fd);
        }
    }

    void respond(int fd, const std::string &request) {
        const size_t start = request.find(' ') + 1;
        const std::string path = request.substr(start, request.find(' ', start) - start);
        uint32_t boot_id, head;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            paths_.push_back(path);
            boot_id = boot_id_;
            head = head_;
        }
        const uint32_t oldest = head > capacity ? head - capacity : 0;
        uint32_t first = head, count = 0;
        const size_t from = path.find("from=");
        if (from != std::string::npos) {
            const uint32_t want = static_cast<uint32_t>(std::stoul(path.substr(from + 5)));
            first = std::min(std::max(want, oldest), head);
            count = std::min(head - first, batch_max);
        }
        const std::string body = batch_body(boot_id, capacity, head, first, count);
        // Split the reply so the client has to reassemble it.
        const std::string reply =
            "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\n"
            "Content-Length: " +
            std::to_string(body.size()) + "\r\n\r\n" + body;
        const size_t half = reply.size() / 2;
        ::send(fd, reply.data(), half, MSG_NOSIGNAL);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        ::send(fd, reply.data() + half, reply.size() - half, MSG_NOSIGNAL);
    }

    int listen_fd_ = -1;
    int port_ = 0;
    std::atomic<bool> stop_{false};
    std::atomic<bool> offline_{false};
    std::mutex mutex_;
    uint32_t boot_id_ = 1;
    uint32_t head_ = 0;
    std::vector<std::string> paths_;
    std::thread thread_;
};

template <typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return predicate();
}

Esp32WifiTelemetryOptions options_for(const FakeRingServer &server) {
    Esp32WifiTelemetryOptions options;
    options.host = "127.0.0.1";
    options.port = server.port();
    options.poll_period = std::chrono::milliseconds(10);
    options.retry_period = std::chrono::milliseconds(20);
    return options;
}

// Collects events until `n` have arrived, checking seqs run without gaps.
std::vector<Esp32DiagnosticsEvent> collect(Esp32WifiTelemetry &telemetry, size_t n) {
    std::vector<Esp32DiagnosticsEvent> events;
    wait_until(
        [&] {
            for (auto &e : telemetry.drain()) events.push_back(e);
            return events.size() >= n;
        },
        std::chrono::seconds(5));
    return events;
}

TEST(Esp32WifiTelemetryTest, StartsAtHeadThenReadsEverythingNew) {
    FakeRingServer server;
    server.set(1, 30);  // history the client must not replay
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));
    EXPECT_EQ(server.paths().front(), "/telemetry");

    server.set(1, 75);  // 45 new records, more than one 20-record batch
    const auto events = collect(telemetry, 45);
    ASSERT_EQ(events.size(), 45u);
    for (size_t i = 0; i < events.size(); i++) EXPECT_EQ(events[i].seq, 30 + i);

    // Only the newest row of the poll that caught up is a clock sample.
    EXPECT_EQ(events.back().backfill, false);
    for (size_t i = 0; i + 1 < events.size(); i++) EXPECT_EQ(events[i].backfill, true) << i;
    const Esp32WifiTelemetryStats stats = telemetry.stats();
    EXPECT_EQ(stats.lost_records, 0u);
    EXPECT_EQ(stats.lag_records, 0u);
    EXPECT_EQ(stats.ring_capacity, 50u);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, CatchesUpAfterAnOutageWithoutLoss) {
    FakeRingServer server;
    server.set(1, 0);
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));

    server.set_offline(true);
    ASSERT_TRUE(wait_until([&] { return !telemetry.stats().connected; }, std::chrono::seconds(5)));
    server.set(1, 40);  // the robot kept recording while out of range
    server.set_offline(false);

    const auto events = collect(telemetry, 40);
    ASSERT_EQ(events.size(), 40u);
    for (size_t i = 0; i < events.size(); i++) EXPECT_EQ(events[i].seq, i);
    EXPECT_EQ(telemetry.stats().lost_records, 0u);
    EXPECT_GE(telemetry.stats().poll_failures, 1u);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, CountsRecordsTheRingOverwrote) {
    FakeRingServer server;
    server.set(1, 0);
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));

    server.set_offline(true);
    ASSERT_TRUE(wait_until([&] { return !telemetry.stats().connected; }, std::chrono::seconds(5)));
    server.set(1, 80);  // outage outlasted the 50-record ring: seqs 0..29 are gone
    server.set_offline(false);

    const auto events = collect(telemetry, 50);
    ASSERT_EQ(events.size(), 50u);
    EXPECT_EQ(events.front().seq, 30u);
    EXPECT_EQ(events.back().seq, 79u);
    EXPECT_EQ(telemetry.stats().lost_records, 30u);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, RebootResyncsToTheNewHead) {
    FakeRingServer server;
    server.set(1, 10);
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));
    server.set(1, 15);
    ASSERT_EQ(collect(telemetry, 5).size(), 5u);

    server.set(2, 3);  // rebooted: new boot id, seqs restarted
    ASSERT_TRUE(
        wait_until([&] { return telemetry.stats().reboots == 1; }, std::chrono::seconds(5)));
    server.set(2, 8);
    const auto events = collect(telemetry, 5);
    ASSERT_EQ(events.size(), 5u);
    EXPECT_EQ(events.front().seq, 3u);
    EXPECT_EQ(telemetry.stats().lost_records, 0u);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, StopIsPromptWithNoRobot) {
    Esp32WifiTelemetryOptions options;
    options.host = "127.0.0.1";
    options.port = 1;  // nothing listens
    Esp32WifiTelemetry telemetry(options);
    telemetry.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    const auto t0 = std::chrono::steady_clock::now();
    telemetry.stop();
    EXPECT_LT(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(1500));
    EXPECT_FALSE(telemetry.stats().connected);
}

}  // namespace
}  // namespace auto_battlebot
