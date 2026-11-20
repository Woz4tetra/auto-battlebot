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
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <iterator>
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
    r.pack_volts = 15.5f;
    r.pack_amps_discharge = NAN;
    r.yaw_rate_cw_dps = 12.0f;
    r.yaw_rate_cmd_cw_dps = NAN;
    r.motor_rise_left_c = 31.5f;
    r.motor_speed_right_rad_s = NAN;
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
    h.next_seq = head;
    h.oldest_seq = head > capacity ? head - capacity : 0;
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
    EXPECT_EQ(batch->header.next_seq, 12u);
    ASSERT_EQ(batch->records.size(), 2u);
    EXPECT_EQ(batch->records[1].telemetry_seq, 11u);
    EXPECT_EQ(batch->records[1].timestamp_ms, 1110u);
}

constexpr size_t kMagicOffset = offsetof(TelemetryBatchHeader, magic);
constexpr size_t kVersionOffset = offsetof(TelemetryBatchHeader, version);
constexpr size_t kRecordSizeOffset = offsetof(TelemetryBatchHeader, record_size);

TEST(Esp32TelemetryBatchTest, RejectsWrongMagicVersionSizeOrLength) {
    std::string body = batch_body(7, 100, 12, 10, 2);
    EXPECT_FALSE(decode_esp32_telemetry_batch(body.substr(0, body.size() - 1)).has_value());
    EXPECT_FALSE(decode_esp32_telemetry_batch(body + "x").has_value());
    EXPECT_FALSE(decode_esp32_telemetry_batch(body.substr(0, 10)).has_value());
    std::string bad = body;
    bad[kMagicOffset] ^= 1;
    EXPECT_FALSE(decode_esp32_telemetry_batch(bad).has_value());
    bad = body;
    bad[kVersionOffset] ^= 1;
    EXPECT_FALSE(decode_esp32_telemetry_batch(bad).has_value());
    bad = body;
    bad[kRecordSizeOffset] ^= 1;
    EXPECT_FALSE(decode_esp32_telemetry_batch(bad).has_value());
}

TEST(Esp32TelemetryBatchTest, RecordBecomesDiagnosticsEvent) {
    const Esp32DiagnosticsEvent e = to_esp32_diagnostics_event(record(42), 99, true);
    EXPECT_EQ(e.host_receive_wall_ns, 99u);
    EXPECT_EQ(e.robot_millis, 1420u);
    EXPECT_TRUE(e.armed);
    EXPECT_DOUBLE_EQ(e.a_percent, 42.0);
    ASSERT_TRUE(e.vbat.has_value());
    EXPECT_FALSE(e.ibat.has_value());
    EXPECT_FALSE(e.yaw_rate_cmd_cw_deg_s.has_value());
    EXPECT_EQ(e.seq, 42u);
    EXPECT_EQ(e.wifi_clients, 0);
    EXPECT_EQ(e.backfill, true);
    ASSERT_TRUE(e.guard.has_value());
    EXPECT_DOUBLE_EQ(e.guard->motor_rise_left_c, 31.5);
    EXPECT_TRUE(std::isnan(e.guard->motor_speed_right_rad_s));
    EXPECT_DOUBLE_EQ(e.guard->guard_scale, 0.5);
    EXPECT_EQ(e.guard->guard_state, MotorGuardState::LIMITING);
    EXPECT_FALSE(e.guard->left_stalled);
    EXPECT_TRUE(e.guard->right_stalled);
    EXPECT_EQ(e.guard->erpm_crc_errors_since_boot, 517u);
    const std::string json = to_esp32_diagnostics_json(e);
    EXPECT_NE(json.find("\"seq\":42"), std::string::npos) << json;
    EXPECT_NE(json.find("\"backfill\":true"), std::string::npos) << json;
    EXPECT_NE(json.find("\"ibat\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"motor_speed_right\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"erpm_crc_errors\":517"), std::string::npos) << json;
}

void send_reply_in_two_halves(int fd, const std::string &reply) {
    const size_t half = reply.size() / 2;
    ::send(fd, reply.data(), half, MSG_NOSIGNAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    ::send(fd, reply.data() + half, reply.size() - half, MSG_NOSIGNAL);
}

constexpr uint32_t kRingCapacity = 50;
constexpr uint32_t kBatchMax = 20;

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

    uint32_t capacity = kRingCapacity;
    uint32_t batch_max = kBatchMax;

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
        const std::string reply =
            "HTTP/1.1 200 OK\r\nContent-Type: application/octet-stream\r\n"
            "Content-Length: " +
            std::to_string(body.size()) + "\r\n\r\n" + body;
        send_reply_in_two_halves(fd, reply);
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

std::vector<Esp32DiagnosticsEvent> collect_at_least(Esp32WifiTelemetry &telemetry, size_t n) {
    std::vector<Esp32DiagnosticsEvent> events;
    wait_until(
        [&] {
            for (auto &e : telemetry.drain()) events.push_back(e);
            return events.size() >= n;
        },
        std::chrono::seconds(5));
    return events;
}

TEST(Esp32WifiTelemetryTest, StartingTwiceRunsOneWorker) {
    FakeRingServer server;
    constexpr uint32_t kHead = 30;
    constexpr uint32_t kNewRecords = 10;
    server.set(1, kHead);
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    telemetry.start();
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));

    server.set(1, kHead + kNewRecords);
    auto events = collect_at_least(telemetry, kNewRecords);
    const bool got_duplicates = wait_until(
        [&] {
            for (auto &e : telemetry.drain()) events.push_back(e);
            return events.size() > kNewRecords;
        },
        std::chrono::milliseconds(500));
    telemetry.stop();
    EXPECT_FALSE(got_duplicates) << "a second worker delivers every record twice";
    ASSERT_EQ(events.size(), kNewRecords);
    for (size_t i = 0; i < events.size(); i++) EXPECT_EQ(events[i].seq, kHead + i) << i;
}

TEST(Esp32WifiTelemetryTest, StartsAtHeadThenReadsEverythingNew) {
    FakeRingServer server;
    constexpr uint32_t kHistoryHeadNotReplayed = 30;
    constexpr uint32_t kNewRecordsSpanningTwoBatches = 2 * kBatchMax + 5;
    server.set(1, kHistoryHeadNotReplayed);
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));
    EXPECT_EQ(server.paths().front(), "/telemetry");

    server.set(1, kHistoryHeadNotReplayed + kNewRecordsSpanningTwoBatches);
    const auto events = collect_at_least(telemetry, kNewRecordsSpanningTwoBatches);
    ASSERT_EQ(events.size(), kNewRecordsSpanningTwoBatches);
    for (size_t i = 0; i < events.size(); i++)
        EXPECT_EQ(events[i].seq, kHistoryHeadNotReplayed + i);

    EXPECT_EQ(events.back().backfill, false);
    for (size_t i = 0; i + 1 < events.size(); i++) EXPECT_EQ(events[i].backfill, true) << i;
    for (size_t i = 1; i < kBatchMax; i++)
        EXPECT_EQ(events[i].host_receive_wall_ns, events[0].host_receive_wall_ns)
            << "rows of one poll share its arrival stamp, " << i;
    const Esp32WifiTelemetryStats stats = telemetry.stats();
    EXPECT_EQ(stats.ring_overwritten_records, 0u);
    EXPECT_EQ(stats.lag_records, 0u);
    EXPECT_EQ(stats.ring_capacity, kRingCapacity);
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
    constexpr uint32_t kRecordedDuringOutage = 40;
    server.set(1, kRecordedDuringOutage);
    server.set_offline(false);

    const auto events = collect_at_least(telemetry, kRecordedDuringOutage);
    ASSERT_EQ(events.size(), kRecordedDuringOutage);
    for (size_t i = 0; i < events.size(); i++) EXPECT_EQ(events[i].seq, i);
    EXPECT_EQ(telemetry.stats().ring_overwritten_records, 0u);
    EXPECT_GE(telemetry.stats().poll_failures, 1u);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, FullBatchIsFollowedByAnImmediateRequest) {
    FakeRingServer server;
    server.set(1, 0);
    Esp32WifiTelemetryOptions options = options_for(server);
    options.poll_period = std::chrono::milliseconds(600);
    Esp32WifiTelemetry telemetry(options);
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));

    const auto start = std::chrono::steady_clock::now();
    server.set(1, 50);
    const auto events = collect_at_least(telemetry, 50);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    ASSERT_EQ(events.size(), 50u);
    EXPECT_LT(elapsed, std::chrono::milliseconds(1000))
        << "3 batches should arrive within one poll period, not one per period";
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
    constexpr uint32_t kOverwrittenRecords = 30;
    constexpr uint32_t kHeadAfterOutage = kRingCapacity + kOverwrittenRecords;
    server.set(1, kHeadAfterOutage);
    server.set_offline(false);

    const auto events = collect_at_least(telemetry, kRingCapacity);
    ASSERT_EQ(events.size(), kRingCapacity);
    EXPECT_EQ(events.front().seq, kOverwrittenRecords);
    EXPECT_EQ(events.back().seq, kHeadAfterOutage - 1);
    EXPECT_EQ(telemetry.stats().ring_overwritten_records, kOverwrittenRecords);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, RebootResyncsToTheNewHead) {
    FakeRingServer server;
    server.set(1, 10);
    Esp32WifiTelemetry telemetry(options_for(server));
    telemetry.start();
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().connected; }, std::chrono::seconds(5)));
    server.set(1, 15);
    ASSERT_EQ(collect_at_least(telemetry, 5).size(), 5u);

    constexpr uint32_t kRebootedBootId = 2;
    constexpr uint32_t kSeqAfterReboot = 3;
    server.set(kRebootedBootId, kSeqAfterReboot);
    ASSERT_TRUE(wait_until([&] { return telemetry.stats().boot_id_changes == 1; },
                           std::chrono::seconds(5)));
    server.set(kRebootedBootId, kSeqAfterReboot + 5);
    const auto events = collect_at_least(telemetry, 5);
    ASSERT_EQ(events.size(), 5u);
    EXPECT_EQ(events.front().seq, kSeqAfterReboot);
    EXPECT_EQ(telemetry.stats().ring_overwritten_records, 0u);
    telemetry.stop();
}

TEST(Esp32WifiTelemetryTest, StopIsPromptWithNoRobot) {
    Esp32WifiTelemetryOptions options;
    options.host = "127.0.0.1";
    constexpr int kNoListenerPort = 1;
    options.port = kNoListenerPort;
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
