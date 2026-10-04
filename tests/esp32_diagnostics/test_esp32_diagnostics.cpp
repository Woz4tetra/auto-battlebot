#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "esp32_diagnostics/esp32_diagnostics_event.hpp"
#include "esp32_diagnostics/esp32_wifi_diagnostics.hpp"

namespace auto_battlebot {
namespace {

// Lines in the firmware's printf format (firmware/mr_stabs_mk2/src/diagnostics_server.cpp).
constexpr const char *kLine20 =
    "123456,1,1,-12.5,3.0,0,1,-15.5,-9.5,0.1,-0.2,9.8,0,812,1,359.9,-1.5,2.3,4.0,-0.25";
constexpr const char *kLine21 =
    "123457,1,0,50.0,0.0,1,0,48.0,52.0,1.1,0.0,9.7,1,905,2,10.0,0.5,-0.5,0.0,0.00,15.842";
constexpr const char *kLine22 =
    "123459,1,1,80.0,0.0,0,0,78.0,82.0,0.5,0.0,9.8,0,910,1,12.0,0.5,-0.5,0.0,0.00,15.201,"
    "-42.37";
constexpr const char *kLine21Nan =
    "123458,0,0,0.0,0.0,0,0,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00,nan";

TEST(Esp32DiagnosticsParseTest, ParsesTwentyFieldsWithoutVbat) {
    auto event = parse_esp32_diagnostics_csv(kLine20, 42);
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->host_receive_ns, 42u);
    EXPECT_EQ(event->timestamp_ms, 123456u);
    EXPECT_TRUE(event->radio_connected);
    EXPECT_TRUE(event->armed);
    EXPECT_DOUBLE_EQ(event->a_percent, -12.5);
    EXPECT_DOUBLE_EQ(event->b_percent, 3.0);
    EXPECT_FALSE(event->button_state);
    EXPECT_EQ(event->flip_switch, 1);
    EXPECT_DOUBLE_EQ(event->left_cmd, -15.5);
    EXPECT_DOUBLE_EQ(event->right_cmd, -9.5);
    EXPECT_DOUBLE_EQ(event->accel_x, 0.1);
    EXPECT_DOUBLE_EQ(event->accel_y, -0.2);
    EXPECT_DOUBLE_EQ(event->accel_z, 9.8);
    EXPECT_FALSE(event->is_upside_down);
    EXPECT_EQ(event->loop_us, 812);
    EXPECT_EQ(event->wifi_clients, 1);
    EXPECT_DOUBLE_EQ(event->orientation_x, 359.9);
    EXPECT_DOUBLE_EQ(event->orientation_y, -1.5);
    EXPECT_DOUBLE_EQ(event->orientation_z, 2.3);
    EXPECT_DOUBLE_EQ(event->pid_setpoint, 4.0);
    EXPECT_DOUBLE_EQ(event->pid_output, -0.25);
    EXPECT_FALSE(event->vbat.has_value());
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyOneFieldsWithVbat) {
    auto event = parse_esp32_diagnostics_csv(kLine21, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_TRUE(event->is_upside_down);
    EXPECT_TRUE(event->button_state);
    ASSERT_TRUE(event->vbat.has_value());
    EXPECT_DOUBLE_EQ(*event->vbat, 15.842);
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyTwoFieldsWithIbat) {
    auto event = parse_esp32_diagnostics_csv(kLine22, 0);
    ASSERT_TRUE(event.has_value());
    ASSERT_TRUE(event->vbat.has_value());
    EXPECT_DOUBLE_EQ(*event->vbat, 15.201);
    ASSERT_TRUE(event->ibat.has_value());
    EXPECT_DOUBLE_EQ(*event->ibat, -42.37);
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"ibat\":-42.37"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, OlderLinesHaveNoIbat) {
    auto event = parse_esp32_diagnostics_csv(kLine21, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->ibat.has_value());
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"ibat\":null"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, NanIbatIsAbsentNotAnError) {
    auto event = parse_esp32_diagnostics_csv(
        "1,0,0,0.0,0.0,0,0,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00,nan,nan", 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->vbat.has_value());
    EXPECT_FALSE(event->ibat.has_value());
}

TEST(Esp32DiagnosticsParseTest, NanVbatIsAbsentNotAnError) {
    auto event = parse_esp32_diagnostics_csv(kLine21Nan, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->vbat.has_value());
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"vbat\":null"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, AcceptsTrailingCarriageReturn) {
    EXPECT_TRUE(parse_esp32_diagnostics_csv(std::string(kLine21) + "\r", 0).has_value());
}

TEST(Esp32DiagnosticsParseTest, RejectsMalformedLines) {
    EXPECT_FALSE(parse_esp32_diagnostics_csv("", 0).has_value());
    EXPECT_FALSE(parse_esp32_diagnostics_csv("connected", 0).has_value());
    // 19 fields
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,1,0.0,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0", 0)
                     .has_value());
    // 23 fields
    EXPECT_FALSE(parse_esp32_diagnostics_csv(std::string(kLine22) + ",1.0", 0).has_value());
    // A non-number in a numeric field
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,x,0.0,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00", 0)
                     .has_value());
    // An empty field
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,1,,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00", 0)
                     .has_value());
    // Trailing junk after a number
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,1,0.0abc,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00", 0)
                     .has_value());
}

TEST(Esp32DiagnosticsParseTest, JsonCarriesEveryFieldWithItsType) {
    auto event = parse_esp32_diagnostics_csv(kLine21, 1788011445339499712ULL);
    ASSERT_TRUE(event.has_value());
    const std::string json = to_esp32_diagnostics_json(*event);
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json.back(), '}');
    for (const char *expected : {"\"host_receive_ns\":1788011445339499712",
                                 "\"timestamp_ms\":123457",
                                 "\"radio_connected\":true",
                                 "\"armed\":false",
                                 "\"a_percent\":50",
                                 "\"b_percent\":0",
                                 "\"button_state\":true",
                                 "\"flip_switch\":0",
                                 "\"left_cmd\":48",
                                 "\"right_cmd\":52",
                                 "\"accel_x\":1.1",
                                 "\"accel_y\":0",
                                 "\"accel_z\":9.7",
                                 "\"is_upside_down\":true",
                                 "\"loop_us\":905",
                                 "\"wifi_clients\":2",
                                 "\"orientation_x\":10",
                                 "\"orientation_y\":0.5",
                                 "\"orientation_z\":-0.5",
                                 "\"pid_setpoint\":0",
                                 "\"pid_output\":0",
                                 "\"vbat\":15.842"}) {
        EXPECT_NE(json.find(expected), std::string::npos) << expected << " missing in " << json;
    }
}

/**
 * A stand-in for the firmware's ESPAsyncWebServer: `/record/start` and `/record/stop` answer
 * "ok", and `/events` streams SSE the way AsyncEventSource frames it. The first stream sends the
 * connect hello, three good rows, one malformed row, then drops; later streams send one row each
 * and stay open.
 */
class FakeEsp32Server {
   public:
    FakeEsp32Server() {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        int reuse = 1;
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = 0;
        ::bind(listen_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
        socklen_t length = sizeof(addr);
        ::getsockname(listen_fd_, reinterpret_cast<sockaddr *>(&addr), &length);
        port_ = ntohs(addr.sin_port);
        ::listen(listen_fd_, 8);
        thread_ = std::thread([this] { serve(); });
    }

    ~FakeEsp32Server() {
        stop_.store(true);
        if (thread_.joinable()) thread_.join();
        for (int fd : open_streams_) ::close(fd);
        ::close(listen_fd_);
    }

    int port() const { return port_; }
    int record_starts() const { return record_starts_.load(); }
    int record_stops() const { return record_stops_.load(); }
    int streams() const { return streams_.load(); }

   private:
    static void send_text(int fd, const std::string &text) {
        ::send(fd, text.data(), text.size(), MSG_NOSIGNAL);
    }

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
            if (request.rfind("GET /record/start ", 0) == 0) {
                record_starts_++;
                send_text(fd, "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nok");
                ::close(fd);
            } else if (request.rfind("GET /record/stop ", 0) == 0) {
                record_stops_++;
                send_text(fd, "HTTP/1.1 200 OK\r\nContent-Length: 2\r\n\r\nok");
                ::close(fd);
            } else if (request.rfind("GET /events ", 0) == 0) {
                const int stream = ++streams_;
                send_text(fd,
                          "HTTP/1.1 200 OK\r\nContent-Type: text/event-stream\r\n"
                          "Cache-Control: no-cache\r\nConnection: keep-alive\r\n\r\n");
                send_text(fd, "retry: 1000\nid: 5\ndata: connected\n\n");
                if (stream == 1) {
                    send_text(fd, std::string("id: 6\ndata: ") + kLine20 + "\n\n");
                    // A row split across two writes still reassembles.
                    send_text(fd, std::string("id: 7\ndata: ") + std::string(kLine21, 10));
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    send_text(fd, std::string(kLine21 + 10) + "\n\n");
                    send_text(fd, std::string("id: 8\ndata: ") + kLine21Nan + "\r\n\r\n");
                    send_text(fd, "id: 9\ndata: 1,2,3\n\n");
                    ::close(fd);  // the robot drops off mid-stream
                } else {
                    send_text(fd, std::string("id: 10\ndata: ") + kLine21 + "\n\n");
                    open_streams_.push_back(fd);
                }
            } else {
                send_text(fd, "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n");
                ::close(fd);
            }
        }
    }

    int listen_fd_ = -1;
    int port_ = 0;
    std::atomic<bool> stop_{false};
    std::atomic<int> record_starts_{0};
    std::atomic<int> record_stops_{0};
    std::atomic<int> streams_{0};
    std::vector<int> open_streams_;
    std::thread thread_;
};

template <typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return predicate();
}

TEST(Esp32WifiDiagnosticsTest, StreamsReconnectsAndStopsRecording) {
    FakeEsp32Server server;
    Esp32WifiDiagnosticsOptions options;
    options.host = "127.0.0.1";
    options.port = server.port();
    options.record_mode = true;
    options.reconnect_period = std::chrono::milliseconds(50);

    std::atomic<uint64_t> fake_clock{1000};
    Esp32WifiDiagnostics diagnostics(options, [&fake_clock] { return fake_clock.fetch_add(1); });
    diagnostics.start();

    std::vector<Esp32DiagnosticsEvent> events;
    ASSERT_TRUE(wait_until(
        [&] {
            for (auto &event : diagnostics.drain()) events.push_back(event);
            return events.size() >= 4;
        },
        std::chrono::seconds(5)));

    ASSERT_GE(events.size(), 4u);
    EXPECT_EQ(events[0].timestamp_ms, 123456u);
    EXPECT_FALSE(events[0].vbat.has_value());
    EXPECT_EQ(events[1].timestamp_ms, 123457u);
    ASSERT_TRUE(events[1].vbat.has_value());
    EXPECT_DOUBLE_EQ(*events[1].vbat, 15.842);
    EXPECT_EQ(events[2].timestamp_ms, 123458u);
    EXPECT_FALSE(events[2].vbat.has_value());
    EXPECT_EQ(events[3].timestamp_ms, 123457u);  // first row of the second stream
    EXPECT_LT(events[0].host_receive_ns, events[3].host_receive_ns);

    const Esp32WifiDiagnosticsStats stats = diagnostics.stats();
    EXPECT_TRUE(stats.connected);
    EXPECT_EQ(stats.events, 4u);
    // The malformed row counts; the "connected" hellos on both streams do not.
    EXPECT_EQ(stats.parse_errors, 1u);
    EXPECT_EQ(stats.reconnects, 1u);
    EXPECT_EQ(server.streams(), 2);
    // Record mode is re-requested on every connection, since a rebooted robot comes back at 10 Hz.
    EXPECT_EQ(server.record_starts(), 2);

    diagnostics.stop();
    EXPECT_EQ(server.record_stops(), 1);
}

TEST(Esp32WifiDiagnosticsTest, NoRecordRequestsOutsideRecordMode) {
    FakeEsp32Server server;
    Esp32WifiDiagnosticsOptions options;
    options.host = "127.0.0.1";
    options.port = server.port();
    options.record_mode = false;
    options.reconnect_period = std::chrono::milliseconds(50);

    Esp32WifiDiagnostics diagnostics(options);
    diagnostics.start();
    ASSERT_TRUE(
        wait_until([&] { return diagnostics.stats().events >= 4; }, std::chrono::seconds(5)));
    diagnostics.stop();
    EXPECT_EQ(server.record_starts(), 0);
    EXPECT_EQ(server.record_stops(), 0);
}

TEST(Esp32WifiDiagnosticsTest, UnreachableRobotNeitherBlocksNorCrashes) {
    // Nothing listens on the discard port of loopback, so every connect is refused.
    Esp32WifiDiagnosticsOptions options;
    options.host = "127.0.0.1";
    options.port = 9;
    options.reconnect_period = std::chrono::milliseconds(20);
    Esp32WifiDiagnostics diagnostics(options);
    diagnostics.start();
    ASSERT_TRUE(wait_until([&] { return diagnostics.stats().connect_failures >= 2; },
                           std::chrono::seconds(5)));
    EXPECT_FALSE(diagnostics.stats().connected);
    EXPECT_TRUE(diagnostics.drain().empty());

    const auto start = std::chrono::steady_clock::now();
    diagnostics.stop();
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
}

}  // namespace
}  // namespace auto_battlebot
