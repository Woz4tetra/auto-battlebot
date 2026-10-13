#include <gtest/gtest.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "viz/frame.hpp"
#include "viz/viz_sink.hpp"

namespace auto_battlebot {
namespace {

// Minimal stand-in for the relay: one listening unix socket, one accepted client.
class FakeRelay {
   public:
    explicit FakeRelay(const std::string& path) : path_(path) {
        ::unlink(path_.c_str());
        listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
        EXPECT_EQ(::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)), 0);
        EXPECT_EQ(::listen(listen_fd_, 1), 0);
    }
    ~FakeRelay() {
        if (client_fd_ >= 0) ::close(client_fd_);
        if (listen_fd_ >= 0) ::close(listen_fd_);
        ::unlink(path_.c_str());
    }

    bool accept_client(std::chrono::milliseconds timeout) {
        timeval tv{};
        tv.tv_sec = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
        ::setsockopt(listen_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        client_fd_ = ::accept(listen_fd_, nullptr, nullptr);
        return client_fd_ >= 0;
    }

    // Reads one full frame (body without the length prefix).
    bool read_frame(std::vector<std::byte>& body, std::chrono::milliseconds timeout) {
        timeval tv{};
        tv.tv_sec = static_cast<long>(timeout.count() / 1000);
        tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);
        ::setsockopt(client_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        std::byte header[4];
        if (!read_exact(header, 4)) return false;
        size_t off = 0;
        uint32_t len = 0;
        viz::detail::get_u32(header, 4, off, len);
        body.resize(len);
        return read_exact(body.data(), len);
    }

    void send_subscriber_count(uint32_t channel, uint32_t count) {
        send_frame(viz::encode_subscriber_count(channel, count));
    }

    void send_frame(const std::vector<std::byte>& frame) {
        ASSERT_EQ(::send(client_fd_, frame.data(), frame.size(), MSG_NOSIGNAL),
                  static_cast<ssize_t>(frame.size()));
    }

    void drop_client() {
        ::close(client_fd_);
        client_fd_ = -1;
    }

    /** Accepts and closes connections until `duration` passes. Returns how many came in. */
    int refuse_for(std::chrono::milliseconds duration) {
        int accepted = 0;
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            if (accept_client(std::chrono::milliseconds(50))) {
                ++accepted;
                drop_client();
            }
        }
        return accepted;
    }

   private:
    bool read_exact(std::byte* out, size_t len) {
        size_t got = 0;
        while (got < len) {
            ssize_t n = ::recv(client_fd_, out + got, len - got, 0);
            if (n <= 0) return false;
            got += static_cast<size_t>(n);
        }
        return true;
    }

    std::string path_;
    int listen_fd_ = -1;
    int client_fd_ = -1;
};

std::string test_socket_path() {
    return "/tmp/auto_battlebot_viz_test_" + std::to_string(::getpid()) + ".sock";
}

const VizSchema kImageSchema{"foxglove.CompressedImage", "protobuf", ""};
const VizSchema kStatusSchema{"status", "jsonschema", "{}"};

bool wait_until(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!done()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

void publish_bytes(VizSink& sink, uint32_t channel, size_t len, uint64_t log_time) {
    const std::vector<std::byte> payload(len, std::byte{0x5a});
    sink.publish(channel, payload.data(), payload.size(), log_time);
}

}  // namespace

TEST(VizSinkTest, PublishesNothingAndDoesNotBlockWithoutRelay) {
    const auto path = test_socket_path();
    ::unlink(path.c_str());
    const auto start = std::chrono::steady_clock::now();
    VizSink sink(path);
    const uint32_t channel = sink.advertise("/x", "json", VizSchema{}, false);
    EXPECT_EQ(channel, 1u);
    const std::string payload = "{}";
    for (int i = 0; i < 100; ++i) {
        sink.publish(channel, reinterpret_cast<const std::byte*>(payload.data()), payload.size(),
                     1);
    }
    EXPECT_FALSE(sink.connected());
    EXPECT_EQ(sink.dropped_messages(), 100u);
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
}

TEST(VizSinkTest, ReplaysAdvertisesThenStreamsAndReadsSubscriberCounts) {
    const auto path = test_socket_path();
    FakeRelay relay(path);
    VizSink sink(path);
    const uint32_t channel =
        sink.advertise("/topic", "json", VizSchema{"n", "jsonschema", "{}"}, true);
    ASSERT_TRUE(relay.accept_client(std::chrono::seconds(3)));

    std::vector<std::byte> body;
    ASSERT_TRUE(relay.read_frame(body, std::chrono::seconds(2)));
    ASSERT_EQ(viz::frame_kind(body.data(), body.size()), viz::FrameKind::ADVERTISE);
    viz::AdvertiseFrame adv;
    ASSERT_TRUE(viz::decode_advertise(body.data(), body.size(), adv));
    EXPECT_EQ(adv.topic, "/topic");
    EXPECT_EQ(adv.channel_id, channel);
    EXPECT_TRUE(adv.latch);

    // Wait for the sink to consider itself connected before publishing.
    for (int i = 0; i < 100 && !sink.connected(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(sink.connected());

    const std::string payload = "{\"a\":1}";
    sink.publish(channel, reinterpret_cast<const std::byte*>(payload.data()), payload.size(), 42);
    ASSERT_TRUE(relay.read_frame(body, std::chrono::seconds(2)));
    ASSERT_EQ(viz::frame_kind(body.data(), body.size()), viz::FrameKind::MESSAGE);
    viz::MessageFrame msg;
    ASSERT_TRUE(viz::decode_message(body.data(), body.size(), msg));
    EXPECT_EQ(msg.channel_id, channel);
    EXPECT_EQ(msg.log_time_ns, 42u);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(msg.payload), msg.payload_len), payload);

    EXPECT_EQ(sink.num_subscribers(channel), 0u);
    relay.send_subscriber_count(channel, 3);
    for (int i = 0; i < 100 && sink.num_subscribers(channel) != 3; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(sink.num_subscribers(channel), 3u);

    // The relay dropping the connection resets the subscriber count, and the sink reconnects on
    // its own and replays the advertise so the relay can rebuild its channel table.
    relay.drop_client();
    for (int i = 0; i < 200 && sink.num_subscribers(channel) != 0; ++i) {
        sink.publish(channel, reinterpret_cast<const std::byte*>(payload.data()), payload.size(),
                     43);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(sink.num_subscribers(channel), 0u);
    ASSERT_TRUE(relay.accept_client(std::chrono::seconds(3)));
    ASSERT_TRUE(relay.read_frame(body, std::chrono::seconds(2)));
    ASSERT_EQ(viz::frame_kind(body.data(), body.size()), viz::FrameKind::ADVERTISE);
    ASSERT_TRUE(viz::decode_advertise(body.data(), body.size(), adv));
    EXPECT_EQ(adv.topic, "/topic");
}

TEST(VizSinkTest, DeliversClientMessagesToHandler) {
    const auto path = test_socket_path();
    FakeRelay relay(path);
    VizSink sink(path);
    std::atomic<int> calls{0};
    std::string topic;
    sink.set_client_message_handler([&](const std::string& t, const std::byte*, size_t) {
        topic = t;
        calls.fetch_add(1);
    });
    ASSERT_TRUE(relay.accept_client(std::chrono::seconds(3)));
    for (int i = 0; i < 100 && !sink.connected(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(sink.connected());

    const std::string payload = "{}";
    relay.send_frame(viz::encode_client_message(
        "/reinit_field", reinterpret_cast<const std::byte*>(payload.data()), payload.size()));
    for (int i = 0; i < 100 && calls.load() == 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    sink.set_client_message_handler(nullptr);
    EXPECT_EQ(calls.load(), 1);
    EXPECT_EQ(topic, "/reinit_field");
}

// A relay that stops reading fills the socket. Commands coming the other way must still arrive,
// the sink must still shut down, and after the stall timeout it drops the connection.
TEST(VizSinkTest, StalledRelayStillDeliversCommandsAndLetsGo) {
    const auto path = test_socket_path();
    FakeRelay relay(path);
    auto sink = std::make_unique<VizSink>(path);
    std::atomic<int> commands{0};
    sink->set_client_message_handler(
        [&](const std::string&, const std::byte*, size_t) { commands.fetch_add(1); });
    const uint32_t images = sink->advertise("/camera/image", "protobuf", kImageSchema, false);
    ASSERT_TRUE(relay.accept_client(std::chrono::seconds(3)));
    ASSERT_TRUE(wait_until([&] { return sink->connected(); }, std::chrono::seconds(1)));

    // Far more than a unix socket buffers. The relay reads none of it.
    for (int i = 0; i < 20; ++i) publish_bytes(*sink, images, 1 << 20, 1 + i);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const std::string payload = "{}";
    relay.send_frame(viz::encode_client_message("/command/system_action",
                                                reinterpret_cast<const std::byte*>(payload.data()),
                                                payload.size()));
    EXPECT_TRUE(wait_until([&] { return commands.load() == 1; }, std::chrono::milliseconds(300)))
        << "a command waited behind a stalled send";

    EXPECT_TRUE(wait_until([&] { return !sink->connected(); }, std::chrono::seconds(4)))
        << "the stalled connection was never dropped";

    const auto start = std::chrono::steady_clock::now();
    sink.reset();
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(300));
}

TEST(VizSinkTest, ShutsDownWhileTheRelayIsStalled) {
    const auto path = test_socket_path();
    FakeRelay relay(path);
    auto sink = std::make_unique<VizSink>(path);
    const uint32_t images = sink->advertise("/camera/image", "protobuf", kImageSchema, false);
    ASSERT_TRUE(relay.accept_client(std::chrono::seconds(3)));
    ASSERT_TRUE(wait_until([&] { return sink->connected(); }, std::chrono::seconds(1)));
    for (int i = 0; i < 20; ++i) publish_bytes(*sink, images, 1 << 20, 1 + i);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    const auto start = std::chrono::steady_clock::now();
    sink.reset();
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(300));
}

// Status published after a backlog of camera frames goes out ahead of them.
TEST(VizSinkTest, StatusIsSentAheadOfQueuedImages) {
    const auto path = test_socket_path();
    FakeRelay relay(path);
    VizSink sink(path);
    const uint32_t images = sink.advertise("/camera/image", "protobuf", kImageSchema, false);
    const uint32_t status = sink.advertise("/status/system", "json", kStatusSchema, false);
    ASSERT_TRUE(relay.accept_client(std::chrono::seconds(3)));
    ASSERT_TRUE(wait_until([&] { return sink.connected(); }, std::chrono::seconds(1)));

    constexpr int kImages = 10;
    for (int i = 0; i < kImages; ++i) publish_bytes(sink, images, 1 << 20, 100 + i);
    publish_bytes(sink, status, 64, 1);

    std::vector<uint32_t> message_channels;
    std::vector<std::byte> body;
    while (message_channels.size() < kImages + 1 &&
           relay.read_frame(body, std::chrono::seconds(2))) {
        if (viz::frame_kind(body.data(), body.size()) != viz::FrameKind::MESSAGE) continue;
        viz::MessageFrame msg;
        ASSERT_TRUE(viz::decode_message(body.data(), body.size(), msg));
        message_channels.push_back(msg.channel_id);
    }
    ASSERT_EQ(message_channels.size(), static_cast<size_t>(kImages + 1));
    const auto it = std::find(message_channels.begin(), message_channels.end(), status);
    ASSERT_NE(it, message_channels.end());
    // Only a frame already on its way out can go first.
    EXPECT_LE(it - message_channels.begin(), 1);
}

// A frozen relay's listen backlog still completes connect(), and the socket buffers the first
// sends. That is not a relay, so the sink must not log that it connected until one reads.
TEST(VizSinkTest, ConnectedIsLoggedOnlyOnceTheRelayReads) {
    const auto previous_logger = spdlog::default_logger();
    auto ring = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(64);
    spdlog::set_default_logger(std::make_shared<spdlog::logger>("viz_sink_test", ring));
    const auto logged_connected = [&] {
        for (const auto& line : ring->last_formatted()) {
            if (line.find("Connected to viz_relay") != std::string::npos) return true;
        }
        return false;
    };

    const auto path = test_socket_path();
    FakeRelay relay(path);
    {
        VizSink sink(path);
        sink.advertise("/status/system", "json", kStatusSchema, false);
        ASSERT_TRUE(wait_until([&] { return sink.connected(); }, std::chrono::seconds(2)));
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        EXPECT_FALSE(logged_connected()) << "logged a connection no relay had read from";

        ASSERT_TRUE(relay.accept_client(std::chrono::seconds(1)));
        std::vector<std::byte> body;
        ASSERT_TRUE(relay.read_frame(body, std::chrono::seconds(1)));
        EXPECT_TRUE(wait_until(logged_connected, std::chrono::milliseconds(500)));
    }
    spdlog::set_default_logger(previous_logger);
}

// A relay that accepts and at once closes (another app holds it) is retried once a second, not
// in a busy loop.
TEST(VizSinkTest, RefusingRelayIsRetriedOncePerSecond) {
    const auto path = test_socket_path();
    FakeRelay relay(path);
    VizSink sink(path);
    sink.advertise("/status/system", "json", kStatusSchema, false);
    const int attempts = relay.refuse_for(std::chrono::milliseconds(2500));
    EXPECT_GE(attempts, 2);
    EXPECT_LE(attempts, 4);
}

}  // namespace auto_battlebot
