#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <thread>

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
        auto frame = viz::encode_subscriber_count(channel, count);
        ASSERT_EQ(::send(client_fd_, frame.data(), frame.size(), MSG_NOSIGNAL),
                  static_cast<ssize_t>(frame.size()));
    }

    void drop_client() {
        ::close(client_fd_);
        client_fd_ = -1;
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

}  // namespace auto_battlebot
