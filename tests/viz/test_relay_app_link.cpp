#include <gtest/gtest.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <thread>
#include <vector>

#include "viz/frame.hpp"
#include "viz/relay_app_link.hpp"

namespace auto_battlebot::viz {
namespace {

constexpr size_t kBodyBytes = 16 * 1024;

std::vector<std::byte> numbered_frame(uint32_t number) {
    std::vector<std::byte> out;
    detail::put_u32(out, static_cast<uint32_t>(kBodyBytes));
    detail::put_u32(out, number);
    out.resize(kFrameHeaderBytes + kBodyBytes, static_cast<std::byte>(number & 0xff));
    return out;
}

class RelayAppLinkTest : public ::testing::Test {
   protected:
    void SetUp() override {
        char dir_template[] = "/tmp/relay_link_XXXXXX";
        ASSERT_NE(::mkdtemp(dir_template), nullptr);
        dir_ = dir_template;
        path_ = (dir_ / "relay.sock").string();
    }

    void TearDown() override {
        for (int fd : clients_) ::close(fd);
        link_.reset();
        std::filesystem::remove_all(dir_);
    }

    void build(RelayAppLinkConfig config = {}) {
        RelayAppLinkHooks hooks;
        hooks.on_frame = [this](const std::byte *body, size_t len) {
            frames_.emplace_back(body, body + len);
        };
        hooks.on_connected = [this] { ++connects_; };
        hooks.on_disconnected = [this] { ++disconnects_; };
        link_ = std::make_unique<RelayAppLink>(path_, config, hooks);
        ASSERT_TRUE(link_->open_listener());
    }

    int connect_client() {
        int fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
        EXPECT_EQ(::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
        clients_.push_back(fd);
        return fd;
    }

    void close_client(int fd) {
        ::close(fd);
        clients_.erase(std::find(clients_.begin(), clients_.end(), fd));
    }

    static std::vector<std::byte> drain(int fd, RelayAppLink *link) {
        std::vector<std::byte> bytes;
        std::byte chunk[1 << 16];
        int idle = 0;
        while (idle < 5) {
            if (link != nullptr) link->poll_once(0);
            const ssize_t n = ::recv(fd, chunk, sizeof(chunk), MSG_DONTWAIT);
            if (n > 0) {
                bytes.insert(bytes.end(), chunk, chunk + n);
                idle = 0;
            } else {
                ++idle;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
        return bytes;
    }

    std::filesystem::path dir_;
    std::string path_;
    std::unique_ptr<RelayAppLink> link_;
    std::vector<int> clients_;
    std::vector<std::vector<std::byte>> frames_;
    int connects_ = 0;
    int disconnects_ = 0;
};

TEST_F(RelayAppLinkTest, FullOutboundQueueDropsWholeFramesNeverHalfOnes) {
    build({.max_out_bytes = 4 * (kBodyBytes + kFrameHeaderBytes) + 100});
    const int client = connect_client();
    link_->poll_once(50);
    ASSERT_TRUE(link_->connected());

    constexpr uint32_t kSent = 400;
    for (uint32_t i = 0; i < kSent; ++i) link_->send(numbered_frame(i));

    const std::vector<std::byte> received = drain(client, link_.get());
    const size_t frame_bytes = kFrameHeaderBytes + kBodyBytes;
    ASSERT_EQ(received.size() % frame_bytes, 0u) << "a frame was cut in half";
    const size_t count = received.size() / frame_bytes;
    EXPECT_LT(count, kSent) << "nothing was dropped";
    EXPECT_GT(count, 0u);

    int64_t previous = -1;
    for (size_t i = 0; i < count; ++i) {
        size_t off = i * frame_bytes;
        uint32_t len = 0;
        uint32_t number = 0;
        ASSERT_TRUE(detail::get_u32(received.data(), received.size(), off, len));
        ASSERT_EQ(len, kBodyBytes);
        ASSERT_TRUE(detail::get_u32(received.data(), received.size(), off, number));
        EXPECT_GT(static_cast<int64_t>(number), previous);
        previous = number;
        EXPECT_EQ(received[off + 100], static_cast<std::byte>(number & 0xff));
    }
}

TEST_F(RelayAppLinkTest, RestartedAppConnectingInTheSamePollAsEofTakesTheSlot) {
    build();
    const int first = connect_client();
    link_->poll_once(50);
    ASSERT_TRUE(link_->connected());
    ASSERT_EQ(connects_, 1);

    close_client(first);
    const int second = connect_client();
    link_->poll_once(50);

    EXPECT_TRUE(link_->connected());
    EXPECT_EQ(connects_, 2) << "the restarted app was refused";
    EXPECT_EQ(disconnects_, 1);

    link_->send(numbered_frame(7));
    EXPECT_FALSE(drain(second, link_.get()).empty());
}

TEST_F(RelayAppLinkTest, SecondAppIsRefusedWhileFirstStaysAttached) {
    build();
    const int first = connect_client();
    link_->poll_once(50);
    ASSERT_TRUE(link_->connected());

    const int second = connect_client();
    link_->poll_once(50);

    std::byte probe;
    EXPECT_EQ(::recv(second, &probe, 1, MSG_DONTWAIT), 0) << "second app was kept open";
    EXPECT_EQ(connects_, 1);
    EXPECT_EQ(disconnects_, 0);

    link_->send(numbered_frame(3));
    EXPECT_FALSE(drain(first, link_.get()).empty());
}

TEST_F(RelayAppLinkTest, InboundFramesReachTheHookWhole) {
    build();
    const int client = connect_client();
    link_->poll_once(50);

    const std::vector<std::byte> frame = numbered_frame(9);
    const size_t half = frame.size() / 2;
    ASSERT_EQ(::send(client, frame.data(), half, 0), static_cast<ssize_t>(half));
    link_->poll_once(50);
    EXPECT_TRUE(frames_.empty());
    ASSERT_EQ(::send(client, frame.data() + half, frame.size() - half, 0),
              static_cast<ssize_t>(frame.size() - half));
    link_->poll_once(50);
    ASSERT_EQ(frames_.size(), 1u);
    EXPECT_EQ(frames_[0].size(), kBodyBytes);
}

TEST(ShutdownWatchdogTest, ExitsAfterTheDeadline) {
    static std::atomic<bool> fired;
    fired = false;
    ShutdownWatchdog::arm(std::chrono::milliseconds(20), [] { fired = true; });
    EXPECT_FALSE(fired.load());
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_TRUE(fired.load());
}

}  // namespace
}  // namespace auto_battlebot::viz
