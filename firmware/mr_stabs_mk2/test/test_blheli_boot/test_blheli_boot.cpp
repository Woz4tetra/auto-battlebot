#include <blheli_boot.h>
#include <gtest/gtest.h>

#include <deque>
#include <vector>

using namespace blheli_boot;

namespace {
using Bytes = std::vector<uint8_t>;

Bytes request(uint8_t cmd, uint16_t address, const Bytes &params) {
    Bytes frame = {REQUEST_START, cmd, static_cast<uint8_t>(address >> 8),
                   static_cast<uint8_t>(address & 0xFF),
                   static_cast<uint8_t>(params.size() & 0xFF)};
    frame.insert(frame.end(), params.begin(), params.end());
    const uint16_t crc = crc_xmodem(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc >> 8));
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    return frame;
}

Bytes with_boot_crc(Bytes data) {
    const uint16_t crc = crc_boot(data.data(), data.size());
    data.push_back(static_cast<uint8_t>(crc & 0xFF));
    data.push_back(static_cast<uint8_t>(crc >> 8));
    return data;
}

// Replays scripted ESC replies, one per receive() call, and records everything sent.
struct FakeLink : EscLink {
    std::deque<Bytes> replies;
    std::vector<Bytes> sent;
    std::vector<uint8_t> selected;
    uint32_t low_ms = 0;
    uint8_t count() const override { return 2; }
    void select(uint8_t esc) override { selected.push_back(esc); }
    void send(const uint8_t *data, size_t size) override { sent.emplace_back(data, data + size); }
    size_t receive(uint8_t *buf, size_t max, size_t, uint32_t) override {
        if (replies.empty()) return 0;
        Bytes r = replies.front();
        replies.pop_front();
        const size_t n = std::min(r.size(), max);
        std::copy(r.begin(), r.begin() + n, buf);
        return n;
    }
    void hold_low(uint32_t ms) override { low_ms += ms; }
    void delay_ms(uint32_t) override {}
};

struct Reply {
    uint8_t cmd;
    uint16_t address;
    Bytes params;
    uint8_t ack;
};

Reply run(Server &server, const Bytes &frame) {
    uint8_t out[MAX_RESPONSE];
    const size_t n = server.handle(frame.data(), frame.size(), out);
    EXPECT_GE(n, 8u);
    EXPECT_EQ(out[0], RESPONSE_START);
    EXPECT_EQ(crc_xmodem(out, n - 2), (out[n - 2] << 8) | out[n - 1]);
    const size_t length = out[4] == 0 ? 256 : out[4];
    EXPECT_EQ(n, 8 + length);
    return {out[1], static_cast<uint16_t>((out[2] << 8) | out[3]), Bytes(out + 5, out + 5 + length),
            out[5 + length]};
}

Bytes page_bytes() {
    Bytes page(256);
    for (size_t i = 0; i < page.size(); i++) page[i] = static_cast<uint8_t>(i * 7 + 3);
    return page;
}
}  // namespace

TEST(BlheliBootCrc, XmodemCheckValue) {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc_xmodem(text, sizeof(text)), 0x31C3);
}

TEST(BlheliBootCrc, BootCrcCheckValue) {
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc_boot(text, sizeof(text)), 0xBB3D);
    // A reply with its CRC appended low byte first sums to zero, as the passthrough checks.
    Bytes reply = with_boot_crc({0x10, 0x20, 0x30});
    EXPECT_EQ(crc_boot(reply.data(), reply.size()), 0);
}

TEST(BlheliBootFrames, MatchesThePythonClient) {
    // auto_battlebot.blheli32.fourway.build_request(0x3A, 0xF800, b"\x00")
    EXPECT_EQ(request(DEVICE_READ, 0xF800, {0x00}),
              (Bytes{0x2F, 0x3A, 0xF8, 0x00, 0x01, 0x00, 0x93, 0xF4}));
    Request req;
    const Bytes frame = request(DEVICE_INIT_FLASH, 0, {0x01});
    ASSERT_EQ(parse_request(frame.data(), frame.size(), req), ParseResult::OK);
    EXPECT_EQ(req.cmd, DEVICE_INIT_FLASH);
    EXPECT_EQ(req.length, 1u);
    EXPECT_EQ(req.params[0], 1);
}

TEST(BlheliBootFrames, ParseRejectsBadFrames) {
    Request req;
    Bytes frame = request(DEVICE_READ, 0xF800, {0x00});
    EXPECT_EQ(parse_request(frame.data(), 7, req), ParseResult::SHORT);
    Bytes bad_start = frame;
    bad_start[0] = 0x2E;
    EXPECT_EQ(parse_request(bad_start.data(), bad_start.size(), req), ParseResult::BAD_START);
    Bytes bad_crc = frame;
    bad_crc.back() ^= 1;
    EXPECT_EQ(parse_request(bad_crc.data(), bad_crc.size(), req), ParseResult::BAD_CRC);
    Bytes long_frame = frame;
    long_frame.push_back(0);
    EXPECT_EQ(parse_request(long_frame.data(), long_frame.size(), req), ParseResult::BAD_LENGTH);
    // n = 0 means 256 parameter bytes.
    const Bytes full = request(DEVICE_WRITE, 0xF800, page_bytes());
    ASSERT_EQ(full[4], 0);
    ASSERT_EQ(parse_request(full.data(), full.size(), req), ParseResult::OK);
    EXPECT_EQ(req.length, 256u);
}

TEST(BlheliBootGuard, OnlyTheSettingsPage) {
    EXPECT_TRUE(write_allowed(0xF800, 256));
    EXPECT_FALSE(write_allowed(0xF7FF, 256));
    EXPECT_FALSE(write_allowed(0xF801, 256));
    EXPECT_FALSE(write_allowed(0xF800, 255));
    EXPECT_FALSE(write_allowed(0x0000, 256));
}

TEST(BlheliBootServer, GuardRefusesOtherWritesAndErases) {
    FakeLink link;
    Server server(link);
    for (uint16_t address : {0xF700, 0xF7FF, 0xF801, 0x1000}) {
        Reply r = run(server, request(DEVICE_WRITE, address, page_bytes()));
        EXPECT_EQ(r.ack, ACK_INVALID_PARAM) << address;
    }
    EXPECT_EQ(run(server, request(DEVICE_WRITE, 0xF800, Bytes(16, 0))).ack, ACK_INVALID_PARAM);
    EXPECT_EQ(run(server, request(DEVICE_PAGE_ERASE, 0, {0x3E})).ack, ACK_INVALID_PARAM);
    EXPECT_EQ(run(server, request(DEVICE_ERASE_ALL, 0, {0x00})).ack, ACK_INVALID_PARAM);
    EXPECT_EQ(server.guard_refusals(), 7u);
    EXPECT_TRUE(link.sent.empty());  // nothing reached an ESC
}

TEST(BlheliBootServer, InterfaceCommands) {
    FakeLink link;
    Server server(link);
    EXPECT_EQ(run(server, request(PROTOCOL_GET_VERSION, 0, {0})).params, Bytes{108});
    Reply name = run(server, request(INTERFACE_GET_NAME, 0, {0}));
    EXPECT_EQ(std::string(name.params.begin(), name.params.end()), "MrStabsMk2");
    EXPECT_EQ(run(server, request(INTERFACE_SET_MODE, 0, {MODE_ARM_BLB})).ack, ACK_OK);
    EXPECT_EQ(run(server, request(INTERFACE_SET_MODE, 0, {1})).ack, ACK_INVALID_PARAM);
    EXPECT_EQ(run(server, request(0x40, 0, {0})).ack, ACK_INVALID_CMD);
    EXPECT_EQ(run(server, request(DEVICE_RESET, 0, {2})).ack, ACK_INVALID_CHANNEL);
    EXPECT_EQ(run(server, request(DEVICE_INIT_FLASH, 0, {2})).ack, ACK_INVALID_CHANNEL);
}

TEST(BlheliBootServer, BadFrameCrcIsAnsweredNotForwarded) {
    FakeLink link;
    Server server(link);
    Bytes frame = request(DEVICE_READ, 0xF800, {0x00});
    frame.back() ^= 0xFF;
    EXPECT_EQ(run(server, frame).ack, ACK_INVALID_CRC);
    EXPECT_TRUE(link.sent.empty());
}

TEST(BlheliBootServer, InitFlashReturnsTheSignature) {
    FakeLink link;
    link.replies.push_back({'4', '7', '1', 0x6D, 0x47, 0x06, 0x01, 0x40, BOOT_SUCCESS});
    Server server(link);
    Reply r = run(server, request(DEVICE_INIT_FLASH, 0, {1}));
    EXPECT_EQ(r.ack, ACK_OK);
    EXPECT_EQ(r.params, (Bytes{0x06, 0x47, 0x6D, 0x04}));
    EXPECT_EQ(link.selected.back(), 1);
    ASSERT_EQ(link.sent.size(), 1u);
    EXPECT_EQ(link.sent[0].size(), 17u);  // the handshake goes out with its own CRC
}

TEST(BlheliBootServer, InitFlashWithoutAnEscFails) {
    FakeLink link;
    Server server(link);
    EXPECT_EQ(run(server, request(DEVICE_INIT_FLASH, 0, {0})).ack, ACK_GENERAL_ERROR);
}

TEST(BlheliBootServer, ReadReturnsCheckedData) {
    FakeLink link;
    const Bytes page = page_bytes();
    Bytes reply = with_boot_crc(page);
    reply.push_back(BOOT_SUCCESS);
    link.replies = {{BOOT_SUCCESS}, reply};
    Server server(link);
    Reply r = run(server, request(DEVICE_READ, 0xF800, {0x00}));
    EXPECT_EQ(r.ack, ACK_OK);
    EXPECT_EQ(r.address, 0xF800);
    EXPECT_EQ(r.params, page);
    ASSERT_EQ(link.sent.size(), 2u);
    EXPECT_EQ(link.sent[0], with_boot_crc({BOOT_SET_ADDRESS, 0x00, 0xF8, 0x00}));
    EXPECT_EQ(link.sent[1], with_boot_crc({BOOT_READ_FLASH, 0x00}));
}

TEST(BlheliBootServer, ReadRejectsBadCrcAndShortReplies) {
    const Bytes data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    Bytes good = with_boot_crc(data);
    good.push_back(BOOT_SUCCESS);

    Bytes bad_crc = good;
    bad_crc[3] ^= 0x40;
    Bytes short_reply(good.begin(), good.end() - 4);
    Bytes failed = good;
    failed.back() = 0xC2;
    for (const Bytes &reply : {bad_crc, short_reply, failed}) {
        FakeLink link;
        link.replies = {{BOOT_SUCCESS}, reply};
        Server server(link);
        EXPECT_EQ(run(server, request(DEVICE_READ, 0xFA00, {16})).ack, ACK_GENERAL_ERROR);
    }
    FakeLink no_address;
    Server server(no_address);
    EXPECT_EQ(run(server, request(DEVICE_READ, 0xFA00, {16})).ack, ACK_GENERAL_ERROR);
}

TEST(BlheliBootServer, WriteSendsThePassthroughSequence) {
    FakeLink link;
    link.replies = {{BOOT_SUCCESS}, {BOOT_SUCCESS}, {BOOT_SUCCESS}};
    Server server(link);
    const Bytes page = page_bytes();
    Reply r = run(server, request(DEVICE_WRITE, 0xF800, page));
    EXPECT_EQ(r.ack, ACK_OK);
    ASSERT_EQ(link.sent.size(), 4u);
    EXPECT_EQ(link.sent[0], with_boot_crc({BOOT_SET_ADDRESS, 0x00, 0xF8, 0x00}));
    EXPECT_EQ(link.sent[1], with_boot_crc({BOOT_SET_BUFFER, 0x00, 0x01, 0x00}));
    EXPECT_EQ(link.sent[2], with_boot_crc(page));
    EXPECT_EQ(link.sent[3], with_boot_crc({BOOT_PROG_FLASH, 0x01}));
}

TEST(BlheliBootServer, WriteRetriesThenReportsFailure) {
    FakeLink link;
    // Address on the second try, then no reply to the data or the program command.
    link.replies = {{}, {BOOT_SUCCESS}};
    Server server(link);
    EXPECT_EQ(run(server, request(DEVICE_WRITE, 0xF800, page_bytes())).ack, ACK_GENERAL_ERROR);
    // 2 address tries, set buffer, 3 data tries, 3 program tries.
    EXPECT_EQ(link.sent.size(), 9u);
}

TEST(BlheliBootServer, ResetHoldsTheLineLow) {
    FakeLink link;
    Server server(link);
    EXPECT_EQ(run(server, request(DEVICE_RESET, 0, {1})).ack, ACK_OK);
    EXPECT_EQ(link.selected.back(), 1);
    EXPECT_EQ(link.low_ms, 300u);
    ASSERT_EQ(link.sent.size(), 1u);
    EXPECT_EQ(link.sent[0], with_boot_crc({BOOT_RESTART, 0x00}));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
