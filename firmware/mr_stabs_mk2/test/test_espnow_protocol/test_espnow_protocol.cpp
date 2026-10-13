#include <command_tracker.h>
#include <espnow_protocol.h>
#include <gtest/gtest.h>
#include <string.h>

#include <vector>

using namespace espnow_protocol;

namespace {

struct Received {
    uint8_t type;
    std::vector<uint8_t> payload;
};

void collect(void *context, uint8_t type, const uint8_t *payload, uint8_t length) {
    static_cast<std::vector<Received> *>(context)->push_back(
        {type, std::vector<uint8_t>(payload, payload + length)});
}

std::vector<uint8_t> serial_frame(uint8_t type, const std::vector<uint8_t> &payload) {
    std::vector<uint8_t> out(SERIAL_OVERHEAD + payload.size());
    out.resize(encode_serial_frame(type, payload.data(), static_cast<uint8_t>(payload.size()),
                                   out.data()));
    return out;
}

CommandPayload command(uint32_t seq, float channel_a = 0.5f) { return {seq, 0, channel_a, 0.0f}; }

}  // namespace

TEST(Crc16, MatchesCcittFalseCheckValue) {
    const char *check = "123456789";
    EXPECT_EQ(crc16_ccitt(reinterpret_cast<const uint8_t *>(check), 9), 0x29B1);
}

TEST(SerialFrame, MagicIsLittleEndianB07B) {
    const auto frame = serial_frame(MSG_COMMAND, {1, 2});
    ASSERT_EQ(frame.size(), 8u);
    EXPECT_EQ(frame[0], 0x7B);
    EXPECT_EQ(frame[1], 0xB0);
    EXPECT_EQ(frame[2], MSG_COMMAND);
    EXPECT_EQ(frame[3], 2);
}

TEST(SerialFrameParser, ParsesFramesSplitAcrossFeeds) {
    std::vector<uint8_t> stream = serial_frame(MSG_COMMAND, {1, 2, 3});
    const auto second = serial_frame(MSG_OTA_END, {});
    stream.insert(stream.end(), second.begin(), second.end());

    SerialFrameParser parser;
    std::vector<Received> got;
    for (uint8_t byte : stream) parser.feed(&byte, 1, collect, &got);

    ASSERT_EQ(got.size(), 2u);
    EXPECT_EQ(got[0].type, MSG_COMMAND);
    EXPECT_EQ(got[0].payload, (std::vector<uint8_t>{1, 2, 3}));
    EXPECT_EQ(got[1].type, MSG_OTA_END);
    EXPECT_TRUE(got[1].payload.empty());
}

TEST(SerialFrameParser, SkipsLeadingTextAndStrayMagicBytes) {
    const std::string text = "\nespnow_dongle mac=d8:3b {\x7b}\n";
    std::vector<uint8_t> stream(text.begin(), text.end());
    const auto frame = serial_frame(MSG_LINK_STATS, {9});
    stream.insert(stream.end(), frame.begin(), frame.end());

    SerialFrameParser parser;
    std::vector<Received> got;
    parser.feed(stream.data(), stream.size(), collect, &got);

    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0].type, MSG_LINK_STATS);
}

TEST(SerialFrameParser, CorruptFrameDoesNotHideTheNextOne) {
    auto bad = serial_frame(MSG_COMMAND, {1, 2, 3, 4});
    bad[5] ^= 0xFF;  // corrupt the payload so the CRC fails
    const auto good = serial_frame(MSG_TELEMETRY, {7});
    // Truncate the bad frame so the good frame starts inside its claimed length.
    std::vector<uint8_t> stream(bad.begin(), bad.begin() + 6);
    stream.insert(stream.end(), good.begin(), good.end());
    stream.insert(stream.end(), good.begin(), good.end());

    SerialFrameParser parser;
    std::vector<Received> got;
    parser.feed(stream.data(), stream.size(), collect, &got);

    ASSERT_GE(got.size(), 1u);
    for (const auto &r : got) EXPECT_EQ(r.type, MSG_TELEMETRY);
    EXPECT_GE(parser.crc_errors(), 1u);
}

TEST(AirFrame, RoundTripsAndRejectsForeignMagic) {
    uint8_t out[AIR_MAX_FRAME];
    const CommandPayload c = command(5);
    const size_t n =
        encode_air_frame(MSG_COMMAND, reinterpret_cast<const uint8_t *>(&c), sizeof(c), out);
    EXPECT_EQ(n, AIR_HEADER_SIZE + sizeof(c));
    EXPECT_TRUE(is_air_frame(out, n));
    out[0] ^= 1;
    EXPECT_FALSE(is_air_frame(out, n));
}

TEST(AirFrame, RefusesPayloadPastTheEspNowLimit) {
    uint8_t payload[AIR_MAX_PAYLOAD + 1] = {};
    uint8_t out[AIR_MAX_FRAME + 1];
    EXPECT_EQ(encode_air_frame(MSG_OTA_CHUNK, payload, sizeof(payload), out), 0u);
    EXPECT_EQ(encode_air_frame(MSG_OTA_CHUNK, payload, AIR_MAX_PAYLOAD, out), AIR_MAX_FRAME);
}

TEST(CommandTracker, NeedsAStreakBeforeFresh) {
    CommandTracker tracker;
    tracker.on_command(command(1), 0);
    EXPECT_FALSE(tracker.fresh(0));
    tracker.on_command(command(2), 4);
    EXPECT_FALSE(tracker.fresh(4));
    tracker.on_command(command(3), 8);
    EXPECT_TRUE(tracker.fresh(8));
}

TEST(CommandTracker, RepeatsOfTheSameSeqKeepItFresh) {
    CommandTracker tracker;
    for (uint32_t t = 0; t <= 200; t += 4) {
        EXPECT_NE(tracker.on_command(command(7), t), CommandTracker::DROPPED_OLD);
    }
    EXPECT_TRUE(tracker.fresh(200));
    EXPECT_EQ(tracker.latest().seq, 7u);
}

TEST(CommandTracker, RepeatDoesNotChangeValues) {
    CommandTracker tracker;
    tracker.on_command(command(4, 0.25f), 0);
    EXPECT_EQ(tracker.on_command(command(4, 0.9f), 4), CommandTracker::ACCEPTED_REPEAT);
    EXPECT_FLOAT_EQ(tracker.latest().channel_a, 0.25f);
}

TEST(CommandTracker, DropsOlderSeq) {
    CommandTracker tracker;
    tracker.on_command(command(10), 0);
    EXPECT_EQ(tracker.on_command(command(9), 4), CommandTracker::DROPPED_OLD);
    EXPECT_EQ(tracker.latest().seq, 10u);
}

TEST(CommandTracker, GoesStaleImmediatelyAndNeedsANewStreak) {
    CommandTracker tracker;
    for (uint32_t seq = 1; seq <= 5; seq++) tracker.on_command(command(seq), seq * 4);
    EXPECT_TRUE(tracker.fresh(20));
    EXPECT_TRUE(tracker.fresh(20 + COMMAND_STALE_MS));
    EXPECT_FALSE(tracker.fresh(20 + COMMAND_STALE_MS + 1));

    const uint32_t back = 20 + COMMAND_STALE_MS + 10;
    tracker.on_command(command(6), back);
    EXPECT_FALSE(tracker.fresh(back));
    tracker.on_command(command(7), back + 4);
    tracker.on_command(command(8), back + 8);
    EXPECT_TRUE(tracker.fresh(back + 8));
}

TEST(CommandTracker, RebaselinesAfterSilenceSoARestartedAppIsHeard) {
    CommandTracker tracker;
    tracker.on_command(command(5000), 0);
    EXPECT_EQ(tracker.on_command(command(1), 100), CommandTracker::DROPPED_OLD);
    EXPECT_EQ(tracker.on_command(command(1), REACQUIRE_SILENCE_MS + 1),
              CommandTracker::ACCEPTED_NEW);
    EXPECT_EQ(tracker.latest().seq, 1u);
}

TEST(CommandTracker, AgeIsMaxBeforeTheFirstCommand) {
    CommandTracker tracker;
    EXPECT_EQ(tracker.age_ms(1000), UINT32_MAX);
    EXPECT_FALSE(tracker.fresh(0));
}

TEST(OtaSequencer, AcceptsOnlyTheNextOffset) {
    OtaSequencer ota;
    ota.begin(500);
    EXPECT_EQ(ota.on_chunk(240, 240), OtaSequencer::DROP);
    EXPECT_EQ(ota.on_chunk(0, 240), OtaSequencer::ACCEPT);
    EXPECT_EQ(ota.on_chunk(0, 240), OtaSequencer::DROP);  // duplicate
    EXPECT_EQ(ota.on_chunk(240, 240), OtaSequencer::ACCEPT);
    EXPECT_EQ(ota.on_chunk(480, 40), OtaSequencer::DROP);  // runs past the end
    EXPECT_EQ(ota.on_chunk(480, 20), OtaSequencer::ACCEPT);
    EXPECT_TRUE(ota.complete());
}

TEST(OtaSequencer, AcksEveryEightChunksAndAtTheEnd) {
    OtaSequencer ota;
    ota.begin(10 * 240);
    for (uint32_t i = 0; i < 7; i++) {
        ota.on_chunk(i * 240, 240);
        EXPECT_FALSE(ota.take_ack_due());
    }
    ota.on_chunk(7 * 240, 240);
    EXPECT_TRUE(ota.take_ack_due());
    EXPECT_FALSE(ota.take_ack_due());
    ota.on_chunk(8 * 240, 240);
    EXPECT_FALSE(ota.take_ack_due());
    ota.on_chunk(9 * 240, 240);
    EXPECT_TRUE(ota.take_ack_due());
    EXPECT_TRUE(ota.complete());
}

TEST(OtaSequencer, DropsEverythingWhenIdle) {
    OtaSequencer ota;
    EXPECT_EQ(ota.on_chunk(0, 240), OtaSequencer::DROP);
    ota.begin(240);
    ota.reset();
    EXPECT_EQ(ota.on_chunk(0, 240), OtaSequencer::DROP);
    EXPECT_FALSE(ota.complete());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
