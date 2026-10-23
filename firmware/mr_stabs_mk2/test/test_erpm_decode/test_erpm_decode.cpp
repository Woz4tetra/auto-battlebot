#include <erpm_decode.h>
#include <gtest/gtest.h>

#include <vector>

namespace {
// RMT ticks per reply bit at DShot300: 2667 ns at 25 ns per tick.
const float kBitTicks = 106.7f;

// The 21 reply bits for a 12-bit payload, start bit in bit 20, as the ESC builds them.
uint32_t reply_bits(uint16_t payload) {
    uint32_t value = payload & 0xFFFu;
    const uint32_t crc = ~(value ^ (value >> 4) ^ (value >> 8)) & 0xFu;
    value = (value << 4) | crc;
    uint32_t bits = 1;  // start bit
    for (int nibble = 3; nibble >= 0; nibble--) {
        bits = (bits << 5) | erpm_decode::GCR_ENCODE[(value >> (4 * nibble)) & 0xF];
    }
    return bits;
}

// Run lengths in ticks between edges. Each 1 is an edge. The run after the last edge merges into
// the idle line, so it is given as tail_ticks: 0 for what the RMT records at its idle threshold,
// or a long run.
std::vector<uint32_t> runs_for(uint32_t bits, uint32_t tail_ticks, float stretch = 1.0f) {
    std::vector<uint32_t> runs;
    int last_edge = -1;
    for (int bit = 20; bit >= 0; bit--) {
        if ((bits >> bit) & 1u) {
            if (last_edge >= 0) {
                runs.push_back(static_cast<uint32_t>((last_edge - bit) * kBitTicks * stretch));
            }
            last_edge = bit;
        }
    }
    runs.push_back(tail_ticks);
    return runs;
}

uint16_t payload_for_period(uint32_t period_us) {
    uint16_t e = 0;
    while ((period_us >> e) > 0x1FF) e++;
    return static_cast<uint16_t>((e << 9) | (period_us >> e));
}

bool decode_runs(const std::vector<uint32_t> &runs, uint32_t &erpm) {
    return erpm_decode::decode(runs.data(), runs.size(), kBitTicks, erpm);
}
}  // namespace

TEST(ErpmDecode, GcrTableRoundTrips) {
    for (uint8_t nibble = 0; nibble < 16; nibble++) {
        EXPECT_EQ(erpm_decode::gcr_decode(erpm_decode::GCR_ENCODE[nibble]), nibble);
    }
    EXPECT_EQ(erpm_decode::gcr_decode(0x00), 0xFF);
}

TEST(ErpmDecode, StoppedCodeReadsZero) {
    uint32_t erpm = 1;
    ASSERT_TRUE(decode_runs(runs_for(reply_bits(erpm_decode::PAYLOAD_STOPPED), 0), erpm));
    EXPECT_EQ(erpm, 0u);
}

TEST(ErpmDecode, FloorPeriod) {
    // 511 << 7 is the stopped code, so 510 << 7 = 65280 us is the slowest real reading.
    uint32_t erpm = 0;
    ASSERT_TRUE(decode_runs(runs_for(reply_bits((7 << 9) | 510), 0), erpm));
    EXPECT_EQ(erpm, 919u);
}

TEST(ErpmDecode, FullSpeed) {
    // 2300 KV x 16.8 V x 7 pole pairs is about 270k eRPM, a 222 us period.
    uint32_t erpm = 0;
    ASSERT_TRUE(decode_runs(runs_for(reply_bits(payload_for_period(222)), 0), erpm));
    EXPECT_EQ(erpm, 270270u);
}

TEST(ErpmDecode, EveryPeriodSurvivesBothTails) {
    // Whether the RMT records the final run as 0 at its idle threshold or as a long run, and with
    // each run 15% long or short, every payload decodes back to itself.
    for (uint32_t period = 20; period < 65000; period = period * 9 / 8 + 1) {
        const uint16_t payload = payload_for_period(period);
        const uint32_t bits = reply_bits(payload);
        uint32_t expected = 0;
        ASSERT_TRUE(erpm_decode::payload_to_erpm(payload, expected));
        for (uint32_t tail : {0u, 800u}) {
            for (float stretch : {0.85f, 1.0f, 1.15f}) {
                uint32_t erpm = 0;
                ASSERT_TRUE(decode_runs(runs_for(bits, tail, stretch), erpm))
                    << "period " << period << " tail " << tail << " stretch " << stretch;
                EXPECT_EQ(erpm, expected);
            }
        }
    }
}

TEST(ErpmDecode, MeasuredBitHandlesAFastEscClock) {
    // Recorded on the robot: the ESC's bits run at 95 ticks, 11% fast. At 25% fast a 3-bit run
    // rounds down against the nominal bit and fails; measuring the bit recovers it.
    for (uint32_t period = 20; period < 65000; period = period * 9 / 8 + 1) {
        const uint16_t payload = payload_for_period(period);
        uint32_t expected = 0;
        ASSERT_TRUE(erpm_decode::payload_to_erpm(payload, expected));
        for (float stretch : {0.75f, 0.89f, 1.2f}) {
            const std::vector<uint32_t> runs = runs_for(reply_bits(payload), 0, stretch);
            uint32_t erpm = 0;
            ASSERT_TRUE(erpm_decode::decode_measured(runs.data(), runs.size(), kBitTicks, erpm))
                << "period " << period << " stretch " << stretch;
            EXPECT_EQ(erpm, expected);
        }
    }
}

TEST(ErpmDecode, FlippedBitFailsCrcOrGcr) {
    const uint32_t bits = reply_bits(payload_for_period(1000));
    for (int bit = 0; bit < 20; bit++) {
        uint16_t payload = 0;
        EXPECT_FALSE(erpm_decode::bits_to_payload(bits ^ (1u << bit), payload)) << "bit " << bit;
    }
}

TEST(ErpmDecode, RejectsMalformedTrains) {
    uint32_t erpm = 0;
    EXPECT_FALSE(erpm_decode::decode(nullptr, 0, kBitTicks, erpm));
    // A long run mid-reply is noise.
    std::vector<uint32_t> runs = runs_for(reply_bits(payload_for_period(1000)), 0);
    runs[1] = 30 * static_cast<uint32_t>(kBitTicks);
    EXPECT_FALSE(decode_runs(runs, erpm));
    // Too few edges for a reply: the last run cannot be inferred into a valid frame.
    const std::vector<uint32_t> short_train = {static_cast<uint32_t>(kBitTicks), 0};
    EXPECT_FALSE(decode_runs(short_train, erpm));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
