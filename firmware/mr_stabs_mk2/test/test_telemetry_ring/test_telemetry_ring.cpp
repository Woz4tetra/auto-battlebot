#include <gtest/gtest.h>
#include <telemetry_ring.h>

#include <atomic>
#include <cstring>
#include <thread>
#include <vector>

namespace {
// value and check are both derived from the seq the writer was on, so a record whose bytes
// come from two different pushes fails the check.
struct Record {
    uint32_t telemetry_seq;
    uint32_t value;
    uint8_t padding[64];
    uint32_t check;
};

Record make(uint32_t value) {
    Record r = {};
    r.value = value;
    std::memset(r.padding, static_cast<int>(value & 0xFF), sizeof(r.padding));
    r.check = ~value;
    return r;
}

bool consistent(const Record &r) {
    if (r.check != ~r.value) return false;
    for (uint8_t b : r.padding) {
        if (b != (r.value & 0xFF)) return false;
    }
    return true;
}
}  // namespace

TEST(TelemetryRing, ReadsBackInOrderWithSeq) {
    std::vector<Record> storage(8);
    telemetry_ring::TelemetryRing<Record> ring;
    ring.init(storage.data(), 8);
    for (uint32_t i = 0; i < 5; i++) ring.push(make(100 + i));

    Record out[8];
    const auto r = ring.read(0, out, 8);
    EXPECT_EQ(r.first_seq, 0u);
    EXPECT_EQ(r.count, 5u);
    EXPECT_EQ(r.head, 5u);
    EXPECT_EQ(r.oldest, 0u);
    for (uint32_t i = 0; i < 5; i++) {
        EXPECT_EQ(out[i].telemetry_seq, i);
        EXPECT_EQ(out[i].value, 100 + i);
    }
}

TEST(TelemetryRing, ResumesFromSeqAndCapsAtMax) {
    std::vector<Record> storage(16);
    telemetry_ring::TelemetryRing<Record> ring;
    ring.init(storage.data(), 16);
    for (uint32_t i = 0; i < 10; i++) ring.push(make(i));

    Record out[16];
    auto r = ring.read(4, out, 3);
    EXPECT_EQ(r.first_seq, 4u);
    EXPECT_EQ(r.count, 3u);
    EXPECT_EQ(out[0].value, 4u);
    EXPECT_EQ(out[2].value, 6u);

    r = ring.read(10, out, 16);  // caught up
    EXPECT_EQ(r.count, 0u);
    EXPECT_EQ(r.first_seq, 10u);
    r = ring.read(50, out, 16);  // past head: nothing, and first_seq says where head is
    EXPECT_EQ(r.count, 0u);
    EXPECT_EQ(r.first_seq, 10u);
}

TEST(TelemetryRing, OverwrittenRecordsSkipToOldest) {
    std::vector<Record> storage(4);
    telemetry_ring::TelemetryRing<Record> ring;
    ring.init(storage.data(), 4);
    for (uint32_t i = 0; i < 10; i++) ring.push(make(i));

    Record out[4];
    const auto r = ring.read(2, out, 4);
    EXPECT_EQ(r.oldest, 6u);
    EXPECT_EQ(r.first_seq, 6u);
    EXPECT_EQ(r.count, 4u);
    EXPECT_EQ(out[0].value, 6u);
    EXPECT_EQ(out[3].value, 9u);
}

TEST(TelemetryRing, ZeroCapacityIsInert) {
    telemetry_ring::TelemetryRing<Record> ring;
    ring.init(nullptr, 0);
    ring.push(make(1));
    Record out[1];
    const auto r = ring.read(0, out, 1);
    EXPECT_EQ(r.count, 0u);
    EXPECT_EQ(r.head, 0u);
}

// A writer laps a small ring while a reader copies from it. Every record the reader keeps must be
// whole and carry the seq it claims; torn ones must be dropped, never returned.
TEST(TelemetryRing, ConcurrentReaderNeverSeesTornRecords) {
    const uint32_t kCapacity = 32;
    const uint32_t kPushes = 400000;
    std::vector<Record> storage(kCapacity);
    telemetry_ring::TelemetryRing<Record> ring;
    ring.init(storage.data(), kCapacity);

    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (uint32_t i = 0; i < kPushes; i++) ring.push(make(i));
        done.store(true);
    });

    uint64_t kept = 0;
    uint32_t next = 0;
    Record out[kCapacity];
    while (!done.load() || next < ring.head()) {
        const auto r = ring.read(next, out, kCapacity);
        for (uint32_t i = 0; i < r.count; i++) {
            ASSERT_TRUE(consistent(out[i])) << "torn record at seq " << r.first_seq + i;
            ASSERT_EQ(out[i].telemetry_seq, r.first_seq + i);
            ASSERT_EQ(out[i].value, r.first_seq + i);
        }
        ASSERT_GE(r.first_seq, next);
        next = r.first_seq + r.count;
        kept += r.count;
    }
    writer.join();
    EXPECT_GT(kept, 0u);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
