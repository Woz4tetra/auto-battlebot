#include <gtest/gtest.h>
#include <usb_framing.h>

#include <cstring>
#include <string>
#include <utility>
#include <vector>

using usb_framing::copy_drainable_log;

namespace {
const size_t kCapacity = 16;

size_t drain(const std::string &content, size_t read_index = 0) {
    char ring[kCapacity] = {};
    std::memcpy(ring, content.data(), content.size());
    char out[kCapacity];
    size_t copied = 0;
    const size_t write_index = (read_index + content.size()) % kCapacity;
    return copy_drainable_log(ring, kCapacity, read_index, write_index, out, copied);
}
}  // namespace

TEST(UsbFraming, DrainsOnlyWholeLines) {
    EXPECT_EQ(drain("ab\ncd"), 3u);
    EXPECT_EQ(drain("ab\ncd\n"), 6u);
}

TEST(UsbFraming, HoldsAPartialLineBack) { EXPECT_EQ(drain("abc"), 0u); }

TEST(UsbFraming, FullRingWithoutNewlineDrains) {
    EXPECT_EQ(drain(std::string(kCapacity - 1, 'x')), kCapacity - 1);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

namespace {

struct FakePort {
    explicit FakePort(std::vector<int> room) : room_per_poll(std::move(room)) {}
    std::vector<int> room_per_poll;
    size_t polls = 0;
    std::string written;
    int availableForWrite() {
        const size_t i = polls < room_per_poll.size() ? polls : room_per_poll.size() - 1;
        polls++;
        return room_per_poll[i];
    }
    size_t write(const uint8_t *data, size_t size) {
        written.append(reinterpret_cast<const char *>(data), size);
        return size;
    }
};

struct FakeClock {
    uint32_t ms = 0;
    int yields = 0;
};

bool write_text(FakePort &port, FakeClock &clock, const std::string &text, uint32_t timeout_ms) {
    return usb_framing::write_whole(
        port, reinterpret_cast<const uint8_t *>(text.data()), text.size(), timeout_ms,
        [&clock] { return clock.ms; }, [&clock] { clock.ms++, clock.yields++; });
}

}  // namespace

TEST(WriteWhole, WritesImmediatelyWhenThereIsRoom) {
    FakePort port{{64}};
    FakeClock clock;
    EXPECT_TRUE(write_text(port, clock, "hello", 1000));
    EXPECT_EQ(port.written, "hello");
    EXPECT_EQ(clock.yields, 0);
}

TEST(WriteWhole, WaitsUntilRoomOpensThenWritesTheWholeBuffer) {
    FakePort port{{0, 2, 4, 5}};
    FakeClock clock;
    EXPECT_TRUE(write_text(port, clock, "hello", 1000));
    EXPECT_EQ(port.written, "hello");
    EXPECT_EQ(clock.yields, 3);
}

TEST(WriteWhole, GivesUpAfterTheTimeoutAndWritesNothing) {
    FakePort port{{3}};
    FakeClock clock;
    EXPECT_FALSE(write_text(port, clock, "hello", 10));
    EXPECT_TRUE(port.written.empty());
    EXPECT_EQ(clock.ms, 11u);
}

TEST(WriteWhole, DoesNotWritePartialDataWhenRoomIsShort) {
    FakePort port{{4}};
    FakeClock clock;
    EXPECT_FALSE(write_text(port, clock, "hello", 5));
    EXPECT_TRUE(port.written.empty());
}

TEST(PushLogChar, StoresCharactersAndAdvancesTheWriteIndex) {
    char ring[4] = {};
    size_t write_index = 0;
    EXPECT_TRUE(usb_framing::push_log_char(ring, 4, write_index, 0, 'a'));
    EXPECT_TRUE(usb_framing::push_log_char(ring, 4, write_index, 0, 'b'));
    EXPECT_EQ(write_index, 2u);
    EXPECT_EQ(ring[0], 'a');
    EXPECT_EQ(ring[1], 'b');
}

TEST(PushLogChar, DropsTheCharacterWhenTheRingIsFull) {
    char ring[4] = {};
    size_t write_index = 0;
    for (char c : {'a', 'b', 'c'})
        EXPECT_TRUE(usb_framing::push_log_char(ring, 4, write_index, 0, c));
    EXPECT_FALSE(usb_framing::push_log_char(ring, 4, write_index, 0, 'd'));
    EXPECT_EQ(write_index, 3u);
    EXPECT_EQ(ring[3], '\0');
}

TEST(PushLogChar, WrapsAroundOnceTheReaderHasDrained) {
    char ring[4] = {};
    size_t write_index = 3;
    EXPECT_TRUE(usb_framing::push_log_char(ring, 4, write_index, 2, 'z'));
    EXPECT_EQ(write_index, 0u);
    EXPECT_EQ(ring[3], 'z');
}
