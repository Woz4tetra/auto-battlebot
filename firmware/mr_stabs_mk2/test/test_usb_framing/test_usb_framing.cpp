#include <gtest/gtest.h>
#include <usb_framing.h>

#include <cstring>
#include <string>

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
