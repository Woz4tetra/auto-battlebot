#include <gtest/gtest.h>
#include <i2c_scan.h>

#include <set>

struct FakeWire {
    std::set<uint8_t> present;
    uint8_t current = 0;
    void beginTransmission(uint8_t address) { current = address; }
    uint8_t endTransmission() { return present.count(current) ? 0 : 2; }
};

TEST(I2cScan, ResetsTheWatchdogBeforeEveryAddress) {
    FakeWire wire;
    int resets = 0;
    uint8_t found[16];
    i2c_scan::scan(wire, [&]() { resets++; }, found, 16);
    EXPECT_EQ(resets, 0x78 - 0x08);
}

TEST(I2cScan, ListsAckedAddressesInOrder) {
    FakeWire wire;
    wire.present = {0x28, 0x45};
    uint8_t found[16];
    EXPECT_EQ(i2c_scan::scan(wire, []() {}, found, 16), 2);
    EXPECT_EQ(found[0], 0x28);
    EXPECT_EQ(found[1], 0x45);
}

TEST(I2cScan, StopsRecordingAtCapacityButKeepsProbing) {
    FakeWire wire;
    for (uint8_t a = 0x10; a < 0x20; a++) wire.present.insert(a);
    int resets = 0;
    uint8_t found[4];
    EXPECT_EQ(i2c_scan::scan(wire, [&]() { resets++; }, found, 4), 4);
    EXPECT_EQ(resets, 0x78 - 0x08);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
