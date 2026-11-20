#include <bno055_decode.h>
#include <gtest/gtest.h>

using namespace bno055_decode;

namespace {

void put(uint8_t *block, int index, int16_t value) {
    block[index * 2] = static_cast<uint8_t>(value & 0xFF);
    block[index * 2 + 1] = static_cast<uint8_t>((value >> 8) & 0xFF);
}

}  // namespace

TEST(Bno055Decode, GyroIsSixteenLsbPerDpsInRadiansPerSecond) {
    uint8_t block[BLOCK_BYTES] = {};
    put(block, 0, 16);
    put(block, 1, -1600);
    put(block, 2, 0);
    const Vec3 g = gyro_rad_s(block);
    EXPECT_NEAR(g.x, 0.017453293f, 1e-7);
    EXPECT_NEAR(g.y, -100.0f * 0.017453293f, 1e-5);
    EXPECT_EQ(g.z, 0.0f);
}

TEST(Bno055Decode, EulerReadsFromTheSecondHalfOfTheBlock) {
    uint8_t block[BLOCK_BYTES] = {};
    put(block, 3, 5760);
    put(block, 4, -8);
    put(block, 5, 24);
    const Vec3 e = euler_deg(block);
    EXPECT_FLOAT_EQ(e.x, 360.0f);
    EXPECT_FLOAT_EQ(e.y, -0.5f);
    EXPECT_FLOAT_EQ(e.z, 1.5f);
}

TEST(Bno055Decode, LinearAndGravityAreHundredLsbPerMps2) {
    uint8_t block[BLOCK_BYTES] = {};
    put(block, 0, 250);
    put(block, 5, -981);
    EXPECT_FLOAT_EQ(linear_mps2(block).x, 2.5f);
    EXPECT_FLOAT_EQ(gravity_mps2(block).z, -9.81f);
    EXPECT_EQ(gravity_mps2(block).x, 0.0f);
}

TEST(Bno055Decode, SignExtendsNegativeLittleEndianWords) {
    const uint8_t bytes[2] = {0xFF, 0xFF};
    EXPECT_EQ(le16(bytes), -1);
    const uint8_t high[2] = {0x00, 0x80};
    EXPECT_EQ(le16(high), -32768);
}

TEST(Bno055Decode, RegisterBlocksAreContiguous) {
    EXPECT_EQ(REG_GYRO_EULER + VECTOR_BYTES, 0x1A);
    EXPECT_EQ(REG_LINEAR_GRAVITY + VECTOR_BYTES, 0x2E);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
