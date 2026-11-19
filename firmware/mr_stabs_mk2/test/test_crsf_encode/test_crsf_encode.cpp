#include <crsf_encode.h>
#include <gtest/gtest.h>

#include <cmath>

using crsf_encode::to_be_i16;
using crsf_encode::to_be_u16;

TEST(CrsfEncode, U16IsBigEndian) { EXPECT_EQ(to_be_u16(0x1234), htobe16(0x1234)); }

TEST(CrsfEncode, U16NaNAndNegativeBecomeZero) {
    EXPECT_EQ(to_be_u16(NAN), 0);
    EXPECT_EQ(to_be_u16(-5.0f), 0);
}

TEST(CrsfEncode, U16ClampsAtMax) { EXPECT_EQ(be16toh(to_be_u16(1e9f)), 65535); }

TEST(CrsfEncode, I16NaNBecomesZero) { EXPECT_EQ(to_be_i16(NAN), 0); }

TEST(CrsfEncode, I16ClampsBothEnds) {
    EXPECT_EQ(static_cast<int16_t>(be16toh(to_be_i16(1e9f))), 32767);
    EXPECT_EQ(static_cast<int16_t>(be16toh(to_be_i16(-1e9f))), -32768);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
