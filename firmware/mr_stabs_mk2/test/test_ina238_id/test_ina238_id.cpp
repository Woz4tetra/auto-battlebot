#include <gtest/gtest.h>
#include <ina238_id.h>

using namespace ina238_id;

TEST(Ina238Id, AcceptsEveryRevisionOfTheDie) {
    EXPECT_TRUE(is_ina238(0x2381));
    EXPECT_TRUE(is_ina238(0x238F));
}

TEST(Ina238Id, RejectsAnIna228AtTheSameAddress) {
    EXPECT_FALSE(is_ina238(0x2281));
    EXPECT_EQ(die_id(0x2281), INA228_DIE_ID);
}

TEST(Ina238Id, RejectsAnEmptyBus) {
    EXPECT_FALSE(is_ina238(0x0000));
    EXPECT_FALSE(is_ina238(0xFFFF));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
