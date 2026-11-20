#include <gtest/gtest.h>
#include <ina238_id.h>

#include <string>
#include <vector>

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

TEST(Ina238Begin, ReadsTheIdBeforeWritingConfigAndSkipsConfigOnAnIna228) {
    std::vector<std::string> order;
    uint16_t id = 0;
    auto read = [&](uint16_t *out) {
        order.push_back("read");
        *out = 0x2281;
        return true;
    };
    auto write = [&]() {
        order.push_back("write");
        return true;
    };
    EXPECT_FALSE(begin_sequence(read, write, id));
    EXPECT_EQ(order, std::vector<std::string>({"read"}));
    EXPECT_EQ(id, 0x2281);
}

TEST(Ina238Begin, WritesConfigOnlyAfterAnIna238Answers) {
    std::vector<std::string> order;
    uint16_t id = 0;
    auto read = [&](uint16_t *out) {
        order.push_back("read");
        *out = 0x2381;
        return true;
    };
    auto write = [&]() {
        order.push_back("write");
        return true;
    };
    EXPECT_TRUE(begin_sequence(read, write, id));
    EXPECT_EQ(order, std::vector<std::string>({"read", "write"}));
}

TEST(Ina238Begin, MissingChipNeverGetsAConfigWrite) {
    bool wrote = false;
    uint16_t id = 0;
    EXPECT_FALSE(begin_sequence([](uint16_t *) { return false; },
                                [&]() {
                                    wrote = true;
                                    return true;
                                },
                                id));
    EXPECT_FALSE(wrote);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
