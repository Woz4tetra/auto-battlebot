#include <gtest/gtest.h>
#include <tuning.h>

#include <cmath>
#include <cstring>
#include <set>
#include <string>

using namespace tuning;

TEST(Tuning, KeysAreUniqueAndFitNvs) {
    std::set<std::string> keys;
    for (size_t i = 0; i < FIELD_COUNT; i++) {
        EXPECT_LE(strlen(FIELDS[i].key), 15u) << FIELDS[i].key;
        EXPECT_TRUE(keys.insert(FIELDS[i].key).second) << FIELDS[i].key;
    }
}

TEST(Tuning, DefaultsAreInsideTheirRanges) {
    Settings defaults;
    for (size_t i = 0; i < FIELD_COUNT; i++) {
        float v = get(defaults, FIELDS[i]);
        EXPECT_GE(v, FIELDS[i].min) << FIELDS[i].key;
        EXPECT_LE(v, FIELDS[i].max) << FIELDS[i].key;
    }
}

TEST(Tuning, EveryFieldPointsAtItsOwnMember) {
    Settings s;
    for (size_t i = 0; i < FIELD_COUNT; i++) *FIELDS[i].ref(s) = (float)i;
    for (size_t i = 0; i < FIELD_COUNT; i++) EXPECT_EQ(get(s, FIELDS[i]), (float)i);
    EXPECT_EQ(s.left_deadzone, 0.0f);
    EXPECT_EQ(s.yaw.kp, (float)(find("kp") - FIELDS));
    EXPECT_EQ(s.yaw.reverse_ff_scale, (float)(find("rev_ff_scale") - FIELDS));
}

TEST(Tuning, SetValidates) {
    Settings s;
    EXPECT_TRUE(set(s, "kp", 0.05f));
    EXPECT_EQ(s.yaw.kp, 0.05f);
    EXPECT_FALSE(set(s, "kp", -1.0f));
    EXPECT_FALSE(set(s, "kp", NAN));
    EXPECT_FALSE(set(s, "kp", INFINITY));
    EXPECT_FALSE(set(s, "no_such_field", 1.0f));
    EXPECT_EQ(s.yaw.kp, 0.05f);
    EXPECT_TRUE(set(s, "left_deadzone", 0.0f));
    EXPECT_FALSE(set(s, "left_deadzone", 51.0f));
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
