#include <gtest/gtest.h>

#include "enums/anti_banding.hpp"

namespace auto_battlebot {

TEST(AntiBandingTest, ValuesMatchTheZedSdkAeAntibandingModes) {
    EXPECT_EQ(static_cast<int>(AntiBanding::OFF), 0);
    EXPECT_EQ(static_cast<int>(AntiBanding::AUTO), 1);
    EXPECT_EQ(static_cast<int>(AntiBanding::HZ_50), 2);
    EXPECT_EQ(static_cast<int>(AntiBanding::HZ_60), 3);
}

}  // namespace auto_battlebot
