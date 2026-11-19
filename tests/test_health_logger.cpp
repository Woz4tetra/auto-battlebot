#include <gtest/gtest.h>

#include "health/health_logger.hpp"

namespace auto_battlebot {

TEST(ParseNvpmodelModeTest, TakesTheNameFromTheFirstPowerModeLine) {
    const auto mode = parse_nvpmodel_mode("NV Power Mode: MAXN\n0\n");
    ASSERT_TRUE(mode.has_value());
    EXPECT_EQ(*mode, "MAXN");
}

TEST(ParseNvpmodelModeTest, ReturnsNothingWhenTheOutputFormatChanges) {
    EXPECT_FALSE(parse_nvpmodel_mode("power mode = MAXN\n").has_value());
    EXPECT_FALSE(parse_nvpmodel_mode("").has_value());
}

}  // namespace auto_battlebot
