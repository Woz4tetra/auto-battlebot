#include <gtest/gtest.h>

#include <cmath>

#include "foxglove_adapters/common.hpp"

namespace auto_battlebot {

TEST(FoxgloveCommonTest, TimestampSplitsSecondsAndNanos) {
    auto ts = foxglove_adapters::to_timestamp(123.456);
    EXPECT_EQ(ts.sec, 123u);
    EXPECT_EQ(ts.nsec, 456000000u);
}

TEST(FoxgloveCommonTest, TimestampClampsNanosBelowOneSecond) {
    auto ts = foxglove_adapters::to_timestamp(5.9999999999);
    EXPECT_EQ(ts.sec, 5u);
    EXPECT_LE(ts.nsec, 999999999u);
}

TEST(FoxgloveCommonTest, TimestampZeroForUnstamped) {
    auto ts = foxglove_adapters::to_timestamp(0.0);
    EXPECT_EQ(ts.sec, 0u);
    EXPECT_EQ(ts.nsec, 0u);
}

TEST(FoxgloveCommonTest, FrameIdIsLowerCaseEnumName) {
    EXPECT_EQ(foxglove_adapters::frame_id_string(FrameId::CAMERA_WORLD), "camera_world");
    EXPECT_EQ(foxglove_adapters::frame_id_string(FrameId::OUR_ROBOT_1), "our_robot_1");
}

TEST(FoxgloveCommonTest, JsonEscape) {
    EXPECT_EQ(foxglove_adapters::json_escape("a\"b\\c\nd"), "a\\\"b\\\\c\\nd");
    EXPECT_EQ(foxglove_adapters::json_escape("plain/path.svo2"), "plain/path.svo2");
}

TEST(FoxgloveCommonTest, JsonNumberRoundTripsAndNullsNonFinite) {
    EXPECT_EQ(foxglove_adapters::json_number(12.5), "12.5");
    EXPECT_EQ(foxglove_adapters::json_number(3.0), "3");
    EXPECT_EQ(foxglove_adapters::json_number(std::nan("")), "null");
    EXPECT_EQ(foxglove_adapters::json_number(INFINITY), "null");
}

}  // namespace auto_battlebot
