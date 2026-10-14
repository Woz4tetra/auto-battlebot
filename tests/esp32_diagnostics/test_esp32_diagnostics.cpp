#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "esp32_diagnostics/esp32_diagnostics_event.hpp"

namespace auto_battlebot {
namespace {

// Lines in the firmware's printf format (firmware/mr_stabs_mk2/src/diagnostics_server.cpp).
constexpr const char *kLine20 =
    "123456,1,1,-12.5,3.0,0,1,-15.5,-9.5,0.1,-0.2,9.8,0,812,1,359.9,-1.5,2.3,4.0,-0.25";
constexpr const char *kLine21 =
    "123457,1,0,50.0,0.0,1,0,48.0,52.0,1.1,0.0,9.7,1,905,2,10.0,0.5,-0.5,0.0,0.00,15.842";
constexpr const char *kLine22 =
    "123459,1,1,80.0,0.0,0,0,78.0,82.0,0.5,0.0,9.8,0,910,1,12.0,0.5,-0.5,0.0,0.00,15.201,"
    "-42.37";
constexpr const char *kLine25 =
    "123460,1,1,80.0,0.0,0,0,78.0,82.0,0.5,0.0,9.8,0,910,1,12.0,0.5,-0.5,0.0,0.00,15.201,"
    "-42.37,12.5,nan,4294967295";
constexpr const char *kLine21Nan =
    "123458,0,0,0.0,0.0,0,0,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00,nan";

TEST(Esp32DiagnosticsParseTest, ParsesTwentyFieldsWithoutVbat) {
    auto event = parse_esp32_diagnostics_csv(kLine20, 42);
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->host_receive_ns, 42u);
    EXPECT_EQ(event->timestamp_ms, 123456u);
    EXPECT_TRUE(event->radio_connected);
    EXPECT_TRUE(event->armed);
    EXPECT_DOUBLE_EQ(event->a_percent, -12.5);
    EXPECT_DOUBLE_EQ(event->b_percent, 3.0);
    EXPECT_FALSE(event->button_state);
    EXPECT_EQ(event->flip_switch, 1);
    EXPECT_DOUBLE_EQ(event->left_cmd, -15.5);
    EXPECT_DOUBLE_EQ(event->right_cmd, -9.5);
    EXPECT_DOUBLE_EQ(event->accel_x, 0.1);
    EXPECT_DOUBLE_EQ(event->accel_y, -0.2);
    EXPECT_DOUBLE_EQ(event->accel_z, 9.8);
    EXPECT_FALSE(event->is_upside_down);
    EXPECT_EQ(event->loop_us, 812);
    EXPECT_EQ(event->wifi_clients, 1);
    EXPECT_DOUBLE_EQ(event->orientation_x, 359.9);
    EXPECT_DOUBLE_EQ(event->orientation_y, -1.5);
    EXPECT_DOUBLE_EQ(event->orientation_z, 2.3);
    EXPECT_DOUBLE_EQ(event->pid_setpoint, 4.0);
    EXPECT_DOUBLE_EQ(event->pid_output, -0.25);
    EXPECT_FALSE(event->vbat.has_value());
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyOneFieldsWithVbat) {
    auto event = parse_esp32_diagnostics_csv(kLine21, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_TRUE(event->is_upside_down);
    EXPECT_TRUE(event->button_state);
    ASSERT_TRUE(event->vbat.has_value());
    EXPECT_DOUBLE_EQ(*event->vbat, 15.842);
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyTwoFieldsWithIbat) {
    auto event = parse_esp32_diagnostics_csv(kLine22, 0);
    ASSERT_TRUE(event.has_value());
    ASSERT_TRUE(event->vbat.has_value());
    EXPECT_DOUBLE_EQ(*event->vbat, 15.201);
    ASSERT_TRUE(event->ibat.has_value());
    EXPECT_DOUBLE_EQ(*event->ibat, -42.37);
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"ibat\":-42.37"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyFiveFieldsWithYawRateAndSeq) {
    auto event = parse_esp32_diagnostics_csv(kLine25, 0);
    ASSERT_TRUE(event.has_value());
    ASSERT_TRUE(event->yaw_rate.has_value());
    EXPECT_DOUBLE_EQ(*event->yaw_rate, 12.5);
    EXPECT_FALSE(event->yaw_rate_cmd.has_value());
    ASSERT_TRUE(event->seq.has_value());
    EXPECT_EQ(*event->seq, 4294967295u);
    const std::string json = to_esp32_diagnostics_json(*event);
    EXPECT_NE(json.find("\"yaw_rate\":12.5"), std::string::npos) << json;
    EXPECT_NE(json.find("\"yaw_rate_cmd\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"seq\":4294967295,\"backfill\":null}"), std::string::npos) << json;
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyFourFieldsWithoutSeq) {
    const std::string line(kLine25);
    auto event = parse_esp32_diagnostics_csv(line.substr(0, line.rfind(',')), 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_TRUE(event->yaw_rate.has_value());
    EXPECT_FALSE(event->seq.has_value());
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"seq\":null"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, OlderLinesHaveNoIbat) {
    auto event = parse_esp32_diagnostics_csv(kLine21, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->ibat.has_value());
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"ibat\":null"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, NanIbatIsAbsentNotAnError) {
    auto event = parse_esp32_diagnostics_csv(
        "1,0,0,0.0,0.0,0,0,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00,nan,nan", 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->vbat.has_value());
    EXPECT_FALSE(event->ibat.has_value());
}

TEST(Esp32DiagnosticsParseTest, NanVbatIsAbsentNotAnError) {
    auto event = parse_esp32_diagnostics_csv(kLine21Nan, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_FALSE(event->vbat.has_value());
    EXPECT_NE(to_esp32_diagnostics_json(*event).find("\"vbat\":null"), std::string::npos);
}

TEST(Esp32DiagnosticsParseTest, AcceptsTrailingCarriageReturn) {
    EXPECT_TRUE(parse_esp32_diagnostics_csv(std::string(kLine21) + "\r", 0).has_value());
}

TEST(Esp32DiagnosticsParseTest, RejectsMalformedLines) {
    EXPECT_FALSE(parse_esp32_diagnostics_csv("", 0).has_value());
    EXPECT_FALSE(parse_esp32_diagnostics_csv("connected", 0).has_value());
    // 19 fields
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,1,0.0,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0", 0)
                     .has_value());
    // 23 fields
    EXPECT_FALSE(parse_esp32_diagnostics_csv(std::string(kLine22) + ",1.0", 0).has_value());
    // 26 fields
    EXPECT_FALSE(parse_esp32_diagnostics_csv(std::string(kLine25) + ",1", 0).has_value());
    // A negative or fractional seq
    const std::string line25(kLine25);
    const std::string prefix = line25.substr(0, line25.rfind(',') + 1);
    EXPECT_FALSE(parse_esp32_diagnostics_csv(prefix + "-1", 0).has_value());
    EXPECT_FALSE(parse_esp32_diagnostics_csv(prefix + "1.5", 0).has_value());
    // A non-number in a numeric field
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,x,0.0,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00", 0)
                     .has_value());
    // An empty field
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,1,,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00", 0)
                     .has_value());
    // Trailing junk after a number
    EXPECT_FALSE(parse_esp32_diagnostics_csv(
                     "1,1,1,0.0abc,0.0,0,1,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00", 0)
                     .has_value());
}

TEST(Esp32DiagnosticsParseTest, JsonCarriesEveryFieldWithItsType) {
    auto event = parse_esp32_diagnostics_csv(kLine21, 1788011445339499712ULL);
    ASSERT_TRUE(event.has_value());
    const std::string json = to_esp32_diagnostics_json(*event);
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json.back(), '}');
    for (const char *expected : {"\"host_receive_ns\":1788011445339499712",
                                 "\"timestamp_ms\":123457",
                                 "\"radio_connected\":true",
                                 "\"armed\":false",
                                 "\"a_percent\":50",
                                 "\"b_percent\":0",
                                 "\"button_state\":true",
                                 "\"flip_switch\":0",
                                 "\"left_cmd\":48",
                                 "\"right_cmd\":52",
                                 "\"accel_x\":1.1",
                                 "\"accel_y\":0",
                                 "\"accel_z\":9.7",
                                 "\"is_upside_down\":true",
                                 "\"loop_us\":905",
                                 "\"wifi_clients\":2",
                                 "\"orientation_x\":10",
                                 "\"orientation_y\":0.5",
                                 "\"orientation_z\":-0.5",
                                 "\"pid_setpoint\":0",
                                 "\"pid_output\":0",
                                 "\"vbat\":15.842"}) {
        EXPECT_NE(json.find(expected), std::string::npos) << expected << " missing in " << json;
    }
}

}  // namespace
}  // namespace auto_battlebot
