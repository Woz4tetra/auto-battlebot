#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

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
constexpr const char *kLine24 =
    "123460,1,1,80.0,0.0,0,0,78.0,82.0,0.5,0.0,9.8,0,910,1,12.0,0.5,-0.5,0.0,0.00,15.201,"
    "-42.37,12.5,nan";
// kLine24, then the 12 motor guard columns, then seq.
constexpr const char *kLine37 =
    "123460,1,1,80.0,0.0,0,0,78.0,82.0,0.5,0.0,9.8,0,910,1,12.0,0.5,-0.5,0.0,0.00,15.201,"
    "-42.37,12.5,nan,31.5,30.25,410.5,nan,6.5,5.75,0.5,1.04,3,1,2,17,4294967295";
constexpr const char *kLine21Nan =
    "123458,0,0,0.0,0.0,0,0,0.0,0.0,0.0,0.0,9.8,0,800,1,0.0,0.0,0.0,0.0,0.00,nan";

TEST(Esp32DiagnosticsParseTest, ParsesTwentyFieldsWithoutVbat) {
    auto event = parse_esp32_diagnostics_csv(kLine20, 42);
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->host_receive_wall_ns, 42u);
    EXPECT_EQ(event->robot_millis, 123456u);
    EXPECT_TRUE(event->radio_connected);
    EXPECT_TRUE(event->armed);
    EXPECT_DOUBLE_EQ(event->a_percent, -12.5);
    EXPECT_DOUBLE_EQ(event->b_percent, 3.0);
    EXPECT_FALSE(event->button_state);
    EXPECT_EQ(event->flip_switch, 1);
    EXPECT_DOUBLE_EQ(event->left_cmd_post_mixer_pct, -15.5);
    EXPECT_DOUBLE_EQ(event->right_cmd_post_mixer_pct, -9.5);
    EXPECT_DOUBLE_EQ(event->accel_x, 0.1);
    EXPECT_DOUBLE_EQ(event->accel_y, -0.2);
    EXPECT_DOUBLE_EQ(event->accel_z, 9.8);
    EXPECT_FALSE(event->is_upside_down);
    EXPECT_EQ(event->loop_us, 812);
    EXPECT_EQ(event->wifi_clients, 1);
    EXPECT_DOUBLE_EQ(event->orientation_x_deg, 359.9);
    EXPECT_DOUBLE_EQ(event->orientation_y_deg, -1.5);
    EXPECT_DOUBLE_EQ(event->orientation_z_deg, 2.3);
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

TEST(Esp32DiagnosticsParseTest, ParsesThirtySevenFieldsWithGuardAndSeq) {
    auto event = parse_esp32_diagnostics_csv(kLine37, 0);
    ASSERT_TRUE(event.has_value());
    ASSERT_TRUE(event->yaw_rate_cw_deg_s.has_value());
    EXPECT_DOUBLE_EQ(*event->yaw_rate_cw_deg_s, 12.5);
    EXPECT_FALSE(event->yaw_rate_cmd_cw_deg_s.has_value());
    ASSERT_TRUE(event->guard.has_value());
    EXPECT_DOUBLE_EQ(event->guard->motor_rise_left_c, 31.5);
    EXPECT_DOUBLE_EQ(event->guard->motor_rise_right_c, 30.25);
    EXPECT_DOUBLE_EQ(event->guard->motor_speed_left_rad_s, 410.5);
    EXPECT_TRUE(std::isnan(event->guard->motor_speed_right_rad_s));
    EXPECT_DOUBLE_EQ(event->guard->motor_current_left_a, 6.5);
    EXPECT_DOUBLE_EQ(event->guard->motor_current_right_a, 5.75);
    EXPECT_DOUBLE_EQ(event->guard->guard_scale, 0.5);
    EXPECT_DOUBLE_EQ(event->guard->guard_gain, 1.04);
    EXPECT_EQ(event->guard->guard_state, MotorGuardState::LIMITING);
    EXPECT_EQ(event->guard->guard_sensors, MotorGuardSensors::SHUNT_ONLY);
    EXPECT_FALSE(event->guard->left_stalled);
    EXPECT_TRUE(event->guard->right_stalled);
    EXPECT_EQ(event->guard->erpm_crc_errors_since_boot, 17u);
    ASSERT_TRUE(event->seq.has_value());
    EXPECT_EQ(*event->seq, 4294967295u);
    const std::string json = to_esp32_diagnostics_json(*event);
    EXPECT_NE(json.find("\"yaw_rate\":12.5"), std::string::npos) << json;
    EXPECT_NE(json.find("\"yaw_rate_cmd\":null,\"motor_rise_left\":31.5"), std::string::npos)
        << json;
    EXPECT_NE(json.find("\"motor_speed_right\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"guard_state\":3,\"guard_sensors\":1,\"stall_flags\":2"),
              std::string::npos)
        << json;
    EXPECT_NE(json.find("\"erpm_crc_errors\":17,\"seq\":4294967295,\"backfill\":null}"),
              std::string::npos)
        << json;
}

TEST(Esp32DiagnosticsParseTest, ParsesTwentyFourFieldsWithoutGuardOrSeq) {
    auto event = parse_esp32_diagnostics_csv(kLine24, 0);
    ASSERT_TRUE(event.has_value());
    EXPECT_TRUE(event->yaw_rate_cw_deg_s.has_value());
    EXPECT_FALSE(event->guard.has_value());
    EXPECT_FALSE(event->seq.has_value());
    const std::string json = to_esp32_diagnostics_json(*event);
    EXPECT_NE(json.find("\"guard_state\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"erpm_crc_errors\":null,\"seq\":null"), std::string::npos) << json;
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

std::string read_firmware_diagnostics_source() {
    const std::filesystem::path path =
        std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() / "firmware" /
        "mr_stabs_mk2" / "src" / "diagnostics_server.cpp";
    std::ifstream in(path);
    std::stringstream buffer;
    buffer << in.rdbuf();
    return buffer.str();
}

TEST(Esp32DiagnosticsParseTest, ParsesLinePrintedByFirmwareFormat) {
    const std::string source = read_firmware_diagnostics_source();
    const size_t format_start = source.find("\"%lu,%d,%d,");
    ASSERT_NE(format_start, std::string::npos);
    const size_t call = source.rfind("snprintf(", format_start);
    ASSERT_NE(call, std::string::npos);
    const size_t end = source.find(");", call);
    ASSERT_NE(end, std::string::npos);
    const std::string statement = source.substr(call, end - call);

    std::string format;
    const std::regex literal("\"([^\"]*)\"");
    const size_t first_argument = statement.find("(unsigned long)");
    ASSERT_NE(first_argument, std::string::npos);
    const std::string format_text = statement.substr(0, first_argument);
    for (std::sregex_iterator it(format_text.begin(), format_text.end(), literal), last; it != last;
         ++it) {
        format += (*it)[1].str();
    }
    EXPECT_EQ(format,
              "%lu,%d,%d,%.1f,%.1f,%d,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%lu,%u,%.1f,%.1f,%.1f,%.1f,"
              "%.2f,%.3f,%.2f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%u,%u,%u,%u,%lu");

    std::vector<std::string> columns;
    const std::string arguments = statement.substr(first_argument);
    const std::regex member("data->(\\w+)");
    for (std::sregex_iterator it(arguments.begin(), arguments.end(), member), last; it != last;
         ++it) {
        columns.push_back((*it)[1].str());
    }
    const std::vector<std::string> expected = {"timestamp_ms",
                                               "radio_connected",
                                               "armed",
                                               "a_percent",
                                               "b_percent",
                                               "button_state",
                                               "flip_switch",
                                               "left_cmd",
                                               "right_cmd",
                                               "accel_x",
                                               "accel_y",
                                               "accel_z",
                                               "is_upside_down",
                                               "loop_us",
                                               "wifi_clients",
                                               "orientation_x",
                                               "orientation_y",
                                               "orientation_z",
                                               "pid_setpoint",
                                               "pid_output",
                                               "pack_volts",
                                               "pack_amps_discharge",
                                               "yaw_rate_cw_dps",
                                               "yaw_rate_cmd_cw_dps",
                                               "motor_rise_left_c",
                                               "motor_rise_right_c",
                                               "motor_speed_left_rad_s",
                                               "motor_speed_right_rad_s",
                                               "motor_amps_est_left",
                                               "motor_amps_est_right",
                                               "guard_scale",
                                               "guard_shunt_gain",
                                               "guard_state",
                                               "guard_sensors",
                                               "stall_flags",
                                               "erpm_crc_errors"};
    EXPECT_EQ(columns, expected);
    EXPECT_EQ(columns.size() + 1, static_cast<size_t>(kEsp32DiagnosticsFieldsWithGuard));
}

TEST(Esp32DiagnosticsParseTest, RejectsUnknownFieldCounts) {
    const std::string full = kLine37;
    std::vector<size_t> commas;
    for (size_t i = 0; i < full.size(); ++i) {
        if (full[i] == ',') commas.push_back(i);
    }
    ASSERT_EQ(commas.size() + 1, 37u);
    for (size_t count = 1; count <= 37; ++count) {
        const std::string line = count == 37 ? full : full.substr(0, commas[count - 1]);
        const bool accepted =
            count == 20 || count == 21 || count == 22 || count == 24 || count == 37;
        EXPECT_EQ(parse_esp32_diagnostics_csv(line, 0).has_value(), accepted) << count << " fields";
    }
    std::string bad = full;
    bad.replace(commas[30] + 1, commas[31] - commas[30] - 1, "x");
    EXPECT_FALSE(parse_esp32_diagnostics_csv(bad, 0).has_value());
}

TEST(Esp32DiagnosticsParseTest, NonFiniteBecomesNull) {
    auto event = parse_esp32_diagnostics_csv(kLine37, 0);
    ASSERT_TRUE(event.has_value());
    event->a_percent = std::nan("");
    event->vbat = 15.0;
    const std::string json = to_esp32_diagnostics_json(*event);
    EXPECT_NE(json.find("\"a_percent\":null"), std::string::npos) << json;
    EXPECT_NE(json.find("\"motor_speed_right\":null"), std::string::npos) << json;
    EXPECT_EQ(json.find("nan"), std::string::npos) << json;
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
    // 25 fields, the layout before the guard columns
    EXPECT_FALSE(parse_esp32_diagnostics_csv(std::string(kLine24) + ",7", 0).has_value());
    // 38 fields
    EXPECT_FALSE(parse_esp32_diagnostics_csv(std::string(kLine37) + ",1", 0).has_value());
    // A negative or fractional seq
    const std::string line37(kLine37);
    const std::string prefix = line37.substr(0, line37.rfind(',') + 1);
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
