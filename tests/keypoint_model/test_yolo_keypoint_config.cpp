#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <string>

#include "config/config_parser.hpp"
#include "keypoint_model/config.hpp"

namespace auto_battlebot {
namespace {

YoloKeypointModelConfiguration parse_yolo(const std::string &extra) {
    toml::table table = toml::parse(R"(
label_indices = ["MR_STABS_MK2", "MRS_BUFF_MK3"]

[engine]
candidates = ["a.engine"]

[[label_map]]
label = "MR_STABS_MK2"
keypoints = ["MR_STABS_MK2_FRONT", "MR_STABS_MK2_BACK"]
)" + extra);
    ConfigParser parser(table, "keypoint_model");
    YoloKeypointModelConfiguration config;
    config.parse_fields(parser);
    return config;
}

TEST(YoloKeypointModelConfigTest, ReferenceOffsetsDefaultToEmpty) {
    EXPECT_TRUE(parse_yolo("").reference_offset_m_per_label.empty());
}

TEST(YoloKeypointModelConfigTest, ParsesReferenceOffsetsPerLabel) {
    const auto config = parse_yolo(R"(
[reference_offset_m_per_label]
MR_STABS_MK2 = -0.0455
MRS_BUFF_MK3 = 0.0104
)");
    ASSERT_EQ(config.reference_offset_m_per_label.size(), 2u);
    EXPECT_DOUBLE_EQ(config.reference_offset_m_per_label.at(Label::MR_STABS_MK2), -0.0455);
    EXPECT_DOUBLE_EQ(config.reference_offset_m_per_label.at(Label::MRS_BUFF_MK3), 0.0104);
}

TEST(YoloKeypointModelConfigTest, RejectsUnknownLabelAndNonNumber) {
    EXPECT_THROW(parse_yolo("[reference_offset_m_per_label]\nMR_STABZ = 0.1\n"),
                 ConfigValidationError);
    EXPECT_THROW(parse_yolo("[reference_offset_m_per_label]\nMR_STABS_MK2 = \"far\"\n"),
                 ConfigValidationError);
}

}  // namespace
}  // namespace auto_battlebot
