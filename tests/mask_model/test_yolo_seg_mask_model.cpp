#include <gtest/gtest.h>
#include <toml++/toml.h>

#include <opencv2/opencv.hpp>

#include "config/config_cast.hpp"
#include "config/config_parser.hpp"
#include "engine_selector/make_test_selector.hpp"
#include "mask_model/config.hpp"
#include "mask_model/yolo_seg_mask_model.hpp"

namespace auto_battlebot {
class YoloSegMaskModelTest : public ::testing::Test {
   protected:
    void SetUp() override {
        test_image.header.stamp = 123.456;
        test_image.header.frame_id = FrameId::CAMERA;
        test_image.image = cv::Mat(100, 100, CV_8UC3, cv::Scalar(100, 150, 200));
    }

    RgbImage test_image;
};

TEST_F(YoloSegMaskModelTest, ConstructorDefaults) {
    YoloSegMaskModelConfiguration config;
    config.engine.candidates = {"/fake/path/model.engine"};
    config.label_indices = {Label::FIELD, Label::OPPONENT};
    config.output_label = Label::FIELD;

    YoloSegMaskModel model(config, make_test_selector(config));
    SUCCEED();
}

TEST_F(YoloSegMaskModelTest, UpdateWithoutInitialization) {
    YoloSegMaskModelConfiguration config;
    config.engine.candidates = {"/fake/path/model.engine"};
    config.label_indices = {Label::FIELD, Label::OPPONENT};
    config.output_label = Label::FIELD;

    YoloSegMaskModel model(config, make_test_selector(config));
    auto result = model.update(test_image);

    EXPECT_TRUE(result.mask.mask.empty());
    EXPECT_EQ(result.mask.label, Label::FIELD);
    EXPECT_DOUBLE_EQ(result.header.stamp, test_image.header.stamp);
    EXPECT_EQ(result.header.frame_id, test_image.header.frame_id);
}

TEST(YoloSegMaskModelConfigTest, ParseConfigWithDefaults) {
    toml::table table = toml::parse(R"(
type = "YoloSegMaskModel"
label_indices = ["FIELD", "OPPONENT"]

[engine]
candidates = ["data/models/field_yolo.engine"]
)");
    ConfigParser parser(table, "field_model");
    auto base_config = parse_mask_model_config(parser);

    ASSERT_NE(base_config, nullptr);
    EXPECT_EQ(base_config->type, "YoloSegMaskModel");

    const auto &config = config_cast<YoloSegMaskModelConfiguration>(*base_config);
    ASSERT_EQ(config.engine.candidates.size(), 1);
    EXPECT_EQ(config.engine.candidates[0], "data/models/field_yolo.engine");
    ASSERT_EQ(config.label_indices.size(), 2);
    EXPECT_EQ(config.label_indices[0], Label::FIELD);
    EXPECT_EQ(config.label_indices[1], Label::OPPONENT);
    EXPECT_EQ(config.output_label, Label::FIELD);
    EXPECT_FLOAT_EQ(config.confidence_threshold, 0.5f);
    EXPECT_FLOAT_EQ(config.iou_threshold, 0.45f);
    EXPECT_FLOAT_EQ(config.mask_threshold, 0.5f);
    EXPECT_FLOAT_EQ(config.letterbox_padding, 0.1f);
    EXPECT_EQ(config.max_detections, 32);
    EXPECT_FALSE(config.debug_visualization);
}
}  // namespace auto_battlebot
