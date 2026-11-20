#include <gtest/gtest.h>

#include <opencv2/opencv.hpp>

#include "engine_selector/make_test_selector.hpp"
#include "mask_model/deeplab_mask_model.hpp"

namespace auto_battlebot {
class DeepLabMaskModelTest : public ::testing::Test {
   protected:
    void SetUp() override {
        test_image.header.stamp = 123.456;
        test_image.header.frame_id = FrameId::CAMERA;
        test_image.image = cv::Mat(100, 100, CV_8UC3, cv::Scalar(100, 150, 200));
    }

    RgbImage test_image;
};

TEST_F(DeepLabMaskModelTest, ConstructorDefaults) {
    DeepLabMaskModelConfiguration config;
    config.engine.candidates = {"/fake/path/model.pt"};
    config.model_type = DeepLabModelType::DeepLabV3;
    DeepLabMaskModel model(config, make_test_selector(config));
    SUCCEED();
}

TEST_F(DeepLabMaskModelTest, ConstructorCustomParams) {
    DeepLabMaskModelConfiguration config;
    config.engine.candidates = {"/fake/path/model.pt"};
    config.model_type = DeepLabModelType::DeepLabV3Plus;
    DeepLabMaskModel model(config, make_test_selector(config));
    SUCCEED();
}

TEST_F(DeepLabMaskModelTest, UpdateWithoutInitialization) {
    DeepLabMaskModelConfiguration config;
    config.engine.candidates = {"/fake/path/model.pt"};
    config.model_type = DeepLabModelType::DeepLabV3;
    DeepLabMaskModel model(config, make_test_selector(config));

    auto result = model.update(test_image);

    EXPECT_TRUE(result.mask.mask.empty());
    EXPECT_EQ(result.mask.label, Label::EMPTY);
    EXPECT_DOUBLE_EQ(result.header.stamp, 0.0);
    EXPECT_EQ(result.header.frame_id, FrameId::EMPTY);
}
TEST(DeepLabInputTest, ConvertsBgrToRgbAndUsesTheTrainingStats) {
    const cv::Mat red_bgr(8, 8, CV_8UC3, cv::Scalar(0, 0, 255));
    std::vector<float> buffer;
    deeplab_input_nchw(red_bgr, 4, 4, 0, buffer);
    ASSERT_EQ(buffer.size(), 3u * 4u * 4u);
    const size_t plane = 16;
    EXPECT_NEAR(buffer[0], (1.0f - kDeepLabTrainingMeanRgb[0]) / kDeepLabTrainingStdRgb[0], 1e-5)
        << "channel 0 is red";
    EXPECT_NEAR(buffer[plane], -kDeepLabTrainingMeanRgb[1] / kDeepLabTrainingStdRgb[1], 1e-5);
    EXPECT_NEAR(buffer[2 * plane], -kDeepLabTrainingMeanRgb[2] / kDeepLabTrainingStdRgb[2], 1e-5)
        << "channel 2 is blue";
}

TEST(DeepLabInputTest, TrainingStatsMatchThePythonTransforms) {
    EXPECT_FLOAT_EQ(kDeepLabTrainingMeanRgb[0], 0.4611f);
    EXPECT_FLOAT_EQ(kDeepLabTrainingMeanRgb[1], 0.4359f);
    EXPECT_FLOAT_EQ(kDeepLabTrainingMeanRgb[2], 0.3905f);
    EXPECT_FLOAT_EQ(kDeepLabTrainingStdRgb[0], 0.2193f);
    EXPECT_FLOAT_EQ(kDeepLabTrainingStdRgb[1], 0.2150f);
    EXPECT_FLOAT_EQ(kDeepLabTrainingStdRgb[2], 0.2109f);
}

}  // namespace auto_battlebot
