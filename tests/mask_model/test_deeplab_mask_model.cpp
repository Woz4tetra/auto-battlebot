#include <gtest/gtest.h>
#include "mask_model/deeplab_mask_model.hpp"
#include <opencv2/opencv.hpp>

namespace auto_battlebot
{
    class DeepLabMaskModelTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // Create a simple test image (100x100 BGR)
            test_image.header.stamp = 123.456;
            test_image.header.frame_id = FrameId::CAMERA;
            test_image.image = cv::Mat(100, 100, CV_8UC3, cv::Scalar(100, 150, 200));
        }

        RgbImage test_image;
    };

    // Test constructor with default parameters
    TEST_F(DeepLabMaskModelTest, ConstructorDefaults)
    {
        DeepLabMaskModelConfiguration config;
        config.model_path = "/fake/path/model.pt";
        config.model_type = DeepLabModelType::DeepLabV3;
        DeepLabMaskModel model(config);
        // Should construct without throwing
        SUCCEED();
    }

    // Test constructor with custom parameters
    TEST_F(DeepLabMaskModelTest, ConstructorCustomParams)
    {
        DeepLabMaskModelConfiguration config;
        config.model_path = "/fake/path/model.pt";
        config.model_type = DeepLabModelType::DeepLabV3Plus;
        DeepLabMaskModel model(config);
        // Should construct without throwing
        SUCCEED();
    }

    // Test update returns empty mask when not initialized
    TEST_F(DeepLabMaskModelTest, UpdateWithoutInitialization)
    {
        DeepLabMaskModelConfiguration config;
        config.model_path = "/fake/path/model.pt";
        config.model_type = DeepLabModelType::DeepLabV3;
        DeepLabMaskModel model(config);

        auto result = model.update(test_image);

        // Should return empty mask when not initialized
        EXPECT_TRUE(result.mask.mask.empty());
        EXPECT_EQ(result.mask.label, Label::EMPTY);
        // Header is not preserved when returning early due to no initialization
        EXPECT_DOUBLE_EQ(result.header.stamp, 0.0);
        EXPECT_EQ(result.header.frame_id, FrameId::EMPTY);
    }
} // namespace auto_battlebot
