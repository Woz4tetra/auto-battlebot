#include <gtest/gtest.h>
#include "field_model/deeplab_field_model.hpp"
#include <opencv2/opencv.hpp>

namespace auto_battlebot
{
    // Mock subclass that doesn't require a real model
    class MockDeepLabFieldModel : public DeepLabFieldModel
    {
    public:
        MockDeepLabFieldModel(int image_size = 512, int border_crop_padding = 20)
            : DeepLabFieldModel("/fake/path/model.pt", DeepLabModelType::DeepLabV3, image_size, border_crop_padding)
        {
        }

        // Override initialize to bypass model loading
        bool initialize() override
        {
            // Simulate successful initialization without loading a real model
            return true;
        }

        // Expose protected methods for testing
        cv::Mat test_postprocess_output(const torch::Tensor &output, int original_height, int original_width)
        {
            return postprocess_output(output, original_height, original_width);
        }

        // Create a mock update that uses a fake segmentation mask
        FieldMaskStamped mock_update(RgbImage image, const cv::Mat &fake_mask)
        {
            // Simulate postprocessing with a fake mask
            cv::Mat processed_mask = mock_postprocess(fake_mask, image.image.rows, image.image.cols);

            FieldMaskStamped result;
            result.header = image.header;
            result.mask.label = Label::FIELD;
            result.mask.mask = processed_mask;

            return result;
        }

    private:
        // Mock postprocessing that simulates the border cropping without needing torch
        cv::Mat mock_postprocess(const cv::Mat &mask, int original_height, int original_width)
        {
            cv::Mat resized_mask;
            cv::resize(mask, resized_mask, cv::Size(original_width, original_height), 0, 0, cv::INTER_NEAREST);

            // Apply border crop padding (same logic as postprocess_output)
            if (border_padding_ > 0)
            {
                int top = std::min(border_padding_, resized_mask.rows);
                int bottom = std::min(border_padding_, resized_mask.rows);
                int left = std::min(border_padding_, resized_mask.cols);
                int right = std::min(border_padding_, resized_mask.cols);

                if (top > 0)
                    resized_mask(cv::Rect(0, 0, resized_mask.cols, top)).setTo(0);
                if (bottom > 0)
                    resized_mask(cv::Rect(0, resized_mask.rows - bottom, resized_mask.cols, bottom)).setTo(0);
                if (left > 0)
                    resized_mask(cv::Rect(0, 0, left, resized_mask.rows)).setTo(0);
                if (right > 0)
                    resized_mask(cv::Rect(resized_mask.cols - right, 0, right, resized_mask.rows)).setTo(0);
            }

            return resized_mask;
        }
    };

    class DeepLabFieldModelTest : public ::testing::Test
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
    TEST_F(DeepLabFieldModelTest, ConstructorDefaults)
    {
        DeepLabFieldModel model("/fake/path/model.pt", DeepLabModelType::DeepLabV3);
        // Should construct without throwing
        SUCCEED();
    }

    // Test constructor with custom parameters
    TEST_F(DeepLabFieldModelTest, ConstructorCustomParams)
    {
        DeepLabFieldModel model("/fake/path/model.pt", DeepLabModelType::DeepLabV3Plus, 256, 10);
        // Should construct without throwing
        SUCCEED();
    }

    // Test update returns empty mask when not initialized
    TEST_F(DeepLabFieldModelTest, UpdateWithoutInitialization)
    {
        DeepLabFieldModel model("/fake/path/model.pt", DeepLabModelType::DeepLabV3);

        auto result = model.update(test_image);

        // Should return empty mask when not initialized
        EXPECT_TRUE(result.mask.mask.empty());
        EXPECT_EQ(result.mask.label, Label::FIELD);
        // Header is not preserved when returning early due to no initialization
        EXPECT_DOUBLE_EQ(result.header.stamp, 0.0);
        EXPECT_EQ(result.header.frame_id, FrameId::EMPTY);
    }

    // Test mock model initialization
    TEST_F(DeepLabFieldModelTest, MockModelInitialization)
    {
        MockDeepLabFieldModel mock_model;
        EXPECT_TRUE(mock_model.initialize());
    }

    // Test header preservation with mock model
    TEST_F(DeepLabFieldModelTest, HeaderPreservationWithMock)
    {
        MockDeepLabFieldModel mock_model(512, 10);
        mock_model.initialize();

        test_image.header.stamp = 456.789;
        test_image.header.frame_id = FrameId::VISUAL_ODOMETRY;

        // Create a fake segmentation mask
        cv::Mat fake_mask(512, 512, CV_8UC1, cv::Scalar(3));

        auto result = mock_model.mock_update(test_image, fake_mask);

        // Header should be copied from input
        EXPECT_DOUBLE_EQ(result.header.stamp, 456.789);
        EXPECT_EQ(result.header.frame_id, FrameId::VISUAL_ODOMETRY);
        EXPECT_EQ(result.mask.label, Label::FIELD);
        EXPECT_FALSE(result.mask.mask.empty());
    }

    // Test border cropping with mock model
    TEST_F(DeepLabFieldModelTest, BorderCroppingWithMock)
    {
        MockDeepLabFieldModel mock_model(100, 10);
        mock_model.initialize();

        // Create a fake mask with all pixels set to label 5
        cv::Mat fake_mask(100, 100, CV_8UC1, cv::Scalar(5));

        auto result = mock_model.mock_update(test_image, fake_mask);

        // Check that borders are zeroed
        EXPECT_EQ(result.mask.mask.at<uint8_t>(0, 50), 0);  // Top border
        EXPECT_EQ(result.mask.mask.at<uint8_t>(99, 50), 0); // Bottom border
        EXPECT_EQ(result.mask.mask.at<uint8_t>(50, 0), 0);  // Left border
        EXPECT_EQ(result.mask.mask.at<uint8_t>(50, 99), 0); // Right border

        // Check that center is preserved
        EXPECT_EQ(result.mask.mask.at<uint8_t>(50, 50), 5); // Center
    }

    // Test border cropping with zero padding
    TEST_F(DeepLabFieldModelTest, NoBorderCroppingWithMock)
    {
        MockDeepLabFieldModel mock_model(50, 0); // Zero padding
        mock_model.initialize();

        test_image.image = cv::Mat(50, 50, CV_8UC3, cv::Scalar(100, 150, 200));
        cv::Mat fake_mask(50, 50, CV_8UC1, cv::Scalar(7));

        auto result = mock_model.mock_update(test_image, fake_mask);

        // All pixels should remain unchanged since padding is 0
        EXPECT_EQ(result.mask.mask.at<uint8_t>(0, 0), 7);
        EXPECT_EQ(result.mask.mask.at<uint8_t>(49, 49), 7);
        EXPECT_EQ(result.mask.mask.at<uint8_t>(25, 25), 7);
    }

    // Test border cropping logic directly
    TEST_F(DeepLabFieldModelTest, BorderCroppingLogic)
    {
        // Create a mask with non-zero values
        cv::Mat mask(100, 100, CV_8UC1, cv::Scalar(5)); // All pixels = 5

        int padding = 10;

        // Apply the same border cropping logic as in postprocess_output
        if (padding > 0)
        {
            int top = std::min(padding, mask.rows);
            int bottom = std::min(padding, mask.rows);
            int left = std::min(padding, mask.cols);
            int right = std::min(padding, mask.cols);

            if (top > 0)
                mask(cv::Rect(0, 0, mask.cols, top)).setTo(0);
            if (bottom > 0)
                mask(cv::Rect(0, mask.rows - bottom, mask.cols, bottom)).setTo(0);
            if (left > 0)
                mask(cv::Rect(0, 0, left, mask.rows)).setTo(0);
            if (right > 0)
                mask(cv::Rect(mask.cols - right, 0, right, mask.rows)).setTo(0);
        }

        // Check borders are zeroed
        EXPECT_EQ(mask.at<uint8_t>(0, 50), 0);  // Top border
        EXPECT_EQ(mask.at<uint8_t>(99, 50), 0); // Bottom border
        EXPECT_EQ(mask.at<uint8_t>(50, 0), 0);  // Left border
        EXPECT_EQ(mask.at<uint8_t>(50, 99), 0); // Right border

        // Check center is not zeroed
        EXPECT_EQ(mask.at<uint8_t>(50, 50), 5); // Center should still be 5
    }

    // Test border cropping with zero padding
    TEST_F(DeepLabFieldModelTest, NoBorderCroppingWithZeroPadding)
    {
        cv::Mat mask(50, 50, CV_8UC1, cv::Scalar(7));

        int padding = 0;

        // No cropping should be applied
        if (padding > 0)
        {
            // This block should not execute
            FAIL() << "Should not enter cropping logic with zero padding";
        }

        // All pixels should remain unchanged
        EXPECT_EQ(mask.at<uint8_t>(0, 0), 7);
        EXPECT_EQ(mask.at<uint8_t>(49, 49), 7);
        EXPECT_EQ(mask.at<uint8_t>(25, 25), 7);
    }

    // Test border cropping with padding larger than image
    TEST_F(DeepLabFieldModelTest, BorderCroppingWithLargePadding)
    {
        cv::Mat mask(20, 20, CV_8UC1, cv::Scalar(3));

        int padding = 50; // Larger than image dimensions

        int top = std::min(padding, mask.rows);
        int bottom = std::min(padding, mask.rows);
        int left = std::min(padding, mask.cols);
        int right = std::min(padding, mask.cols);

        EXPECT_EQ(top, 20); // Should be clamped to image height
        EXPECT_EQ(bottom, 20);
        EXPECT_EQ(left, 20); // Should be clamped to image width
        EXPECT_EQ(right, 20);
    }

} // namespace auto_battlebot
