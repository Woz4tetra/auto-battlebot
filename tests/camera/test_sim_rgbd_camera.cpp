#include <gtest/gtest.h>
#include "rgbd_camera/sim_rgbd_camera.hpp"
#include "data_structures.hpp"
#include "shared_memory/shared_memory_writer.hpp"

namespace auto_battlebot
{
    const int IMAGE_WIDTH = 1280;
    const int IMAGE_HEIGHT = 720;

    class MockUnityFrameWriter
    {
    public:
        MockUnityFrameWriter(size_t file_size) : writer_("auto_battlebot_frame", file_size),
                                                 width_(IMAGE_WIDTH),
                                                 height_(IMAGE_HEIGHT)
        {
        }

        bool initialize() { return writer_.open(true); }

        void write_test_frame(uint64_t frame_id, double timestamp)
        {
            // Calculate offsets like Unity's FrameHeader.Create does
            int rgb_size = width_ * height_ * 3;
            int depth_size = width_ * height_ * sizeof(float);

            FrameHeader header{};
            header.frame_id = frame_id;
            header.timestamp = timestamp;
            header.width = width_;
            header.height = height_;
            header.rgb_offset = FrameHeader::SIZE;  // RGB starts after header
            header.depth_offset = header.rgb_offset + rgb_size;
            header.pose_offset = header.depth_offset + depth_size;
            header.active_buffer = 0;  // Not using double buffering in test
            header.fx = 500.0;
            header.fy = 500.0;
            header.cx = width_ / 2.0;
            header.cy = height_ / 2.0;
            header.k1 = 0.1;
            header.k2 = 0.01;
            header.p1 = 0.001;
            header.p2 = 0.001;
            header.k3 = 0.0001;

            writer_.write_at(0, header);

            // Write dummy RGB data (e.g., all blue in BGR format)
            std::vector<std::byte> rgb(rgb_size, std::byte(0));
            for (int i = 0; i < width_ * height_; i++)
            {
                rgb[i * 3 + 0] = std::byte(255); // B
            }
            writer_.write_bytes(header.rgb_offset, rgb.data(), rgb.size());

            // Write dummy depth (1.5m everywhere) - must be float array
            std::vector<float> depth(width_ * height_, 1.5f);
            writer_.write_bytes(header.depth_offset, reinterpret_cast<std::byte *>(depth.data()), depth.size() * sizeof(float));

            // Write identity pose
            double pose[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
            writer_.write_bytes(header.pose_offset, reinterpret_cast<std::byte *>(pose), sizeof(pose));
        }

    private:
        int width_, height_;
        SharedMemoryWriter writer_;
    };

    TEST(SimRgbdCameraTest, NoMemoryLeaks)
    {
        // Create and destroy multiple times to catch leaks
        for (int i = 0; i < 10; i++)
        {
            SimRgbdCameraConfiguration config;
            SimRgbdCamera camera(config);
            camera.initialize();
            // camera goes out of scope - destructor should clean up
        }
    }

    TEST(SimRgbdCameraIntegration, ReadsFrameFromMockUnity)
    {
        // Arrange - Mock Unity writing a frame
        size_t buffer_size = get_simulation_frame_size(IMAGE_WIDTH, IMAGE_HEIGHT);
        MockUnityFrameWriter mock_unity(buffer_size);
        ASSERT_TRUE(mock_unity.initialize());
        mock_unity.write_test_frame(42, 1.5);

        // Act - SimRgbdCamera reads it (disable double buffering for simpler test)
        SimRgbdCameraConfiguration config;
        config.width = IMAGE_WIDTH;
        config.height = IMAGE_HEIGHT;
        config.enable_double_buffering = false;
        SimRgbdCamera camera(config);
        ASSERT_TRUE(camera.initialize());

        CameraData data;
        bool success = camera.get(data, true);

        // Assert - Verify data matches what mock wrote
        EXPECT_TRUE(success);
        EXPECT_NEAR(1.5, data.camera_info.header.stamp, 0.001);
        EXPECT_EQ(IMAGE_WIDTH, data.rgb.image.cols);
        EXPECT_EQ(IMAGE_HEIGHT, data.rgb.image.rows);
        EXPECT_NEAR(1.5f, data.depth.image.at<float>(IMAGE_HEIGHT / 2, IMAGE_WIDTH / 2), 0.01f); // Center pixel depth
        EXPECT_EQ(255, data.rgb.image.at<cv::Vec3b>(0, 0)[0]);  // Blue channel

        // Verify intrinsics were read correctly
        EXPECT_NEAR(500.0, data.camera_info.intrinsics.at<double>(0, 0), 0.001);  // fx
        EXPECT_NEAR(500.0, data.camera_info.intrinsics.at<double>(1, 1), 0.001);  // fy
        EXPECT_NEAR(IMAGE_WIDTH / 2.0, data.camera_info.intrinsics.at<double>(0, 2), 0.001);  // cx
        EXPECT_NEAR(IMAGE_HEIGHT / 2.0, data.camera_info.intrinsics.at<double>(1, 2), 0.001);  // cy

        // Verify distortion coefficients
        EXPECT_NEAR(0.1, data.camera_info.distortion.at<double>(0, 0), 0.001);     // k1
        EXPECT_NEAR(0.01, data.camera_info.distortion.at<double>(0, 1), 0.001);    // k2
        EXPECT_NEAR(0.001, data.camera_info.distortion.at<double>(0, 2), 0.0001);  // p1
        EXPECT_NEAR(0.001, data.camera_info.distortion.at<double>(0, 3), 0.0001);  // p2
        EXPECT_NEAR(0.0001, data.camera_info.distortion.at<double>(0, 4), 0.00001); // k3
    }

} // namespace auto_battlebot
