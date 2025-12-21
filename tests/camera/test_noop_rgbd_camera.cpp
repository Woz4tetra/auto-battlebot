#include <gtest/gtest.h>
#include "rgbd_camera/noop_rgbd_camera.hpp"
#include "data_structures.hpp"

namespace auto_battlebot
{
    class NoopRgbdCameraTest : public ::testing::Test
    {
    protected:
        NoopRgbdCamera camera;
    };

    TEST_F(NoopRgbdCameraTest, BasicBehavior)
    {
        // Verify all operations return expected values
        EXPECT_TRUE(camera.initialize());
        EXPECT_TRUE(camera.update());
        EXPECT_FALSE(camera.should_close());

        const CameraData &data = camera.get();
        // NoopRgbdCamera returns an empty CameraData
        EXPECT_EQ(data.camera_info.width, 0);
    }

    TEST_F(NoopRgbdCameraTest, MultipleCallsConsistent)
    {
        // Verify behavior is consistent across multiple calls
        EXPECT_TRUE(camera.initialize());
        EXPECT_TRUE(camera.initialize());

        EXPECT_TRUE(camera.update());
        EXPECT_TRUE(camera.update());

        const CameraData &data1 = camera.get();
        const CameraData &data2 = camera.get();
        // Both should return the same empty data
        EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);

        EXPECT_FALSE(camera.should_close());
    }

    TEST_F(NoopRgbdCameraTest, GetReturnsConstReference)
    {
        // Verify get() returns a const reference to internal data
        const CameraData &data = camera.get();

        // NoopRgbdCamera returns an empty/default CameraData
        EXPECT_EQ(data.camera_info.width, 0);
        EXPECT_EQ(data.camera_info.height, 0);
        EXPECT_EQ(data.tf_visodom_from_camera.header.stamp, 0.0);
    }

} // namespace auto_battlebot
