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
        EXPECT_FALSE(camera.should_close());

        CameraData data;
        EXPECT_TRUE(camera.get(data));
        // NoopRgbdCamera returns an empty CameraData
        EXPECT_EQ(data.camera_info.width, 0);
    }

    TEST_F(NoopRgbdCameraTest, MultipleCallsConsistent)
    {
        // Verify behavior is consistent across multiple calls
        EXPECT_TRUE(camera.initialize());
        EXPECT_TRUE(camera.initialize());

        CameraData data1;
        EXPECT_TRUE(camera.get(data1));
        CameraData data2;
        EXPECT_TRUE(camera.get(data2));
        // Both should return the same empty data
        EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);

        EXPECT_FALSE(camera.should_close());
    }

    TEST_F(NoopRgbdCameraTest, ShouldCloseAlwaysFalse)
    {
        // Verify should_close() always returns false
        EXPECT_FALSE(camera.should_close());
        camera.initialize();
        EXPECT_FALSE(camera.should_close());
        CameraData data;
        camera.get(data);
        EXPECT_FALSE(camera.should_close());
    }

    TEST_F(NoopRgbdCameraTest, GetReturnsConstReference)
    {
        // Verify get() populates the provided data reference
        CameraData data;
        EXPECT_TRUE(camera.get(data));

        // NoopRgbdCamera returns an empty/default CameraData
        EXPECT_EQ(data.camera_info.width, 0);
        EXPECT_EQ(data.camera_info.height, 0);
        EXPECT_EQ(data.tf_visodom_from_camera.header.stamp, 0.0);
    }

} // namespace auto_battlebot
