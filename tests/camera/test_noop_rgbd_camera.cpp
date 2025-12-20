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
        
        CameraData data;
        EXPECT_FALSE(camera.get(data));
    }

    TEST_F(NoopRgbdCameraTest, MultipleCallsConsistent)
    {
        // Verify behavior is consistent across multiple calls
        EXPECT_TRUE(camera.initialize());
        EXPECT_TRUE(camera.initialize());
        
        EXPECT_TRUE(camera.update());
        EXPECT_TRUE(camera.update());
        
        CameraData data;
        EXPECT_FALSE(camera.get(data));
        EXPECT_FALSE(camera.get(data));
        
        EXPECT_FALSE(camera.should_close());
    }

    TEST_F(NoopRgbdCameraTest, GetDoesNotModifyData)
    {
        // Verify get() doesn't modify the provided data structure
        CameraData data;
        data.tf_visodom_from_camera.header.timestamp = 123.456;
        data.camera_info.width = 640;
        data.camera_info.height = 480;
        
        camera.get(data);
        
        EXPECT_EQ(data.tf_visodom_from_camera.header.timestamp, 123.456);
        EXPECT_EQ(data.camera_info.width, 640);
        EXPECT_EQ(data.camera_info.height, 480);
    }

} // namespace auto_battlebot
