#include <gtest/gtest.h>

#include "data_structures.hpp"
#include "rgbd_camera/noop_rgbd_camera.hpp"

namespace auto_battlebot {
class NoopRgbdCameraTest : public ::testing::Test {
   protected:
    NoopRgbdCamera camera;
};

TEST_F(NoopRgbdCameraTest, BasicBehavior) {
    EXPECT_TRUE(camera.initialize());
    EXPECT_FALSE(camera.should_close());

    CameraData data;
    EXPECT_TRUE(camera.get(data));
    EXPECT_EQ(data.camera_info.width, 0);
}

TEST_F(NoopRgbdCameraTest, MultipleCallsConsistent) {
    EXPECT_TRUE(camera.initialize());
    EXPECT_TRUE(camera.initialize());

    CameraData data1;
    EXPECT_TRUE(camera.get(data1));
    CameraData data2;
    EXPECT_TRUE(camera.get(data2));
    EXPECT_EQ(data1.camera_info.width, data2.camera_info.width);

    EXPECT_FALSE(camera.should_close());
}

TEST_F(NoopRgbdCameraTest, ShouldCloseAlwaysFalse) {
    EXPECT_FALSE(camera.should_close());
    camera.initialize();
    EXPECT_FALSE(camera.should_close());
    CameraData data;
    camera.get(data);
    EXPECT_FALSE(camera.should_close());
}

TEST_F(NoopRgbdCameraTest, GetReturnsConstReference) {
    CameraData data;
    EXPECT_TRUE(camera.get(data));

    EXPECT_EQ(data.camera_info.width, 0);
    EXPECT_EQ(data.camera_info.height, 0);
    EXPECT_EQ(data.tf_visodom_from_camera.header.stamp, 0.0);
}

}  // namespace auto_battlebot
