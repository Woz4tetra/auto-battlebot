#include <gtest/gtest.h>
#include "data_structures/keypoint.hpp"

namespace auto_battlebot
{
    TEST(KeypointTest, DefaultConstruction)
    {
        Keypoint kp;

        EXPECT_EQ(kp.label, "");
        EXPECT_EQ(kp.keypoint_label, "");
        EXPECT_FLOAT_EQ(kp.x, 0.0f);
        EXPECT_FLOAT_EQ(kp.y, 0.0f);
    }

    TEST(KeypointTest, ValueAssignment)
    {
        Keypoint kp;
        kp.label = "robot_1";
        kp.keypoint_label = "center";
        kp.x = 320.5f;
        kp.y = 240.5f;

        EXPECT_EQ(kp.label, "robot_1");
        EXPECT_EQ(kp.keypoint_label, "center");
        EXPECT_FLOAT_EQ(kp.x, 320.5f);
        EXPECT_FLOAT_EQ(kp.y, 240.5f);
    }

    TEST(KeypointsStampedTest, DefaultConstruction)
    {
        KeypointsStamped stamped;

        EXPECT_EQ(stamped.header.stamp, 0.0);
        EXPECT_EQ(stamped.header.frame_id, FrameId::EMPTY);
        EXPECT_TRUE(stamped.keypoints.empty());
    }

    TEST(KeypointsStampedTest, AddKeypoint)
    {
        KeypointsStamped stamped;
        stamped.header.stamp = 123.456;
        stamped.header.frame_id = FrameId::CAMERA;

        Keypoint kp;
        kp.label = "robot_1";
        kp.keypoint_label = "center";
        kp.x = 320.0f;
        kp.y = 240.0f;

        stamped.keypoints.push_back(kp);

        EXPECT_EQ(stamped.keypoints.size(), 1);
        EXPECT_EQ(stamped.keypoints[0].label, "robot_1");
        EXPECT_EQ(stamped.keypoints[0].keypoint_label, "center");
        EXPECT_FLOAT_EQ(stamped.keypoints[0].x, 320.0f);
        EXPECT_FLOAT_EQ(stamped.keypoints[0].y, 240.0f);
    }

    TEST(KeypointsStampedTest, MultipleKeypoints)
    {
        KeypointsStamped stamped;

        Keypoint kp1;
        kp1.label = "robot_1";
        kp1.keypoint_label = "center";
        kp1.x = 320.0f;
        kp1.y = 240.0f;

        Keypoint kp2;
        kp2.label = "robot_1";
        kp2.keypoint_label = "front";
        kp2.x = 330.0f;
        kp2.y = 240.0f;

        stamped.keypoints.push_back(kp1);
        stamped.keypoints.push_back(kp2);

        EXPECT_EQ(stamped.keypoints.size(), 2);
        EXPECT_EQ(stamped.keypoints[0].keypoint_label, "center");
        EXPECT_EQ(stamped.keypoints[1].keypoint_label, "front");
    }

    TEST(KeypointsStampedTest, CameraInfoIntegration)
    {
        KeypointsStamped stamped;
        stamped.camera_info.width = 640;
        stamped.camera_info.height = 480;

        EXPECT_EQ(stamped.camera_info.width, 640);
        EXPECT_EQ(stamped.camera_info.height, 480);
    }
} // namespace auto_battlebot
